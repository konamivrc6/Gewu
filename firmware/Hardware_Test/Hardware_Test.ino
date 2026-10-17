/*
 * Gewu 上电自检
 * ==============
 *
 * 给刚焊好的板子做一次通电验证，一次验四件事：
 *   1. MAX7219 的 8×8 点阵（含经 SN74HCT125 的电平转换）
 *   2. IMU（模块实测为 MPU6500）的 I2C 总线
 *   3. 串口链路
 *   4. 两个微动开关 + 编码器（含按压）共五路输入
 *
 * 预期现象：
 *   - 点阵以最低亮度一颗一颗跑马灯（一圈约 3.8 秒）。亮度和跑马灯周期都只是上电初值，可以用串口命令临时改
 *   - 串口每 500 ms 一行 IMU 数据（加速度 / 角速度 / 温度）
 *   - 按开关 / 转编码器时当场插一行事件，前缀 [SW2] / [SW3] / [ENCSW] / [ENC]
 *
 * 串口命令：上电默认跑马灯，敲 ? 打印完整清单。点灯用 p x y（不清屏、可连着画），清屏用 c，全亮查死点用 a，看行列走向用 h y / v x，逐行逐列扫用 s row / s col / s off，切回跑马灯用 m，改亮度用 i，看屏幕系重力用 g。命令不区分大小写、以回车结束。
 * 重力模式 g 是标定的落地：IMU 读数先按六面标定与零偏修正，再换到屏幕坐标系，最后把平面内的重力方向画成一颗灯。
 * 那条 500 ms 的 IMU 数据行是【未修正】的原始读数，故意不动——它是标定的输入；修正后的量由 GRAV 行给。两者的推导与判据见《标定.md》第二、三、五节。
 * 逻辑坐标是【数学系】：原点在左下角，x 向右、y 向上，各 0~7。这与 MAX7219 库的行主序 (row, col) 差一个转置，换算只发生在 logicalToHw() 一处。
 * 加这一路的用途是标定点阵屏：MX_HW_TYPE 与 MX_ROTATION 是本草图仅有的两个必须现场试的常量，靠跑马灯只能反推走向，而且跑马灯跑在最暗档、环境光下根本看不清。
 *
 * 失败对照排查：见《硬件接线.md》第七节「易错点」。本文件里凡是非显然的地方都注明了对应的条目号。
 *
 * 依赖（Arduino IDE 库管理器里装）：
 *   MD_MAX72XX      by majicDesigns
 *
 * IMU 一个库都不用，直接拿 Wire 读写寄存器，原因见下面「I2C 原语」一节的开头。
 *
 * 板型设置：AirM2M CORE ESP32C3，Flash Mode = DIO，USB CDC On Boot = Disabled（经典版走板载 USB 转串口芯片）。
 * 串口监视器波特率 921600。
 *
 * 注意：开机横幅只在复位后打印一次。若打开串口监视器时已经错过了，按一下板上 RST 再看。
 */

#include <Wire.h>
#include <MD_MAX72xx.h>
#include <math.h>      // sqrtf / atan2f / lroundf / fabsf，重力那一层要用

// 这台板子实测出来的档案：点阵模块型号、编码器每格跳变数、IMU 量程与九个标定常数。
// 单独成文件不是为了好看，是为了让「测出来的」与「我调的」在物理上分开——见该文件开头的分组规则。
// 放在 common/ 而不是草图目录里：以后的正式草图要 includes 同一份，而不是各自复制一份再慢慢分叉。
#include "../common/BoardProfile.h"

// ============================ 引脚定义 ============================
// 全部以《硬件接线.md》4.3 节「本项目分配」为准，并经 .epro2 网表与 Gerber 里的 FlyingProbeTesting.json 双向核对过。
// 提醒：合宙官方引脚表与立创 EDA 原理图的脚号是镜像的（见该文档 4.2 节），改这里时按 GPIO 号改，不要按原理图脚号改。

static const uint8_t PIN_DIN    = 2;   // GPIO2  → U2 1A(pin2) → 1Y(pin3) → 模块 DIN
static const uint8_t PIN_CLK    = 8;   // GPIO8  → U2 2A(pin5) → 2Y(pin6) → 模块 CLK
static const uint8_t PIN_CS     = 10;  // GPIO10 → U2 3A(pin9) → 3Y(pin8) → 模块 CS
static const uint8_t PIN_SDA    = 4;   // I2C 数据，直连 IMU，4.7k 上拉到 3.3V
static const uint8_t PIN_SCL    = 5;   // I2C 时钟，直连 IMU，4.7k 上拉到 3.3V
static const uint8_t PIN_SW2    = 6;   // 微动 1，4.7k 上拉 + 100nF 到地，低电平有效
static const uint8_t PIN_SW3    = 7;   // 微动 2，同上
static const uint8_t PIN_ENCSW  = 3;   // 编码器按压，同上
static const uint8_t PIN_ENCA   = 1;   // 编码器 A 相，4.7k 上拉，【没有】电容
static const uint8_t PIN_ENCB   = 0;   // 编码器 B 相，同上

// ============================ 可调参数 ============================

// 这一节只剩「我选的参数」。点阵模块档案、编码器每格跳变数、IMU 量程与九个标定常数都是**实测结论**，
// 搬到了 BoardProfile.h —— 那条线是语义的：改了它等于伪造标定（而且错得静默），
// 而这里的值改了只是「我想让它跑快一点 / 亮一点」。别把两边混回去。

static const uint8_t  MX_INTENSITY   = 0;      // 【上电初值】0 = 最暗（MAX7219 的 1/32 占空比），15 = 最亮。写 0 【不是】关屏，是最暗的「亮」。串口 i 命令能临时改，复位后回到这里
static const uint32_t CHASE_MS       = 60;     // 【上电初值】跑马灯每格停留时间；64 格 × 60 ms ≈ 3.8 秒一圈。串口 m 命令能临时改
static const uint32_t SCAN_MS        = 250;    // 逐行 / 逐列扫描每步的间隔。固定值，不给运行时调节口子——这条命令是标定用的，扫快了反而看不清走了哪一行
static const uint32_t PRINT_MS       = 500;    // IMU 数据输出间隔。故意放慢，免得把按键事件冲得找不着
static const uint32_t DEBOUNCE_MS    = 15;     // 按键软件消抖。硬件已有 100nF（6.3 节），这里只需补一道稳定确认
static const uint32_t IMU_RETRY_MS   = 5000;   // 传感器没就绪时的重试间隔

static const uint32_t I2C_HZ         = 400000; // 4.7k 上拉在 400kHz 下上升时间合规（5.4 节）；降速可改 100000

// IMU 认出来的型号。WHO_AM_I 是唯一可信的判据：0x68 = MPU6050，0x70 = MPU6500（本板实测）。
// 认型号不只是为了打印好看——温度换算公式两张图不一样，所以型号必须在读数据之前就知道。
enum ImuKind : uint8_t { IMU_NONE = 0, IMU_6050 = 1, IMU_6500 = 2 };

// 重力显示模式的重画间隔。50 ms = 20 Hz，独立于 PRINT_MS：2 Hz 的画面看着是卡顿的，20 Hz 才像在动，
// 而传感器本身跑 200 Hz，20 Hz 读完全在余量内。
static const uint32_t GRAVITY_MS   = 50;
// 平面内重力 → 像素的系数。5 让 sin(倾角) ≈ 0.7（约 45°）时灯正好走到边缘。
static const float    GRAVITY_PIX_PER_G = 5.0f;

// ============================ 对象与状态 ============================

// 构造函数必须用「五个参数」的软件时序版本（HW_TYPE, dataPin, clkPin, csPin, numDevices）。
// 另一个三参数版本（HW_TYPE, csPin, numDevices）走硬件 SPI，会去用 MCU 默认 SPI 引脚，而本板的 DIN/CLK 是 GPIO2/GPIO8，对不上。
MD_MAX72XX     mx(MX_HW_TYPE, PIN_DIN, PIN_CLK, PIN_CS, MX_DEVICES);

static uint32_t lastChaseMs = 0;
static uint32_t lastPrintMs = 0;
static uint8_t  chaseIdx    = 0;       // 0~63，跑马灯当前位置

// 「谁在画屏幕」只能有一个答案，所以用枚举而不是两个 bool——两个 bool 会留下「都真」这种没法解释的状态。
// 切模式的规则只有一条：进静态模式一律先清屏，从空白画布开始，省掉「这次该不该清」的判断分支。
enum DispMode : uint8_t { MODE_MARQUEE = 0, MODE_MANUAL = 1, MODE_SCAN = 2, MODE_GRAVITY = 3 };
static uint8_t  dispMode    = MODE_MARQUEE;   // 上电就是跑马灯

// 运行时镜像。上面那几个全大写常量一个都没动，语义是「上电初值」；真正被读的是这两个小驼峰变量。
// 分成两份不是啰嗦：这样「串口改的值不持久、复位即回常量」在代码里一眼看得见，标定文档引用 MX_INTENSITY 时也仍然对得上。
static uint8_t  mxIntensity = MX_INTENSITY;
static uint32_t chaseMs     = CHASE_MS;

static bool     scanByRow   = true;    // 扫描方向：真 = 逐行，假 = 逐列
static uint8_t  scanIdx     = 0;       // 扫到第几行 / 第几列
static uint32_t lastScanMs  = 0;

static uint32_t lastGravityMs = 0;     // 重力模式的节拍戳，每 GRAVITY_MS 重画一帧

// 串口行缓冲。32 字节对「p 3 5」这种长度绰绰有余，超了就整行丢掉——见 pollSerial 里 lineOverflow 的用法。
#define SERIAL_LINE_MAX 32
static char     lineBuf[SERIAL_LINE_MAX];
static uint8_t  lineLen      = 0;
static bool     lineOverflow = false;  // 本行已超长，静默丢到行尾、只报一次错。没有这个标志，被截断的尾巴会被当成下一条新命令执行

static bool     imuReady    = false;
static uint32_t imuRetryMs  = 0;
static uint8_t  imuKind     = IMU_NONE;   // 认出来的型号，决定温度用哪条换算公式

// 一次读回的七个原始通道。做成结构体是因为「读」与「打印」必须能各自独立：imuPrint() 每 500 ms 打一次，
// 而重力模式每 50 ms 只读不打印，靠的是同一个 imuRead()。
// 【位置有讲究】这个结构体必须出现在任何用到它的函数之前，不能就近放在 IMU 一节里——
// Arduino 的预处理会把函数原型自动插到文件靠前的位置，原型里的 ImuRaw 那时还不认识，报的是
// 「'ImuRaw' was not declared in this scope」加上一句莫名其妙的「redeclared as different kind of entity」。
struct ImuRaw {
  int16_t acc[3];    // x y z，顺序与数据行一致
  int16_t gyro[3];
  int16_t tRaw;
};

// 五路输入全部低电平有效：硬件已各配 4.7k 上拉到 3.3V（5.5 / 5.6 / 6.1 节），按下或转到格点就接地。
struct ButtonState {
  uint8_t     pin;
  const char *tag;
  bool        stable;      // 已确认的稳定状态，true = 按下
  bool        lastRaw;     // 上次读到的原始状态
  uint32_t    lastEdgeMs;  // 原始状态上次变化的时间
};

static ButtonState buttons[] = {
  { PIN_SW2,   "[SW2]",   false, false, 0 },
  { PIN_SW3,   "[SW3]",   false, false, 0 },
  { PIN_ENCSW, "[ENCSW]", false, false, 0 },
};
static const uint8_t BUTTON_COUNT = sizeof(buttons) / sizeof(buttons[0]);

// 正交解码状态表：索引 = 上一次状态 × 4 + 本次状态，状态值 = (A << 1) | B。
// 查表的本质是「只接受相邻的合法跳变」，非法中间态返回 0 被丢弃，所以这张表本身即抗抖，不需要再叠延时。
// 编码器 A/B 明确不能加电容（易错点 16：并 100nF 后时间常数 4.5 ms，会把脉冲吃掉），所以这里只能靠查表。
// 方向反了就把查表结果取负（见 scanEncoder 里那一行），或把上面的 A/B 两位对调；【不要】只对调一对行——本表反对称，只换 00↔11 两行后 M[01][00] 仍是 +1，而正确取反应为 -1，症状是某个方向丢格、另一个方向多格。
static const int8_t ENC_TABLE[16] = {
  //  本次:  00   01   10   11
             0,  -1,  +1,   0,   // 上次 00
            +1,   0,   0,  -1,   // 上次 01
            -1,   0,   0,  +1,   // 上次 10
             0,  +1,  -1,   0,   // 上次 11
};

static int8_t  encPrev   = 0;   // 上次的 (A<<1)|B
static int8_t  encAccum  = 0;   // 校正交跳变累加，满 ±ENC_STEPS_PER_DETENT 才报一格
static int32_t encDetent = 0;   // 累计格数

// ============================ 小工具 ============================

// 事件行统一格式：标签左对齐 8 列 + 内容左对齐 8 列 + 毫秒时间戳。
// 标签打头是为了能 grep '^\[' 一把捞出所有事件，而 grep '^MPU' 拿数据行；带时间戳是为了看出触点抖动——
// 连按一下却冒出十几个间隔几十毫秒的沿，就是消抖没压住。
static void eventLine(const char *tag, const char *detail) {
  Serial.printf("%-8s%-8s@%lumillis\n", tag, detail, (unsigned long)millis());
}

// 编码器事件：方向和累计格数一起打。方向必须打出来——查表法的符号约定取决于 A/B 哪个接在 GPIO1，
// 万一反了，看 dir 的符号立刻能发现，不用去猜。
static void encoderEvent(int dir, int32_t detent) {
  Serial.printf("[ENC]    dir=%+d   detent=%d   @%lumillis\n",
                dir, (int)detent, (unsigned long)millis());
}

// ============================ I2C 原语 ============================

// ---- 本文件用到的 IMU 寄存器。两张寄存器图里这些地址完全一致，差异只在 ACCEL_CONFIG2 等少数几个 ----
static const uint8_t REG_SMPLRT_DIV    = 0x19;
static const uint8_t REG_CONFIG        = 0x1A;
static const uint8_t REG_GYRO_CONFIG   = 0x1B;
static const uint8_t REG_ACCEL_CONFIG  = 0x1C;
static const uint8_t REG_ACCEL_CONFIG2 = 0x1D;   // 只有 MPU6500 有：加速度计单独的 DLPF
static const uint8_t REG_ACCEL_OUT     = 0x3B;   // 从这里连着读 14 字节 = accel(6) + temp(2) + gyro(6)
static const uint8_t REG_PWR_MGMT_1    = 0x6B;
static const uint8_t REG_PWR_MGMT_2    = 0x6C;
static const uint8_t REG_WHO_AM_I      = 0x75;

// 裸读一个 8 位寄存器。返回 0xFF 只表示这次事务失败——但 0xFF 也可能是合法寄存器值，
// 所以调用方只拿它当线索，别当成「确实读到了 0xFF」这个事实。
// endTransmission(false) 表示不发 STOP，紧接着读同一个寄存器，这是读寄存器的标准两段式写法。
static uint8_t i2cReadReg8(uint8_t devAddr, uint8_t reg) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom((uint16_t)devAddr, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

static bool i2cWriteReg8(uint8_t devAddr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// 连续读一块。一次事务读完 14 字节，而不是 14 次单字节读——后者读到的六个轴可能来自六个不同的采样时刻，
// 对刚上电的静止判定没影响，但没必要。
static bool i2cReadBlock(uint8_t devAddr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint16_t)devAddr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

static const char *whoamiHint(uint8_t who) {
  switch (who) {
    case 0x68: return "  (MPU6050/MPU6000，可用)";
    case 0x70: return "  (MPU6500，本板实测型号，可用)";
    case 0x71: return "  (MPU9250 的标识码；模块上若没有磁力计，多半是 6500 被写成了 9250 的 ID)";
    case 0x73: return "  (MPU9255 的标识码)";
    case 0x19: return "  (ICM/MPU6886 系列，本草图不支持)";
    case 0xFF: return "  (读不到——I2C 无应答：查 M3 的 VCC 是否 3.3V、AD0 是否接地、SDA/SCL 有没有误走 U2 缓冲器)";
    default:   return "  (未知标识码，本草图只认 0x68 和 0x70)";
  }
}

static const char *imuModelName(uint8_t kind) {
  switch (kind) {
    case IMU_6050: return "MPU6050";
    case IMU_6500: return "MPU6500";
    default:       return "未知 IMU";
  }
}

// 量程是写寄存器时定的，这里只把「当前配置」翻成人话给开机横幅用，读不到实际值也无所谓
static const char *accelRangeStr() {
  switch (IMU_ACCEL_AFS) {
    case 0x00: return "+-2g";
    case 0x08: return "+-4g";
    case 0x10: return "+-8g";
    case 0x18: return "+-16g";
    default:   return "?";
  }
}

static const char *gyroRangeStr() {
  switch (IMU_GYRO_FS) {
    case 0x00: return "+-250dps";
    case 0x08: return "+-500dps";
    case 0x10: return "+-1000dps";
    case 0x18: return "+-2000dps";
    default:   return "?";
  }
}

// ============================ IMU ============================
//
// 为什么一个库都不用：Adafruit_MPU6050 的 begin() 里有一句硬比较
//     if (chip_id.read() != MPU6050_DEVICE_ID) return false;   // 0x68
// 而本板实测 WHO_AM_I = 0x70（MPU6500），于是它永远返回 false，外围再怎么重试都救不回来。
// 自己读写寄存器反而更短，还顺带真的验了裸 I2C 总线——这正是自检草图该干的事。
// 两张寄存器图里本文件用到的地址完全一致，差异只在 ACCEL_CONFIG2（6500 独有）和温度公式。

static bool imuInit() {
  // 失败后【不需要】再钉一次引脚：Wire.begin() 在 core 里对已初始化的总线是空操作，既不重设引脚也不改频率。
  // 重试能救的只有「传感器上电比 MCU 慢」「接触瞬时不良」这类瞬时故障，接线错误救不回来。
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);

  uint8_t who = i2cReadReg8(MPU_ADDR, REG_WHO_AM_I);
  Serial.printf("IMU WHO_AM_I = 0x%02X%s\n", who, whoamiHint(who));

  // WHO_AM_I 是唯一可信的判据。丝印、商品页、卖家话术都不可信（本板就是按 MPU6050 买的，实测 0x70）。
  if (who == 0x68)      imuKind = IMU_6050;
  else if (who == 0x70) imuKind = IMU_6500;
  else { imuKind = IMU_NONE; return false; }

  // 唤醒：清 SLEEP 位，时钟源选 X 轴陀螺的 PLL（比内部 8MHz 振荡器稳得多）
  if (!i2cWriteReg8(MPU_ADDR, REG_PWR_MGMT_1, 0x01)) return false;
  delay(50);                                      // 等 PLL 起振
  i2cWriteReg8(MPU_ADDR, REG_PWR_MGMT_2, 0x00);   // 六轴全部退出待机；这两张图里这几位含义一致，直接写 0 是安全的

  // 这三组值写在 BoardProfile.h 里而不是就地写字面量：标定只在这一组采样率 / DLPF 下成立，
  // 它们和九个常数是一套的。放在两处的话，改了这里忘了那里不会有任何报错。
  i2cWriteReg8(MPU_ADDR, REG_SMPLRT_DIV, IMU_SMPLRT_DIV);   // 1kHz / (1+4) = 200 Hz，自检用不上 1kHz
  i2cWriteReg8(MPU_ADDR, REG_CONFIG, IMU_DLPF_CFG);         // 陀螺 DLPF 开在 44Hz 档
  if (imuKind == IMU_6500) {
    // 6500 把加速度计 DLPF 搬到了独立的 ACCEL_CONFIG2；6050 的 0x1D 是保留位，
    // 往保留位写有可能碰到出厂校准值，所以这一句必须挂在型号判断里，不能无条件执行。
    i2cWriteReg8(MPU_ADDR, REG_ACCEL_CONFIG2, IMU_ACCEL_DLPF_CFG);
  }
  i2cWriteReg8(MPU_ADDR, REG_GYRO_CONFIG,  IMU_GYRO_FS);    // FCHOICE_B 保持 0，即走上面的 DLPF
  i2cWriteReg8(MPU_ADDR, REG_ACCEL_CONFIG, IMU_ACCEL_AFS);

  // 回读一次确认写进去了。读回 0xFF 说明事务压根没成；SLEEP 位还是 1 说明唤醒失败。
  uint8_t pm = i2cReadReg8(MPU_ADDR, REG_PWR_MGMT_1);
  if (pm == 0xFF || (pm & 0x40)) {
    Serial.printf("PWR_MGMT_1 回读 = 0x%02X，SLEEP 位没清掉\n", pm);
    return false;
  }
  return true;
}

// 读 14 字节连续寄存器（accel 6 + temp 2 + gyro 6）并拆成七个 16 位大端有符号数。ImuRaw 定义在「对象与状态」一节。
// 必须先拼成 int16_t 再换算——直接用 int 拼，负数会算错。
static bool imuRead(ImuRaw *r) {
  uint8_t b[14];
  if (!i2cReadBlock(MPU_ADDR, REG_ACCEL_OUT, b, (uint8_t)sizeof(b))) return false;
  for (uint8_t i = 0; i < 3; i++) {
    r->acc[i]  = (int16_t)((b[i * 2]     << 8) | b[i * 2 + 1]);
    r->gyro[i] = (int16_t)((b[8 + i * 2] << 8) | b[9 + i * 2]);
  }
  r->tRaw = (int16_t)((b[6] << 8) | b[7]);
  return true;
}

// 修正成物理量：加速度 g、角速度 dps、温度 °C。常数在 BoardProfile.h，来历见《标定.md》第二、三节。
static bool imuReadCalibrated(float *aG, float *wDps, float *tC) {
  ImuRaw r;
  if (!imuRead(&r)) return false;
  for (uint8_t i = 0; i < 3; i++) {
    aG[i]   = ((float)r.acc[i]  - ACCEL_OFFSET[i]) / ACCEL_GAIN[i];
    wDps[i] = ((float)r.gyro[i] - GYRO_OFFSET[i])  / IMU_GYRO_LSB;
  }
  *tC = (imuKind == IMU_6500) ? ((float)r.tRaw / 333.87f + 21.0f)
                              : ((float)r.tRaw / 340.0f  + 36.53f);
  return true;
}

// 模组坐标系 → 屏幕坐标系。跟 logicalToHw() 一样，全文件只有这一处定义这套对应关系。
// 【实测结论，不是约定】《标定.md》第五节：屏幕 x̂ = 模组 −Y、ŷ = 模组 −X、n̂（出屏、朝观察者）= 模组 −Z，
// 三根轴构成右手系，与点阵那边「原点左下、x 右、y 上」直接对齐。
// 静止时加速度计测的是支持力 r，方向朝**上**，所以重力是 −r；两次取负合起来就是一次交换：
//     g_screen = (−r)·(x̂, ŷ, n̂) = (r_y, r_x, r_z)
// 也就是只把 x 与 y 对调，【不取反、不翻转】。这里没有漏负号——写这段推导就是为了下一个人不必再推一遍。
static void gravityInScreen(float ax, float ay, float az, float *gx, float *gy, float *gz) {
  *gx = ay;
  *gy = ax;
  *gz = az;
}

// 模组系 → 屏幕系，角速度版。跟 gravityInScreen() 长得像但**符号相反**，
// 而且符号相反的理由不同——所以是两个函数，不是一个带符号参数的函数：混在一起早晚有人以为那个参数是笔误。
//
// 符号从哪来：R 的列就是屏幕三根轴在模组系里的坐标，ω_screen = Rᵀ·ω_module，
// 于是三个分量各自带上「这根屏幕轴在模组里指向负方向」那一个负号：ω_screen = (−ω_Y, −ω_X, −ω_Z)。
// 重力那边之所以是正的，是因为加速度计读的是**支持力**（朝上），重力要再取一次反；角速度是直接的物理量，不取反。
//
// 还有一处比符号更容易翻车的：角速度是**赝矢量**，一般变换是 ω' = det(R)·R·ω，比普通矢量多一个行列式符号。
// 这里 det(R) = +1 —— 屏幕系与模组系**都是右手系**（x̂ × ŷ = (−Y) × (−X) = −Z = n̂），
// 所以那个符号是 +1，不需要额外处理。若哪天真把屏幕系改成镜像（左手系），
// 这个 +1 会变成 −1，而重力那边**不会有任何变化**。这两条路必须分开写、分开注释。
static void rateInScreen(float wx, float wy, float wz, float *rx, float *ry, float *rz) {
  *rx = -wy;
  *ry = -wx;
  *rz = -wz;
}

static const float RAD2DEG = 57.2957795f;   // 180 / π

// 平面内方向分八扇区给一个 ASCII 短名。按【屏幕方向】命名而不是按物理直觉：抬起右边缘时重力指向屏幕左边，
// 于是读到 "L" 一眼能对上，不用在脑子里再翻一次。约 2.9° 以内算平放，那种姿态下平面内方向根本没有意义。
static const char *gravityDirName(float gx, float gy) {
  if (sqrtf(gx * gx + gy * gy) < 0.05f) return "LEVEL";
  const float SECTOR = 6.2831853f / 8.0f;                 // 每 45° 一档
  int32_t sec = (int32_t)lroundf(atan2f(gy, gx) / SECTOR); // 0 档 = 屏幕右，逆时针为正
  sec = ((sec % 8) + 8) % 8;
  static const char *NAMES[8] = { "R", "UR", "U", "UL", "L", "DL", "D", "DR" };
  return NAMES[sec];
}

// 屏幕系传感器读数。只在重力显示模式下由主循环调用（模式判断在调用处）。
// 行首是 GRAV 而不是 MPU6，所以 tools/serial_send.py 的过滤不会把它吃掉——验收要看的正是这一行。
// 【这一行里的数全是屏幕系】：gx / gy / gz 是重力，w 是角速度，两者各自的换算见上面两个函数。
// tilt 是板面偏离水平的角，即重力与屏幕法线的夹角；它用 |平面内| 与 |gz| 的反正切直接得到，不需要先归一化。
static void gravityPrint() {
  float aG[3], wDps[3], tC;
  if (!imuReadCalibrated(aG, wDps, &tC)) {
    Serial.println(F("IMU 读 14 字节数据块失败"));
    return;
  }
  float gx, gy, gz, rx, ry, rz;
  gravityInScreen(aG[0], aG[1], aG[2], &gx, &gy, &gz);
  rateInScreen(wDps[0], wDps[1], wDps[2], &rx, &ry, &rz);
  const float inPlane = sqrtf(gx * gx + gy * gy);

  Serial.printf("GRAV gx %+.3f gy %+.3f gz %+.3f |g| %.3f tilt %4.1f dir %-5s | w %+.2f %+.2f %+.2f dps\n",
                gx, gy, gz, sqrtf(inPlane * inPlane + gz * gz),
                atan2f(inPlane, fabsf(gz)) * RAD2DEG, gravityDirName(gx, gy),
                rx, ry, rz);
}

static void imuPrint() {
  ImuRaw r;
  if (!imuRead(&r)) {
    Serial.println(F("IMU 读 14 字节数据块失败"));
    return;
  }

  // 温度换算两张图不一样，这是换型号后最容易静默出错的地方：套错公式在室温下会偏高约 15 °C，不报错也不警告。
  // 原始值一并打出来——哪天真换了芯片，一眼就能看出该用哪条公式。
  float tC = (imuKind == IMU_6500) ? (r.tRaw / 333.87f + 21.0f)
                                   : (r.tRaw / 340.0f   + 36.53f);

  // 这一行是【未修正】的原始读数，故意不动：它是标定的输入，换模块重标定要从这种日志里取数，
  // 而且 raw 一并打出来才有换芯片时的对照。修正后的物理量由重力模式的 GRAV 行给，两者可以互相核对。
  // 【格式是契约】tools/imu_stat.py 与 tools/accel_calib.py 都按这一行解析，改格式要同时改那两支。
  Serial.printf("%s ax %+.2f ay %+.2f az %+.2f g  |  gx %+.1f gy %+.1f gz %+.1f deg/s  |  %.1f C (raw %d)\n",
                imuModelName(imuKind),
                r.acc[0] / IMU_ACCEL_LSB, r.acc[1] / IMU_ACCEL_LSB, r.acc[2] / IMU_ACCEL_LSB,
                r.gyro[0] / IMU_GYRO_LSB, r.gyro[1] / IMU_GYRO_LSB, r.gyro[2] / IMU_GYRO_LSB,
                tC, r.tRaw);
}

// 数据节拍到了就调这里。没就绪时按 IMU_RETRY_MS 重试，只在状态变化时出声，免得刷屏。
static void imuTick() {
  if (imuReady) {
    imuPrint();
    return;
  }

  uint32_t now = millis();
  if (now - imuRetryMs < IMU_RETRY_MS) return;
  imuRetryMs = now;

  if (imuInit()) {
    imuReady = true;
    Serial.printf("%s OK  AD0=0x%02X  accel=%s  gyro=%s  (重试成功)\n",
                  imuModelName(imuKind), MPU_ADDR, accelRangeStr(), gyroRangeStr());
  } else {
    Serial.printf("[%8lu] IMU 仍未就绪，%lu 秒后重试\n",
                  (unsigned long)now, (unsigned long)(IMU_RETRY_MS / 1000));
  }
}

// ============================ 点阵 ============================

// 逻辑坐标 → 硬件坐标。全文件只有这一处定义坐标约定，跑马灯、扫描、串口点灯全都走它，免得几套变换各转各的。
// 逻辑坐标是【数学系】：原点左下，x 向右、y 向上，各 0~7。MAX7219 库是行主序 (row, col)、row 向下编号，
// 而本模块装好后逻辑 y 恰好向上增长（实机验过，见《标定.md》六节），于是 y 与 row 数值直接相等——
// 换算只是把两个参数换个位置，【不用】做 7 - y。这一点是实测结论，不是约定，换板或换模块要重验。
// 库（3.5.1）没有 setRotation()，MX_ROTATION 只能自己转。库里另有一个 transform(TRC) 也是顺时针 90°，但【没有采用】——
// 它就地变换、非幂等，与「点直接画进缓冲、可以累加」的用法叠在一起会互相干扰（画 A、变换、画 B、再变换，得到的是转了两次的 A）；
// 而且它是按「片」转的，MX_DEVICES 大于 1 时会变成每个模块在各自方块内自转，不是整场旋转。代数上两者是同一个映射，所以没有任何理由换。
static void logicalToHw(uint8_t x, uint8_t y, uint8_t *hr, uint8_t *hc) {
  for (uint8_t i = 0; i < MX_ROTATION; i++) {
    uint8_t t = x;
    x = (uint8_t)(7 - y);    // 顺时针 90°：(x, y) → (7 - y, x)
    y = t;
  }
  *hr = y;    // 行：库的 row 向下编号，逻辑 y 向上编号，两者数值相等
  *hc = x;    // 列
}

// 点亮一个逻辑坐标 (x, y)。故意【不】带 on 参数：串口命令只可能点亮，留个能关的口子，早晚有人写出破坏「只累加、只在 c 命令时擦除」这条规则的代码。
static void lightPoint(uint8_t x, uint8_t y) {
  uint8_t hr, hc;
  logicalToHw(x, y, &hr, &hc);
  mx.setPoint(hr, hc, true);
}

// 亮度统一从这里改。必须先自己校验：MD_MAX72XX 的 controlHardware 对越界值是静默钳到 MAX_INTENSITY 的，靠它拒绝会看不出错。
static void setIntensity(uint8_t v) {
  mxIntensity = v;
  mx.control(MD_MAX72XX::INTENSITY, mxIntensity);
}

// 切进静态模式。本来就在静态里就什么都不做，命令接着往上累加；从别的模式切过来则先清屏，
// 「进静态一律从空白画布开始」是一条规则，省掉「这次该不该清」的判断分支。
static void enterManual() {
  if (dispMode != MODE_MANUAL) {
    mx.clear();
    dispMode = MODE_MANUAL;
  }
}

// 开扫描。重置计时基准，让第一拍等满一个间隔——否则切过去会立刻跳一格，看起来像丢了一格。
static void startScan(bool byRow) {
  scanByRow  = byRow;
  scanIdx    = 0;
  dispMode   = MODE_SCAN;
  lastScanMs = millis();
}

// 逐格跑马灯：清屏 → 点亮当前那一颗 → 索引前进。逐点画而不是整行拼字节，这样 MX_HW_TYPE 的映射才真正被验证到
// （拼行字节的话，行列反了你根本看不出来）。
// 索引沿 x 走满一行再上抬一层（chaseIdx % 8 是 x、/ 8 是 y），跟改坐标约定之前画出来的物理路径逐格相同。
// 64 是 2 的幂，所以用 & 63 取模，不写 %。
static void chaseStep() {
  mx.clear();
  lightPoint(chaseIdx % 8, chaseIdx / 8);
  chaseIdx = (chaseIdx + 1) & 63;
}

// 逐行或逐列扫：清屏 → 点亮整条 → 前进。用的是同一套逻辑坐标，「行」是逻辑行——
// 判断 MX_HW_TYPE 的行列走向看的就是逻辑行落在屏上是横的还是竖的，这里与串口的 h y / v x 是同一件事的自动版。
static void scanStep() {
  mx.clear();
  for (uint8_t i = 0; i < 8; i++) {
    if (scanByRow) lightPoint(i, scanIdx);   // 逐行：y 固定，x 走满 8 格 → 一条横线
    else           lightPoint(scanIdx, i);   // 逐列：x 固定，y 走满 8 格 → 一条竖线
  }
  scanIdx = (uint8_t)((scanIdx + 1) & 7);
}

// 开重力模式。照 startScan() 的样板：清屏、置模式、重置节拍戳，让第一拍等满一个间隔。
static void startGravity() {
  mx.clear();
  dispMode      = MODE_GRAVITY;
  lastGravityMs = millis();
}

// 平面内重力分量 → 像素。中心取 3.5（8×8 没有正中像素），四舍五入后落在 (4,4)，即四个中心像素之一。
// 由此映射对 ± 并不严格对称：到右边要 gx ≥ 0.6、到左边要 gx ≤ -0.7。这是 3.5 这个中心带来的，
// 对玩具不必较真，但要知道，免得日后量出来以为是传感器的毛病。
static uint8_t gravityPixel(float v) {
  const float u = 3.5f + GRAVITY_PIX_PER_G * v;
  int n = (int)(u + (u >= 0.0f ? 0.5f : -0.5f));
  if (n < 0) n = 0;
  if (n > 7) n = 7;
  return (uint8_t)n;
}

// 把平面内的重力方向画成一颗灯：像素 = 中心 + 系数 × (gx, gy)。
// 平放时 gx ≈ gy ≈ 0，灯落在正中——所以它同时是个水平仪，而「球往哪边滚」这件事一眼就懂。
// 这条路径每 GRAVITY_MS 走一次，只读不打印（打印由 500 ms 那一拍单独负责），两条节拍互不干扰。
static void gravityStep() {
  if (!imuReady) return;       // 没就绪、或者单次 I2C 失败，都保留上一帧：清了屏会一直空白，看着像坏了
  float aG[3], wDps[3], tC;
  if (!imuReadCalibrated(aG, wDps, &tC)) return;
  float gx, gy, gz;
  gravityInScreen(aG[0], aG[1], aG[2], &gx, &gy, &gz);
  mx.clear();
  lightPoint(gravityPixel(gx), gravityPixel(gy));   // gz 是屏幕法线方向，平面内的画面用不到，但换算一律给三个分量
}

// ============================ 输入 ============================

static void scanButtons(uint32_t now) {
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    ButtonState &b = buttons[i];
    bool raw = (digitalRead(b.pin) == LOW);   // 低电平有效

    if (raw != b.lastRaw) {
      b.lastRaw    = raw;
      b.lastEdgeMs = now;
      continue;                                // 刚变化，先等它稳下来
    }
    if (raw != b.stable && (now - b.lastEdgeMs) >= DEBOUNCE_MS) {
      b.stable = raw;
      // 按下和松开两个沿都要报：易错点 12（微动开关用同侧两脚）的故障症状就是「永远导通」，
      // 只报按下的话，这种板子看起来和好板子一模一样。
      eventLine(b.tag, raw ? "press" : "release");
    }
  }
}

// 正交解码 + 每格折合一次事件。
// 关键点：EC11 带定位，转【一格】会产生 ENC_STEPS_PER_DETENT 个正交跳变，不是 1 个。
// 每个 ±1 都打一行的话，转一格刷 4 行，没法看。所以原始跳变只累加，满一格才出声。
static void scanEncoder() {
  // A/B 分两次 digitalRead 取回，两次之间若正好发生跳变，读到的状态对可能从未真实存在过。
  // 查表法会把「不相邻的状态对」判 0 丢弃，但对「看起来合法的反向相邻态」无能为力，最坏情况是格点附近偶发 ±1 计数（合 1/4 格，通常被下一格吃掉）。
  // 要彻底消除得一次性取回两位（REG_READ(GPIO_IN_REG) 之类）；自检用途下不值得，知道边界即可。
  uint8_t curr = (uint8_t)((digitalRead(PIN_ENCA) << 1) | digitalRead(PIN_ENCB));
  int8_t  step = ENC_TABLE[(encPrev << 2) | curr];
  encPrev = (int8_t)curr;
  if (step == 0) return;                       // 非法跳变，丢弃

  encAccum += step;

  while (encAccum >= ENC_STEPS_PER_DETENT) {   // 用 while 而不是 if：拨得快时一次扫描可能跨过好几格
    encAccum -= ENC_STEPS_PER_DETENT;
    encDetent++;
    encoderEvent(+1, encDetent);
  }
  while (encAccum <= -ENC_STEPS_PER_DETENT) {
    encAccum += ENC_STEPS_PER_DETENT;
    encDetent--;
    encoderEvent(-1, encDetent);
  }
}

// ============================ 串口命令 ============================
//
// 这一层的用途是标定与调试点阵屏：点名点灯、全亮查死点、整行整列看走向、运行时改亮度与跑马灯速度。
// MX_HW_TYPE 与 MX_ROTATION 已经靠它确认过（结论见《标定.md》六节），但换模块、换朝向或改板子布局时还得再走一遍，
// 所以这条路留着，不随这次标定结束而删。
//
// 三条设计约束：
//   1. 全程不阻塞。读用 Serial.read()（底层 uartReadBytes 的超时是 0）。千万别换成 readStringUntil / readBytes / parseInt
//      ——那几个都会等 _timeout（默认 1000 ms），一卡就把跑马灯和按键扫描全拖慢，文件头那句「全程没有 delay」也就成了空话。
//   2. 回显必须是 ASCII。eventLine 的 %-8s 按【字节】计宽，中文塞进去会把 @millis 那一列冲歪，而那个对齐正是它存在的理由。
//      所以 ack 走 [CMD] 紧凑形式，中文提示另起一行、不带标签；帮助文本则完全不走 eventLine。
//   3. 参数解析不成功就不改任何状态。宁可整条命令丢掉，也不能半途生效——标定场景下最怕「以为画到这里了，其实画在别处」。

// 只用一次大小写转换，为它拉一个 <ctype.h> 不划算；而且 pollSerial 已经把高位字节滤掉了，这里收到的必是 ASCII。
static char lowerAscii(char ch) { return (ch >= 'A' && ch <= 'Z') ? (char)(ch + 32) : ch; }

static bool eqNoCase(const char *a, const char *b) {
  for (; *a && *b; a++, b++) if (lowerAscii(*a) != lowerAscii(*b)) return false;
  return *a == '\0' && *b == '\0';
}

// 严格解析一个 0~maxV 的十进制数。不用 atoi：atoi("1x") 返回 1，于是 "p 1x 2" 看起来成功了，实际画到了别处——
// 这种错误在标定场景下最坑，因为你会去怀疑屏而不是怀疑命令。
static bool parseU8(const char *s, uint8_t maxV, uint8_t *out) {
  if (s == nullptr || *s == '\0') return false;
  uint16_t v = 0;
  for (const char *p = s; *p; p++) {
    if (*p < '0' || *p > '9') return false;
    v = (uint16_t)(v * 10 + (uint8_t)(*p - '0'));
    if (v > maxV) return false;      // 及早收手，顺手也挡住了长数字串的溢出
  }
  *out = (uint8_t)v;
  return true;
}

// 按空格就地切词。返回切出的【真实】词数，最多只往 out 里填 n 个——调用方拿真实词数跟预期数量比对，
// 「参数给多了」于是能被发现，而不是被静默丢掉。
// 手工切而不是 strtok：strtok 带内部静态状态，在这个每轮 loop 都会被调用的地方是个隐患。
static uint8_t splitWords(char *s, char **out, uint8_t n) {
  uint8_t cnt = 0;
  while (*s) {
    while (*s == ' ') s++;
    if (*s == '\0') break;
    if (cnt < n) out[cnt] = s;
    cnt++;
    while (*s && *s != ' ') s++;
    if (*s) *s++ = '\0';
  }
  return cnt;
}

static const char *modeName() {
  switch (dispMode) {
    case MODE_MANUAL: return "manual";
    case MODE_SCAN:   return scanByRow ? "scan-row" : "scan-col";
    case MODE_GRAVITY: return "gravity";
    default:          return "marquee";
  }
}

// 回显统一走 eventLine，所以格式、时间戳、可 grep 性都跟按键与编码器事件一致：grep '^\[' 一把捞出全部事件。
static void ackPlain(const char *what) {
  eventLine("[CMD]", what);
}

static void ack1(const char *what, uint8_t v) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%s %u", what, (unsigned)v);
  eventLine("[CMD]", buf);
}

static void ack2(const char *what, uint8_t a, uint8_t b) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%s %u %u", what, (unsigned)a, (unsigned)b);
  eventLine("[CMD]", buf);
}

// 报错：ASCII 的理由进 eventLine，中文提示另起一行。那一行故意不带标签，正是为了不破坏上面那条对齐规则。
static void ackErr(const char *why) {
  eventLine("[CMD]", why);
  Serial.println(F("  命令格式不对，敲 ? 看清单。"));
}

// 帮助文本全走 F()：TX 环形缓冲是 0，底层 FIFO 只有 128 字节，一次倾倒几百字节会把 loop() 卡住几毫秒。
// 把它压短，比给这一条命令专门开 TX 缓冲划算。
static void printHelp() {
  Serial.println();
  Serial.println(F("命令不区分大小写、以回车结束。坐标是数学系逻辑坐标 0~7：原点在左下角，x 向右、y 向上，x 在前。"));
  Serial.println(F("  p x y           点亮 (x,y)。不清屏，可以连着画"));
  Serial.println(F("  c               清屏。这是唯一会擦除内容的命令"));
  Serial.println(F("  a               全亮，查死点"));
  Serial.println(F("  h y             点亮整行：y 固定，x 走满 8 格"));
  Serial.println(F("  v x             点亮整列：x 固定，y 走满 8 格"));
  Serial.println(F("  s row|col|off   逐行扫 / 逐列扫 / 停。s 单独一条则报当前状态"));
  Serial.println(F("  m [ms]          切回跑马灯；带参数则同时改每格毫秒（1~255）"));
  Serial.println(F("  g               重力模式：把平面内的重力方向画成一颗灯，20 Hz 实时更新，每 500 ms 打一行 GRAV。离开用别的模式命令"));
  Serial.println(F("  i [0-15]        设亮度；不带参数则报当前值。0 是最暗档，不是关屏"));
  Serial.println(F("  ?               本清单"));
  Serial.printf("当前：mode=%s  rotation=%u  intensity=%u  chase=%lums\n",
                modeName(), (unsigned)MX_ROTATION, (unsigned)mxIntensity, (unsigned long)chaseMs);
  Serial.println(F("验坐标朝向：c 清屏 → p 0 0 应在左下角 → p 1 0 应正上方 → p 0 1 应正右方 → p 7 7 应在右上角。"));
}

// 分发。所有状态变更都发生在参数全部解析成功之后。
static void runCommand(char *line) {
  char *w[3];
  const uint8_t nw = splitWords(line, w, 3);
  if (nw == 0) return;                                    // 空行：连按两次回车、以及 \r\n 的第二个字符都走这里

  const char cmd = lowerAscii(w[0][0]);
  if (w[0][1] != '\0') { ackErr("err unknown"); return; }  // 命令就一个字母，粘了别的东西一律不认

  uint8_t a = 0, b = 0;

  switch (cmd) {
    case 'p':
      if (nw != 3 || !parseU8(w[1], 7, &a) || !parseU8(w[2], 7, &b)) { ackErr("err p x y"); return; }
      enterManual();
      lightPoint(a, b);                                      // a = x，b = y
      ack2("p", a, b);
      break;

    case 'c':
      if (nw != 1) { ackErr("err c"); return; }
      enterManual();          // 清屏也算进静态模式：在跑马灯下敲 c 只会被下一拍擦掉，看起来像没反应
      mx.clear();
      ackPlain("c");
      break;

    case 'a':
      if (nw != 1) { ackErr("err a"); return; }
      enterManual();
      for (uint8_t y = 0; y < 8; y++)
        for (uint8_t x = 0; x < 8; x++) lightPoint(x, y);
      ackPlain("a");
      break;

    case 'h':
    case 'v':
      if (nw != 2 || !parseU8(w[1], 7, &a)) { ackErr(cmd == 'h' ? "err h y" : "err v x"); return; }
      enterManual();
      for (uint8_t i = 0; i < 8; i++) {
        if (cmd == 'h') lightPoint(i, a);    // 整行：y = a 固定，x 走满 8 格
        else            lightPoint(a, i);    // 整列：x = a 固定，y 走满 8 格
      }
      ack1(cmd == 'h' ? "h" : "v", a);
      break;

    case 's':
      if (nw == 1) { ackPlain(modeName()); break; }          // 不带参数：只报当前状态
      if (nw != 2) { ackErr("err s arg"); return; }
      if (eqNoCase(w[1], "row"))      { startScan(true);  ackPlain("s row"); }
      else if (eqNoCase(w[1], "col")) { startScan(false); ackPlain("s col"); }
      else if (eqNoCase(w[1], "off")) { enterManual(); mx.clear(); ackPlain("s off"); }
      else ackErr("err s arg");
      break;

    case 'm':
      if (nw > 2) { ackErr("err m ms"); return; }
      if (nw == 2) {
        if (!parseU8(w[1], 255, &a) || a == 0) { ackErr("err m ms"); return; }
        chaseMs = a;
      }
      dispMode    = MODE_MARQUEE;
      lastChaseMs = millis();                                // 让第一拍等满一个周期，否则切过去会立刻跳一格，看起来像丢了一格
      {
        char buf[24];
        snprintf(buf, sizeof(buf), "m %lums", (unsigned long)chaseMs);
        eventLine("[CMD]", buf);
      }
      break;

    case 'i':
      if (nw > 2) { ackErr("err i n"); return; }
      if (nw == 2) {
        if (!parseU8(w[1], 15, &a)) { ackErr("err i 0..15"); return; }
        setIntensity(a);
      }
      ack1("i", mxIntensity);                                // 无参数时就是查询，所以两种写法都报一次当前值
      break;

    case 'g':
      if (nw != 1) { ackErr("err g"); return; }
      startGravity();
      ackPlain("g");
      // IMU 没就绪也照进模式：重试逻辑不受模式影响，它自己会好。这里只是说明为什么屏上暂时没东西。
      if (!imuReady) Serial.println(F("  重力模式已开，但 IMU 还没就绪——它每 5 秒重试一次，就绪后画面自己会出来。"));
      break;

    case '?':
      printHelp();                                           // 刻意不产生 [CMD] 行：帮助是给人看的，不是一次状态变更
      break;

    default:
      ackErr("err unknown");
      break;
  }
}

// 非阻塞收一行。整批处理也就几毫秒，扰不动跑马灯与按键的节拍。
static void pollSerial() {
  while (Serial.available()) {
    const int ch = Serial.read();
    if (ch < 0) break;

    if (ch == '\r' || ch == '\n') {
      if (lineOverflow) {
        lineOverflow = false;      // 先清标志、再复位长度：这样 \r\n 里的第二个终止符看到的是干净状态，不会重复报错
        lineLen = 0;
        ackErr("err line long");
      } else if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        runCommand(lineBuf);
        lineLen = 0;
      }
      continue;                    // 其余情况是空行，什么都不做
    }

    if (ch == 0x08 || ch == 0x7F) {           // 退格：敲错的 p 12 还救得回来
      if (lineLen > 0 && !lineOverflow) lineLen--;
      continue;
    }

    if (ch < 0x20 || ch > 0x7E) continue;     // 其余控制字符与高位字节一律丢掉，免得串口监视器的上色序列混进来
    if (lineOverflow) continue;               // 已经超长，静默丢到行尾

    if (lineLen + 1 >= SERIAL_LINE_MAX) {     // 留一个字节给结尾的 '\0'
      lineOverflow = true;                    // 只置标志就够：在这里报错的话，超长行会每收到一个字节报一次
      continue;
    }
    lineBuf[lineLen++] = (char)ch;
  }
}

// ============================ 初始化 ============================

void setup() {
  // RX 环形缓冲默认只有 256 字节（921600 下约 2.8 ms 就填满）。要是把一串命令整批粘进来，处理前几条时后面的就可能被丢掉，
  // 而且丢得静悄悄，看起来像「命令偶尔失灵」。这句必须放在 begin() 之前：begin() 之后再调只会打一句警告并返回 0，什么也不做。
  Serial.setRxBufferSize(1024);
  Serial.begin(921600);
  delay(300);   // 给主机一点时间挂上串口，免得错过开机横幅。只有这一处阻塞，只跑一次

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F(" Gewu 上电自检"));
  Serial.println(F(" 板型 AirM2M CORE ESP32C3 / 串口 921600"));
  Serial.println(F(" 引脚与排查依据：仓库根目录《硬件接线.md》"));
  Serial.println(F(" 串口命令：敲 ? 看清单（p x y 点灯 / a 全亮 / s 扫描 / g 重力 / i 亮度 / m 跑马灯；坐标原点在左下角）"));
  Serial.println(F("========================================"));

  // 点阵排在最前：上电到此刻之间 MAX7219 移位寄存器里是随机值，早初始化早清屏。
  mx.begin();
  mx.control(MD_MAX72XX::SHUTDOWN, false);
  mx.control(MD_MAX72XX::DECODE, (uint8_t)0);            // 必须关 BCD 解码，否则收到的不是点阵图案
  mx.control(MD_MAX72XX::SCANLIMIT, (uint8_t)7);         // 扫描全部 8 行
  setIntensity(MX_INTENSITY);                            // 显式设成初值：库的 begin() 会把它设成中间值（MAX_INTENSITY / 2）
  // 全文件不用调 update()：库的 begin() 里已经 control(UPDATE, ON)，而 setPoint() / clear() 内部还会各自 flushBuffer 下发
  mx.clear();
  Serial.printf("MAX7219 已初始化  DIN=%u CLK=%u CS=%u  亮度=%u/15\n",
                PIN_DIN, PIN_CLK, PIN_CS, (unsigned)mxIntensity);

  // 五路输入一律 INPUT，【不要】用 INPUT_PULLUP：
  // 硬件已各配 4.7k 上拉到 3.3V，启用 ESP32 内部 45k 上拉只会并联削弱有效上拉。易错点 15 说明，
  // 靠内部 45k 带编码器和按钮时湿电流仅 73µA，低于 ALPS 的 1mA 最小额定，
  // 症状是「用几个月才偶发丢步」，极易误判成软件问题。
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    pinMode(buttons[i].pin, INPUT);
  }
  pinMode(PIN_ENCA, INPUT);
  pinMode(PIN_ENCB, INPUT);

  // 记录编码器初相位，否则第一格会被算成一次虚假跳变
  encPrev = (int8_t)((digitalRead(PIN_ENCA) << 1) | digitalRead(PIN_ENCB));

  if (imuInit()) {
    imuReady = true;
    Serial.printf("%s OK  AD0=0x%02X  accel=%s  gyro=%s\n",
                  imuModelName(imuKind), MPU_ADDR, accelRangeStr(), gyroRangeStr());
    Serial.println(F("判定标准：点阵面朝上平放静止时 ax / ay ≈ 0、az ≈ -1.0 g（本板模组 +Z 朝地，见《标定.md》第五节），角速度是靠近 0 的小值"));
  } else {
    imuReady  = false;
    imuRetryMs = millis();
    Serial.println(F("IMU 初始化失败——对照《硬件接线.md》易错点 5 / 9 / 17 排查，5 秒后重试"));
  }

  Serial.println(F("开始：跑马灯 + 数据流。敲 ? 看串口命令清单；按键与编码器事件行以方括号标签开头（如 [SW2]）。"));
  Serial.println();
}

// ============================ 主循环 ============================

// 五个节拍各自计时，全程没有任何 delay，所以慢的活（跑马灯、数据输出）不会推迟快的事（按键事件当场出）。
// 唯一会等一会儿的是 Serial 写满底层 FIFO 的时候（TX 环是 0），所以帮助文本压得很短。
// 三个画屏的节拍必须按模式门控：它们每拍都清屏，忘了门控就会不停擦掉串口画上去的内容，症状看起来像点灯命令有 bug。
// 重力模式那一路是 20 Hz，三个里最快的，所以放在最后量时间戳也无妨——它读一次 IMU 只花几百微秒。
void loop() {
  uint32_t now = millis();

  pollSerial();        // 放在最前：命令的响应越快越顺手，而且它自己永不阻塞

  if (dispMode == MODE_MARQUEE && now - lastChaseMs >= chaseMs) {
    lastChaseMs = now;
    chaseStep();
  }

  if (dispMode == MODE_SCAN && now - lastScanMs >= SCAN_MS) {
    lastScanMs = now;
    scanStep();
  }

  if (dispMode == MODE_GRAVITY && now - lastGravityMs >= GRAVITY_MS) {
    lastGravityMs = now;
    gravityStep();
  }

  scanButtons(now);    // 这两路不按模式门控：任何模式下输入事件都得继续流
  scanEncoder();

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    imuTick();
    // 重力模式下再加一行屏幕系读数。放在这里而不是 gravityStep() 里，是因为画面要 20 Hz、
    // 串口只该 2 Hz 出声，两者必须用各自的节拍。
    if (dispMode == MODE_GRAVITY && imuReady) gravityPrint();
  }
}
