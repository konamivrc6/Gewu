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
 *   - 点阵以最低亮度一颗一颗跑马灯（一圈约 3.8 秒）
 *   - 串口每 500 ms 一行 IMU 数据（加速度 / 角速度 / 温度）
 *   - 按开关 / 转编码器时当场插一行事件，前缀 [SW2] / [SW3] / [ENCSW] / [ENC]
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

// 点阵模块的「硬件档案」——描述模块内部行列接线走向，不同厂家的 8×8 模块不一样，是本文件唯一必须现场试的常量。
// 跑马灯方向不对就换下一个候选重编译；四个都不对，再调 MX_ROTATION。
//   MD_MAX72XX::FC16_HW      最常见的单块 8×8 模块（FC-16 板型），先试这个
//   MD_MAX72XX::PAROLA_HW    Parola 套件用的模块
//   MD_MAX72XX::GENERIC_HW   通用型
//   MD_MAX72XX::ICSTATION_HW ICStation 的模块
static const MD_MAX72XX::moduleType_t MX_HW_TYPE  = MD_MAX72XX::FC16_HW;
static const uint8_t                  MX_ROTATION  = 0;    // 0~3，每加 1 把绘制坐标顺时针转 90°（实现见 chaseStep）
static const uint8_t                  MX_DEVICES  = 1;    // 级联模块数，本板只有一块

static const uint8_t  MX_INTENSITY   = 0;      // 0 = 最暗（MAX7219 的 1/32 占空比），15 = 最亮。写 0 【不是】关屏，是最暗的「亮」
static const uint32_t CHASE_MS       = 60;     // 跑马灯每格停留时间；64 格 × 60 ms ≈ 3.8 秒一圈
static const uint32_t PRINT_MS       = 500;    // IMU 数据输出间隔。故意放慢，免得把按键事件冲得找不着
static const uint32_t DEBOUNCE_MS    = 15;     // 按键软件消抖。硬件已有 100nF（6.3 节），这里只需补一道稳定确认
static const uint8_t  ENC_STEPS_PER_DETENT = 4; // EC11 转一格产生的正交跳变数。转一格刷出 4 行事件就说明这个值不对，改它
static const uint32_t IMU_RETRY_MS   = 5000;   // 传感器没就绪时的重试间隔

static const uint8_t  MPU_ADDR       = 0x68;   // AD0 接地（5.4 节）
static const uint32_t I2C_HZ         = 400000; // 4.7k 上拉在 400kHz 下上升时间合规（5.4 节）；降速可改 100000

// IMU 认出来的型号。WHO_AM_I 是唯一可信的判据：0x68 = MPU6050，0x70 = MPU6500（本板实测）。
// 认型号不只是为了打印好看——温度换算公式两张图不一样，所以型号必须在读数据之前就知道。
enum ImuKind : uint8_t { IMU_NONE = 0, IMU_6050 = 1, IMU_6500 = 2 };

// IMU 量程。寄存器值和每单位 LSB 数是一对，改一个必须同时改另一个。
//   AFS_SEL 0x00=±2g(16384) 0x08=±4g(8192) 0x10=±8g(4096) 0x18=±16g(2048)
//   FS_SEL  0x00=±250(131)  0x08=±500(65.5) 0x10=±1000(32.8) 0x18=±2000(16.4)  —— 单位 dps
static const uint8_t IMU_ACCEL_AFS = 0x00;
static const float   IMU_ACCEL_LSB = 16384.0f;
static const uint8_t IMU_GYRO_FS   = 0x00;
static const float   IMU_GYRO_LSB  = 131.0f;

// ============================ 对象与状态 ============================

// 构造函数必须用「五个参数」的软件时序版本（HW_TYPE, dataPin, clkPin, csPin, numDevices）。
// 另一个三参数版本（HW_TYPE, csPin, numDevices）走硬件 SPI，会去用 MCU 默认 SPI 引脚，而本板的 DIN/CLK 是 GPIO2/GPIO8，对不上。
MD_MAX72XX     mx(MX_HW_TYPE, PIN_DIN, PIN_CLK, PIN_CS, MX_DEVICES);

static uint32_t lastChaseMs = 0;
static uint32_t lastPrintMs = 0;
static uint8_t  chaseIdx    = 0;       // 0~63，跑马灯当前位置

static bool     imuReady    = false;
static uint32_t imuRetryMs  = 0;
static uint8_t  imuKind     = IMU_NONE;   // 认出来的型号，决定温度用哪条换算公式

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

  i2cWriteReg8(MPU_ADDR, REG_SMPLRT_DIV, 0x04);   // 1kHz / (1+4) = 200 Hz，自检用不上 1kHz
  i2cWriteReg8(MPU_ADDR, REG_CONFIG, 0x03);       // 陀螺 DLPF 开在 44Hz 档
  if (imuKind == IMU_6500) {
    // 6500 把加速度计 DLPF 搬到了独立的 ACCEL_CONFIG2；6050 的 0x1D 是保留位，
    // 往保留位写有可能碰到出厂校准值，所以这一句必须挂在型号判断里，不能无条件执行。
    i2cWriteReg8(MPU_ADDR, REG_ACCEL_CONFIG2, 0x03);
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

static void imuPrint() {
  uint8_t b[14];
  if (!i2cReadBlock(MPU_ADDR, REG_ACCEL_OUT, b, (uint8_t)sizeof(b))) {
    Serial.println(F("IMU 读 14 字节数据块失败"));
    return;
  }

  // 七个都是 16 位大端有符号数，必须先拼成 int16_t 再除以比例——直接用 int 拼会把负数算错。
  int16_t ax = (int16_t)((b[0]  << 8) | b[1]);
  int16_t ay = (int16_t)((b[2]  << 8) | b[3]);
  int16_t az = (int16_t)((b[4]  << 8) | b[5]);
  int16_t rt = (int16_t)((b[6]  << 8) | b[7]);   // 温度原始值
  int16_t gx = (int16_t)((b[8]  << 8) | b[9]);
  int16_t gy = (int16_t)((b[10] << 8) | b[11]);
  int16_t gz = (int16_t)((b[12] << 8) | b[13]);

  // 温度换算两张图不一样，这是换型号后最容易静默出错的地方：套错公式在室温下会偏高约 15 °C，不报错也不警告。
  // 原始值一并打出来——哪天真换了芯片，一眼就能看出该用哪条公式。
  float tC = (imuKind == IMU_6500) ? (rt / 333.87f + 21.0f)
                                   : (rt / 340.0f   + 36.53f);

  Serial.printf("%s ax %+.2f ay %+.2f az %+.2f g  |  gx %+.1f gy %+.1f gz %+.1f deg/s  |  %.1f C (raw %d)\n",
                imuModelName(imuKind),
                ax / IMU_ACCEL_LSB, ay / IMU_ACCEL_LSB, az / IMU_ACCEL_LSB,
                gx / IMU_GYRO_LSB,  gy / IMU_GYRO_LSB,  gz / IMU_GYRO_LSB,
                tC, rt);
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

// 逐格跑马灯：清屏 → 点亮当前那一颗 → 索引前进。逐点画而不是整行拼字节，这样 MX_HW_TYPE 的映射才真正被验证到
// （拼行字节的话，行列反了你根本看不出来）。
// 64 是 2 的幂，所以用 & 63 取模，不写 %。
static void chaseStep() {
  mx.clear();

  // 先按 MX_ROTATION 把坐标顺时针转 90°×n。这个库（3.5.1）没有 setRotation()，方向只能自己转：
  // 对单个点来说，顺时针 90° 就是 (行, 列) → (列, 7 - 行)。
  uint8_t r = chaseIdx / 8;
  uint8_t c = chaseIdx % 8;
  for (uint8_t i = 0; i < MX_ROTATION; i++) {
    uint8_t t = r;
    r = c;
    c = (uint8_t)(7 - t);
  }
  mx.setPoint(r, c, true);
  chaseIdx = (chaseIdx + 1) & 63;
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

// ============================ 初始化 ============================

void setup() {
  Serial.begin(921600);
  delay(300);   // 给主机一点时间挂上串口，免得错过开机横幅。只有这一处阻塞，只跑一次

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F(" Gewu 上电自检"));
  Serial.println(F(" 板型 AirM2M CORE ESP32C3 / 串口 921600"));
  Serial.println(F(" 引脚与排查依据：仓库根目录《硬件接线.md》"));
  Serial.println(F("========================================"));

  // 点阵排在最前：上电到此刻之间 MAX7219 移位寄存器里是随机值，早初始化早清屏。
  mx.begin();
  mx.control(MD_MAX72XX::SHUTDOWN, false);
  mx.control(MD_MAX72XX::DECODE, (uint8_t)0);            // 必须关 BCD 解码，否则收到的不是点阵图案
  mx.control(MD_MAX72XX::SCANLIMIT, (uint8_t)7);         // 扫描全部 8 行
  mx.control(MD_MAX72XX::INTENSITY, MX_INTENSITY);       // 显式设最低亮度：库的 begin() 会把它设成中间值（MAX_INTENSITY / 2）
  // 全文件不用调 update()：库的 begin() 里已经 control(UPDATE, ON)，而 setPoint() / clear() 内部还会各自 flushBuffer 下发
  mx.clear();
  Serial.printf("MAX7219 已初始化  DIN=%u CLK=%u CS=%u  亮度=%u/15\n",
                PIN_DIN, PIN_CLK, PIN_CS, MX_INTENSITY);

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
    Serial.println(F("判定标准：平放静止时 az ≈ 1.00 g，ax / ay ≈ 0，角速度是靠近 0 的小值"));
  } else {
    imuReady  = false;
    imuRetryMs = millis();
    Serial.println(F("IMU 初始化失败——对照《硬件接线.md》易错点 5 / 9 / 17 排查，5 秒后重试"));
  }

  Serial.println(F("开始：跑马灯 + 数据流。按键与编码器事件行以方括号标签开头（如 [SW2]）。"));
  Serial.println();
}

// ============================ 主循环 ============================

// 三个节拍各自计时，全程没有任何 delay，所以慢的活（跑马灯、数据输出）不会推迟快的事（按键事件当场出）。
void loop() {
  uint32_t now = millis();

  if (now - lastChaseMs >= CHASE_MS) {
    lastChaseMs = now;
    chaseStep();
  }

  scanButtons(now);
  scanEncoder();

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    imuTick();
  }
}
