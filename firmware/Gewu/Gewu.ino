/*
 * Gewu 正式固件 v0.2 —— 格子锚定物理内核 + 水 / 粒子双模式
 * ========================================================
 *
 * 这一版接编码器与 MPU6500 两路输入：顺时针一格在随机空白格添加一个粒子，逆时针一格删除最新添加的粒子（上限 64）；加速度计给实时重力，陀螺给三轴角速度，粒子会因倾斜滑落、因甩板获得冲量、因转板受到离心/科氏/欧拉力。IMU 没就绪时退回“平放、无平面内重力、不转动”，屏幕保持静止。
 * SW3 是演示用的慢速生成任务：按下清空全部粒子并立刻补一颗，之后每 60 秒加一颗；SW3 重按、编码器按下或转过一格都会打断它。SW2 切换水 / 粒子模式，切换保留现有粒子、只恢复该模式的默认参数，不打断这个任务，后续粒子按新模式的物理继续加。编码器按下直接清空全部粒子。
 *
 * 水模式（feat/water 分支默认）：板面与块间摩擦归零，只留静水压强，重力标度 0.0120，不再按倍数绑定粒子模式，转速增益取两倍（200%；离心按 ω² 长，实际是四倍）。一格一颗保证不可压缩，竖直方向上的重量沿重力反方向数出水头差，变成横向推力，所以竖柱会塌、液面会平。内聚力与键轴黏性取很小的值（a 4 / j 4）：4 mm 的格子上表面张力的毛细长度 2.7 mm、黏性边界层 0.2 mm，两者都不足一个像素，宏观形状由静水压强决定；但低速流淌时没有它们，水会散成单个格子。m particles 切回粒子模式，a / j 可现场把内聚与黏性加回来，h 调静水压强，g 调当前模式的重力标度。
 *
 * 物理内核在 ../common/PhysicsWorld.h，逐位移植自 sim/world2.py；定点原语在 ../common/fixed_q16.h。这个草图只做三件事：读传感器、把输入翻译成增删与转动、把格号画到点阵上。
 *
 * 坐标有三套，各自的翻转只允许出现一次：
 *   - 引擎：x 右、y 下，原点左上，cy=0 在顶部；+z 入屏，即屏幕上的顺时针
 *   - 固件逻辑：x 右、y 上，原点左下，与自检草图一致
 *   - 屏幕系：x 右、y 上、n 出屏；重力与角速度都先落在这里
 * 引擎→逻辑的 y 翻转只在 renderWorld()；屏幕系→引擎的翻转只在 PhysicsWorld::setGravityScreenQ16()（重力翻 y）与 setSpinScreenQ16()（角速度翻 y、z）。串口命令与事件行一律用固件逻辑坐标。
 *
 * 硬件层（引脚、MAX7219 初始化、IMU 裸 I2C、编码器查表、串口行缓冲）从 Hardware_Test.ino 复制。暂时不抽到 common/：第三次出现同类草图时再抽，避免现在动坏已经验过的自检草图。
 *
 * 依赖：MD_MAX72XX by majicDesigns（版本钉在 sketch.yaml）。
 * 板型：AirM2M CORE ESP32C3，Flash Mode = DIO，USB CDC On Boot = Disabled，串口 921600。
 */

#include <Wire.h>
#include <MD_MAX72xx.h>
#include <math.h>
#include <string.h>
#include <esp_system.h>

#include "../common/BoardProfile.h"
#include "../common/fixed_q16.h"
#include "../common/PhysicsWorld.h"

// ============================ 引脚 ============================
// 以《硬件接线.md》4.3 节为准，与自检草图完全一致。改这里时按 GPIO 号改，不要按原理图脚号改。

static const uint8_t PIN_DIN   = 2;
static const uint8_t PIN_CLK   = 8;
static const uint8_t PIN_CS    = 10;
static const uint8_t PIN_SDA   = 4;
static const uint8_t PIN_SCL   = 5;
static const uint8_t PIN_SW2   = 6;
static const uint8_t PIN_SW3   = 7;
static const uint8_t PIN_ENCSW = 3;
static const uint8_t PIN_ENCA  = 1;
static const uint8_t PIN_ENCB  = 0;

// ============================ 可调参数 ============================

static const uint8_t  MX_INTENSITY    = 0;        // 上电初值，与自检草图一致；0 是最暗档不是关屏，串口 i 可改
static const uint32_t IMU_MS          = 5;        // 重力与角速度的更新间隔，200 Hz，与传感器 200 Hz 输出对齐；原来是 50 ms
static const uint32_t IMU_RETRY_MS    = 5000;     // IMU 没就绪时的重试间隔
static const uint8_t  IMU_FAIL_LIMIT  = 200;      // 连续读失败约 1 秒才回退；20 Hz 时代是 20 次
// 陀螺：每轴死区压掉静止噪声与残余零偏；角加速度用相邻样本差分，再过一阶低通。
static const float    GYRO_DEADBAND_DPS = 0.2f;   // 标定残差 ≤0.03 dps、静止噪声约 0.056 dps，0.2 dps 约 3.6σ
static const float    GYRO_WD_TAU_S     = 0.025f; // 25 ms 低通，把差分的 0.28 rad/s² 噪声压到约 0.08
static const float    GYRO_WD_DEADBAND  = 0.5f;   // rad/s²，低通后仍低于此值就归零
static const uint8_t  SPIN_FAIL_LIMIT   = 3;      // 连续读失败约 15 ms 就停转，不保留旧转速
static const uint32_t SUBSTEP_US      = 4167;     // 240 Hz 子步，1e6 / 240 的整数近似，误差 0.008%
static const uint8_t  MAX_CATCHUP     = 8;        // 一次 loop 最多补 8 个子步，约 33 ms
// 微动消抖。硬件已各配 4.7k 上拉与 100nF（硬件接线.md 5.5 / 6.3），软件只补一道 15 ms 稳定确认。
static const uint32_t DEBOUNCE_MS     = 15;
// SW3 慢速生成任务的默认周期。串口 t N 可临时改，重启回到这里。
static const uint32_t DRIP_INTERVAL_MS = 60000;
// 1 g 对应引擎的格/子步²。这是标度旋钮，不是标定常数；串口 g N 按 N/10000 现场改，例如 g 118 = 0.0118。
// 粒子模式按 1 格 = 4 mm、每帧 4 子步折算，实时重力是 0.0426；0.0118 对应约 1.9 倍慢放，再高就压不住堆叠——竖直柱的冲量每子步最多传一格，
// 顶层残余 |vy| 与 g 成正比，0.012 附近约 0.28 格/子步，接近 vmax 的一半，再提就会抖散。这条是内核上限，不是审美取舍。
static const double   GRAVITY_PER_G_INIT = 0.0118;
// 水模式不再按倍数绑定粒子。粒子提到 0.0118 后两者已经接近，水的「更重」由静水压强表现，重力标度保持原绝对值，手感不被粒子调参牵动。
static const double   GRAVITY_PER_G_WATER = 0.0120;
static double         gravityPerG        = GRAVITY_PER_G_WATER;
// 转速增益，百分数。转动伪力随它线性缩放（离心是平方），串口 w N 现场改，0 等于停转。
// 粒子模式保持 100%：IMU 给的是真实角速度，离心 ω²r 本来就应该和重力同尺度；重力标度提到实时附近之后，两者的比例才是物理的，人为放大反而假。
// 水模式仍取 200%，那是艺术夸张——水的黏性与低摩擦让它需要更明显的甩动才看得出来。
static const uint16_t SPIN_SCALE_INIT  = 100;
static const uint16_t SPIN_SCALE_WATER = 200;
static uint16_t       spinScale        = SPIN_SCALE_WATER;
static const double   RAD_PER_DEG     = 0.017453292519943295769;   // 不要叫 DEG_TO_RAD，Arduino.h 里已有同名宏
// 手感初值。PhysicsWorld::Params 的默认仍是 0.35 / 0.55，那是与 Python 原型逐位对照用的；固件在这里单独覆盖，别把两边混成一个默认值。
// 这组值按「颗粒材料」反推，不是按某一种实物标定：方块不滚、互锁强，真实材料的系数只能当起点。
// 板面 μ_p 取 0.05，启滑倾角约 2.9°。真实圆粒在光滑板上会滚（滚动阻力系数 0.01~0.03），方块不会，所以要用很低的库仑系数去等效那个缺失的自由度。
// 块-块 μ 取 0.45，按 tan 换回约 24° 的休止角；方块互锁比圆粒强，同样的 μ 会堆得更陡，0.45 是往回找的补偿。
static const double   PLATE_FRICTION_INIT = 0.05; // 板面 μ_p
static const double   PAIR_FRICTION_INIT  = 0.45; // 块-块与块-墙 μ
// 水模式初值：零摩擦、零恢复、静水压强，再加一对很小的内聚力与键轴黏性。
// 宏观形状由静水压强决定：4 mm 格子上表面张力的毛细长度 2.7 mm、黏性边界层 0.2 mm，都不足一个像素，
// 32 mm 容器的邦德数是 138，所以关掉内聚力时倾斜、甩板、竖柱塌落的结果逐项不变。
// 但低速流淌时会散成单个格子——速度差让相邻块逐渐分离，而内聚力只在相邻时作用，分开就回不来。
// a 4 / j 4 是板上调出来的：a 3 还散，a 5 已经黏得发滞；j 负责把相邻块的速度差抹平，静止液面不受影响。
// a / j 仍可现场改，重启回到这组。
static const double   WATER_COHESION_INIT = 0.0004;
static const double   WATER_COHESION_DIAG = 0.5;
static const double   WATER_COHESION_DAMP = 0.04;
// v water 的逐位基准独立于上面的手感参数：Python 的 sim/water/port_trace.py 写的就是这组，改常量不能动它。
static const double   WATER_TRACE_COHESION = 0.0010;
static const double   WATER_TRACE_COHESION_DIAG = 0.5;
static const double   WATER_TRACE_COHESION_DAMP = 0.05;
static const double   WATER_HYDROSTATIC_INIT = 1.0;   // 静水压强增益。1.0 = p = ρg·h 的水平梯度；0 关掉后竖直水柱会原地立着（旧版行为）
static const double   WATER_RESTITUTION   = 0.0;
// 0.50 是观感甜点：低于 0.4 碰撞闷，像掉进泥里；高于 0.6 一堆粒子同时弹就变成噪点。0.5 每跳留下 25% 的动能，视觉上是「弹一下、明显弹、收住」。
static const double   PARTICLE_RESTITUTION = 0.50;
// 速度阻尼按每秒损失折算。粒子 0.9998 在 240 Hz 下每秒剩约 95.3%，接近真实空气阻力（本可忽略），让碰撞与摩擦去耗散；
// 水模式保持 0.9995：内聚力归零后它成了唯一的耗散，负责液面晃荡的衰减。真实水在 32 mm 容器里的晃荡衰减时间约 8 秒，慢放 1.9 倍后大致对得上。
static const double   PARTICLE_DAMPING = 0.9998;
static const double   WATER_DAMPING    = 0.9995;
static const uint32_t I2C_HZ          = 400000;
static const uint32_t K_MAX           = 60000;    // k 命令一次最多推进多少帧，防止串口挂死
static const uint32_t TRACE_MAX_FRAME = 5000;
static const uint32_t SERIAL_LINE_MAX = 32;

// 编码器方向。顺时针对应查表 +1 还是 -1 取决于 A/B 哪一相接在 GPIO1，只能上板实转确认；反了就把 ENCODER_SIGN 取负，不要改 ENC_TABLE。
// 本板实测：查表 +1 对应逆时针，所以这里取 −1，顺时针才是添加。
static const int8_t ENCODER_SIGN = -1;
static const bool   CW_ADDS      = true;

enum EncFunction : uint8_t {
  ENC_FN_PARTICLES = 0,   // 本版唯一实现：顺加逆删
  ENC_FN_GRAVITY,
  ENC_FN_RESTITUTION,
  ENC_FN_DAMPING,
  ENC_FN_TIMESTEP,
  ENC_FN_COUNT
};
static uint8_t encFunction = ENC_FN_PARTICLES;

enum SimMode : uint8_t { MODE_WATER = 0, MODE_PARTICLES = 1 };
static uint8_t simMode = MODE_WATER;

// ============================ 对象与状态 ============================

// 构造函数必须用「五个参数」的软件时序版本，本板 DIN/CLK 是 GPIO2/GPIO8，不能走硬件 SPI 的默认引脚。
MD_MAX72XX mx(MX_HW_TYPE, PIN_DIN, PIN_CLK, PIN_CS, MX_DEVICES);

static gw::PhysicsWorld world;
static gw::PhysicsWorld traceWorld;

static uint8_t  mxIntensity = MX_INTENSITY;
static uint64_t lastMask    = 0;
static bool     haveMask    = false;

static uint32_t nextSubUs        = 0;
static uint8_t  subInFrame       = 0;
static uint32_t droppedSubsteps  = 0;
static uint32_t loopMaxUs        = 0;   // 含串口命令的回显耗时；要干净值先用 v/k 清零，静置几秒再读 s

static uint32_t lastImuMs   = 0;
static uint32_t imuRetryMs  = 0;
static bool     imuReady    = false;
static uint8_t  imuKind     = 0;      // ImuKind：0 未知 / 1 MPU6050 / 2 MPU6500
static uint8_t  imuFailCount = 0;
static float    lastGx = 0.0f, lastGy = 0.0f, lastGz = 0.0f;   // 屏幕系，单位 g
// IMU 链路的性能计数。只包住一次读取 + 标定换算，不含随后的 N 重算，用来回答“200 Hz 到底吃多少时间”。
static uint32_t imuReadUs     = 0;      // 最近一次读取耗时
static uint32_t imuReadMaxUs  = 0;      // 上电以来最大
static uint32_t imuSamples    = 0;      // 成功采样总数
static uint32_t imuWinStartMs = 0;      // 速率统计窗口起点
static uint32_t imuWinCount   = 0;
static float    imuRateHz     = 0.0f;   // 实际采样率

// 陀螺链路。单位统一用屏幕系 dps 与 dps/s，最后一次换算才进定点。
static float    lastWx = 0.0f, lastWy = 0.0f, lastWz = 0.0f;       // 已过死区与增益，喂给物理的值
static float    lastWdx = 0.0f, lastWdy = 0.0f, lastWdz = 0.0f;    // 已过低通、死区与增益
static float    prevWx = 0.0f, prevWy = 0.0f, prevWz = 0.0f;       // 差分的上一拍，未过死区与增益
static float    filtWdx = 0.0f, filtWdy = 0.0f, filtWdz = 0.0f;    // 低通状态，dps/s
static uint32_t lastImuUs     = 0;      // 上一拍成功采样的时刻，供差分的实测 dt
static bool     havePrevW     = false;  // 第一个样本、长间隔跳变与读失败后不差分
static uint8_t  spinFailCount = 0;      // 连续读失败次数
static uint32_t spinUpdates   = 0;      // 成功写入转动状态的次数
static uint32_t spinRejects   = 0;      // setSpinScreenQ16 拒绝的次数

// ImuRaw 必须出现在任何用到它的函数之前。Arduino 的自动原型会被插到文件靠前的位置，原型里的 ImuRaw 那时还不认识，报的是「was not declared in this scope」加一句莫名其妙的「redeclared as different kind of entity」。
struct ImuRaw {
  int16_t acc[3];
  int16_t gyro[3];
};

enum ImuKind : uint8_t { IMU_NONE = 0, IMU_6050 = 1, IMU_6500 = 2 };

// 放在 ImuRaw 之后：Arduino 的自动原型会插在第一个函数之前，函数放得太靠前会把原型顶到 struct ImuRaw 之前。
static const char *simModeName() { return simMode == MODE_WATER ? "water" : "particles"; }

// 串口行缓冲。和自检草图同一套：超长整行丢掉，只报一次错。
static char    lineBuf[SERIAL_LINE_MAX];
static uint8_t lineLen      = 0;
static bool    lineOverflow = false;

// 编码器正交解码状态表：索引 = 上次状态 × 4 + 本次状态，状态值 = (A << 1) | B。只接受相邻合法跳变，非法中间态返回 0。
static const int8_t ENC_TABLE[16] = {
  //  本次:  00   01   10   11
             0,  -1,  +1,   0,   // 上次 00
            +1,   0,   0,  -1,   // 上次 01
            -1,   0,   0,  +1,   // 上次 10
             0,  +1,  -1,   0,   // 上次 11
};
static int8_t  encPrev  = 0;
static int8_t  encAccum = 0;
static int32_t encDetent = 0;

// 三路微动的消抖状态。低电平有效，按下沿才动作；松开沿不报，免得串口刷屏。
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

// SW3 慢速生成任务。按下清空全部粒子并立刻补一颗，之后每 dripIntervalMs 加一颗；SW3 重按、编码器按下或转过一格都会终止它，SW2 只切模式不打断（后续粒子按新模式加）。串口命令不打断，方便台面调参时让它继续跑。
static uint32_t dripIntervalMs = DRIP_INTERVAL_MS;
static bool     dripActive  = false;
static uint32_t dripNextMs  = 0;
static uint16_t dripAdded   = 0;

// ============================ 小工具 ============================

static void eventLine(const char *tag, const char *detail) {
  Serial.printf("%-8s%-8s@%lumillis\n", tag, detail, (unsigned long)millis());
}

static char lowerAscii(char ch) { return (ch >= 'A' && ch <= 'Z') ? (char)(ch + 32) : ch; }

static bool eqNoCase(const char *a, const char *b) {
  for (; *a && *b; a++, b++) if (lowerAscii(*a) != lowerAscii(*b)) return false;
  return *a == '\0' && *b == '\0';
}

static bool parseU8(const char *s, uint8_t maxV, uint8_t *out) {
  if (s == nullptr || *s == '\0') return false;
  uint16_t v = 0;
  for (const char *p = s; *p; p++) {
    if (*p < '0' || *p > '9') return false;
    v = (uint16_t)(v * 10 + (uint8_t)(*p - '0'));
    if (v > maxV) return false;
  }
  *out = (uint8_t)v;
  return true;
}

static bool parseU32(const char *s, uint32_t maxV, uint32_t *out) {
  if (s == nullptr || *s == '\0') return false;
  uint32_t v = 0;
  for (const char *p = s; *p; p++) {
    if (*p < '0' || *p > '9') return false;
    v = v * 10 + (uint32_t)(*p - '0');
    if (v > maxV) return false;
  }
  *out = v;
  return true;
}

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

// ============================ I2C 原语 ============================

static const uint8_t REG_SMPLRT_DIV    = 0x19;
static const uint8_t REG_CONFIG        = 0x1A;
static const uint8_t REG_GYRO_CONFIG   = 0x1B;
static const uint8_t REG_ACCEL_CONFIG  = 0x1C;
static const uint8_t REG_ACCEL_CONFIG2 = 0x1D;
static const uint8_t REG_ACCEL_OUT     = 0x3B;
static const uint8_t REG_GYRO_OUT      = 0x43;
static const uint8_t REG_PWR_MGMT_1    = 0x6B;
static const uint8_t REG_PWR_MGMT_2    = 0x6C;
static const uint8_t REG_WHO_AM_I      = 0x75;

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

static bool i2cReadBlock(uint8_t devAddr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)devAddr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

// ============================ IMU ============================

static const char *whoamiHint(uint8_t who) {
  switch (who) {
    case 0x68: return "  (MPU6050/MPU6000，可用)";
    case 0x70: return "  (MPU6500，本板实测型号，可用)";
    case 0x71: return "  (MPU9250 的标识码；模块上若没有磁力计，多半是 6500 被写成了 9250 的 ID)";
    case 0x73: return "  (MPU9255 的标识码)";
    case 0x19: return "  (ICM/MPU6886 系列，本固件不支持)";
    case 0xFF: return "  (读不到——I2C 无应答：查 M3 的 VCC 是否 3.3V、AD0 是否接地、SDA/SCL 有没有误走 U2 缓冲器)";
    default:   return "  (未知标识码，本固件只认 0x68 和 0x70)";
  }
}

static const char *imuModelName(uint8_t kind) {
  switch (kind) {
    case IMU_6050: return "MPU6050";
    case IMU_6500: return "MPU6500";
    default:       return "未知 IMU";
  }
}

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

static bool imuInit() {
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);

  uint8_t who = i2cReadReg8(MPU_ADDR, REG_WHO_AM_I);
  Serial.printf("IMU WHO_AM_I = 0x%02X%s\n", who, whoamiHint(who));

  if (who == 0x68)      imuKind = IMU_6050;
  else if (who == 0x70) imuKind = IMU_6500;
  else { imuKind = IMU_NONE; return false; }

  if (!i2cWriteReg8(MPU_ADDR, REG_PWR_MGMT_1, 0x01)) return false;
  delay(50);
  i2cWriteReg8(MPU_ADDR, REG_PWR_MGMT_2, 0x00);

  i2cWriteReg8(MPU_ADDR, REG_SMPLRT_DIV, IMU_SMPLRT_DIV);
  i2cWriteReg8(MPU_ADDR, REG_CONFIG, IMU_DLPF_CFG);
  if (imuKind == IMU_6500) {
    i2cWriteReg8(MPU_ADDR, REG_ACCEL_CONFIG2, IMU_ACCEL_DLPF_CFG);
  }
  i2cWriteReg8(MPU_ADDR, REG_GYRO_CONFIG, IMU_GYRO_FS);
  i2cWriteReg8(MPU_ADDR, REG_ACCEL_CONFIG, IMU_ACCEL_AFS);

  uint8_t pm = i2cReadReg8(MPU_ADDR, REG_PWR_MGMT_1);
  if (pm == 0xFF || (pm & 0x40)) {
    Serial.printf("PWR_MGMT_1 回读 = 0x%02X，SLEEP 位没清掉\n", pm);
    return false;
  }
  return true;
}

// 两次独立的 6 字节读：加速度从 0x3B 起，陀螺从 0x43 起。温度夹在中间（0x41..0x42），
// 一次 14 字节连续读会把温度也读回来，白占 I2C 时间；两次 6 字节读的总线开销与 14 字节读接近（只多一组 START/STOP），但不碰温度。
static bool imuRead(ImuRaw *r) {
  uint8_t b[6];
  if (!i2cReadBlock(MPU_ADDR, REG_ACCEL_OUT, b, (uint8_t)sizeof(b))) return false;
  for (uint8_t i = 0; i < 3; i++) {
    r->acc[i] = (int16_t)((b[i * 2] << 8) | b[i * 2 + 1]);
  }
  if (!i2cReadBlock(MPU_ADDR, REG_GYRO_OUT, b, (uint8_t)sizeof(b))) return false;
  for (uint8_t i = 0; i < 3; i++) {
    r->gyro[i] = (int16_t)((b[i * 2] << 8) | b[i * 2 + 1]);
  }
  return true;
}

// 加速度与陀螺一起换算。加速度的标定是在 ±2 g 量程下做的，当前 ±4 g，零点与增益一起乘 IMU_ACCEL_SCALE；
// 陀螺标定与运行都在 ±250 dps，没有量程补偿。
static bool imuReadCalibrated(float *aG, float *wDps) {
  ImuRaw r;
  if (!imuRead(&r)) return false;
  for (uint8_t i = 0; i < 3; i++) {
    aG[i]  = ((float)r.acc[i]  - ACCEL_OFFSET[i] * IMU_ACCEL_SCALE)
                              / (ACCEL_GAIN[i]   * IMU_ACCEL_SCALE);
    wDps[i] = ((float)r.gyro[i] - GYRO_OFFSET[i]) / IMU_GYRO_LSB;
  }
  return true;
}

// 模组系 → 屏幕系。屏幕 x̂ = 模组 −Y、ŷ = 模组 −X、n̂ = 模组 −Z；静止时加速度计测的是支持力 r（朝上），重力是 −r，两次取负合起来就是一次交换：g_screen = (r_y, r_x, r_z)。这里没有漏负号。
static void gravityInScreen(float ax, float ay, float az, float *gx, float *gy, float *gz) {
  *gx = ay;
  *gy = ax;
  *gz = az;
}

// 模组系 → 屏幕系角速度。陀螺与加速度计同一套轴，但角速度是赝矢量、不经过“支持力取反”，所以每个指向模组负方向的屏幕轴都留下一个负号：ω_screen = (−ω_Y, −ω_X, −ω_Z)。符号预言与实测见《标定.md》第五节。
static void rateInScreen(float wx, float wy, float wz, float *rx, float *ry, float *rz) {
  *rx = -wy;
  *ry = -wx;
  *rz = -wz;
}

static void ensureImu(uint32_t nowMs) {
  if (imuReady) return;
  if (nowMs - imuRetryMs < IMU_RETRY_MS) return;
  imuRetryMs = nowMs;
  if (imuInit()) {
    imuReady = true;
    imuFailCount = 0;
    Serial.printf("%s OK  AD0=0x%02X  accel=%s  gyro=%s  (重试成功)\n",
                  imuModelName(imuKind), MPU_ADDR, accelRangeStr(), gyroRangeStr());
  } else {
    Serial.printf("[%8lu] IMU 仍未就绪，%lu 秒后重试\n",
                  (unsigned long)nowMs, (unsigned long)(IMU_RETRY_MS / 1000));
  }
}

static void applyFallbackGravity() {
  // 平放、无平面内重力：粒子的平面内受力为 0，法向压力 N = G，板面摩擦满额。不要退回“竖直向下”——IMU 一坏整屏粒子会全滑走。
  world.setGravityScreenQ16(0, 0, fx::fromFloat(gravityPerG));
  // 转动也一起停掉：读不到陀螺时保留旧转速，等于让粒子继续受一个虚构的离心力。
  world.stopSpin();
  havePrevW = false;
  filtWdx = filtWdy = filtWdz = 0.0f;
  lastWx = lastWy = lastWz = 0.0f;
  lastWdx = lastWdy = lastWdz = 0.0f;
}

static void tickImu(uint32_t nowMs) {
  if (nowMs - lastImuMs < IMU_MS) return;
  lastImuMs = nowMs;

  if (!imuReady) {
    ensureImu(nowMs);
    if (!imuReady) return;
  }

  float aG[3];
  float wDps[3];
  const uint32_t t0 = micros();
  const bool ok = imuReadCalibrated(aG, wDps);
  const uint32_t used = micros() - t0;
  imuReadUs = used;
  if (used > imuReadMaxUs) imuReadMaxUs = used;
  if (!ok) {
    if (imuFailCount < IMU_FAIL_LIMIT) imuFailCount++;
    // 差分断链：下一拍成功时不能拿一个跨过失败的斜率。
    havePrevW = false;
    filtWdx = filtWdy = filtWdz = 0.0f;
    if (++spinFailCount >= SPIN_FAIL_LIMIT) {
      world.stopSpin();   // 不保留旧转速，否则粒子会继续受一个虚构的离心力
      lastWx = lastWy = lastWz = 0.0f;
      lastWdx = lastWdy = lastWdz = 0.0f;
    }
    if (imuFailCount >= IMU_FAIL_LIMIT) applyFallbackGravity();
    return;
  }
  imuFailCount = 0;
  spinFailCount = 0;

  imuSamples++;
  imuWinCount++;
  if (imuWinStartMs == 0) imuWinStartMs = nowMs ? nowMs : 1;
  const uint32_t winSpan = nowMs - imuWinStartMs;
  if (winSpan >= 1000) {
    imuRateHz     = (float)imuWinCount * 1000.0f / (float)winSpan;
    imuWinStartMs = nowMs;
    imuWinCount   = 0;
  }

  float gx, gy, gz;
  gravityInScreen(aG[0], aG[1], aG[2], &gx, &gy, &gz);
  lastGx = gx;
  lastGy = gy;
  lastGz = gz;

  const double G = sqrt((double)gx * gx + (double)gy * gy + (double)gz * gz);
  const int32_t Gq  = fx::fromFloat(G * gravityPerG);
  const int32_t gxq = fx::fromFloat((double)gx * gravityPerG);
  const int32_t gyq = fx::fromFloat((double)gy * gravityPerG);
  world.setGravityScreenQ16(gxq, gyq, Gq);

  // ---- 角速度：模组系 → 屏幕系 ----
  float sx, sy, sz;
  rateInScreen(wDps[0], wDps[1], wDps[2], &sx, &sy, &sz);

  // ---- 角加速度：相邻样本差分，单位 dps/s ----
  // dt 用实测值并夹在 2~20 ms：首拍、读失败、长间隔跳变都不差分，避免把一次重启或总线卡顿
  // 算成一个巨大的欧拉冲量。低通时间常数见 GYRO_WD_TAU_S。
  const float dt = (float)(t0 - lastImuUs) * 1e-6f;
  const bool canDiff = havePrevW && dt >= 0.002f && dt <= 0.020f;
  if (canDiff) {
    const float alpha = dt / (GYRO_WD_TAU_S + dt);
    filtWdx += alpha * ((sx - prevWx) / dt - filtWdx);
    filtWdy += alpha * ((sy - prevWy) / dt - filtWdy);
    filtWdz += alpha * ((sz - prevWz) / dt - filtWdz);
  } else {
    filtWdx = filtWdy = filtWdz = 0.0f;
  }
  prevWx = sx;
  prevWy = sy;
  prevWz = sz;
  lastImuUs = t0;
  havePrevW = true;

  // ---- 死区、增益、定点换算 ----
  // 死区在差分之后、增益之前：它压的是传感器零偏与噪声，不是用户想要的运动。
  const float k = (float)spinScale / 100.0f;
  const float wx = (fabsf(sx) < GYRO_DEADBAND_DPS) ? 0.0f : sx * k;
  const float wy = (fabsf(sy) < GYRO_DEADBAND_DPS) ? 0.0f : sy * k;
  const float wz = (fabsf(sz) < GYRO_DEADBAND_DPS) ? 0.0f : sz * k;
  float wdx = filtWdx * (float)RAD_PER_DEG;
  float wdy = filtWdy * (float)RAD_PER_DEG;
  float wdz = filtWdz * (float)RAD_PER_DEG;
  if (fabsf(wdx) < GYRO_WD_DEADBAND) wdx = 0.0f;
  if (fabsf(wdy) < GYRO_WD_DEADBAND) wdy = 0.0f;
  if (fabsf(wdz) < GYRO_WD_DEADBAND) wdz = 0.0f;
  wdx *= k;
  wdy *= k;
  wdz *= k;

  lastWx = wx;
  lastWy = wy;
  lastWz = wz;
  lastWdx = wdx / (float)RAD_PER_DEG;
  lastWdy = wdy / (float)RAD_PER_DEG;
  lastWdz = wdz / (float)RAD_PER_DEG;

  const double hz = world.params().substepHz;
  const double wToSub  = RAD_PER_DEG / hz;   // dps → rad/子步
  const double wdToSub = 1.0 / (hz * hz);   // rad/s² → rad/子步²（wdx 已经是 rad/s²）
  const int32_t wxq  = fx::fromFloat((double)wx * wToSub);
  const int32_t wyq  = fx::fromFloat((double)wy * wToSub);
  const int32_t wzq  = fx::fromFloat((double)wz * wToSub);
  const int32_t wdxq = fx::fromFloat((double)wdx * wdToSub);
  const int32_t wdyq = fx::fromFloat((double)wdy * wdToSub);
  const int32_t wdzq = fx::fromFloat((double)wdz * wdToSub);
  if (world.setSpinScreenQ16(wxq, wyq, wzq, wdxq, wdyq, wdzq)) {
    spinUpdates++;
  } else {
    world.stopSpin();
    spinRejects++;
  }
}

// ============================ 点阵 ============================

// 逻辑坐标（数学系，原点左下）→ 硬件坐标。全文件只有这一处定义坐标约定；本模块 MX_ROTATION = 0 已实测，y 与 row 数值直接相等。
static void logicalToHw(uint8_t x, uint8_t y, uint8_t *hr, uint8_t *hc) {
  for (uint8_t i = 0; i < MX_ROTATION; i++) {
    uint8_t t = x;
    x = (uint8_t)(7 - y);
    y = t;
  }
  *hr = y;
  *hc = x;
}

static void lightPoint(uint8_t x, uint8_t y) {
  uint8_t hr, hc;
  logicalToHw(x, y, &hr, &hc);
  mx.setPoint(hr, hc, true);
}

static void setIntensity(uint8_t v) {
  mxIntensity = v;
  mx.control(MD_MAX72XX::INTENSITY, mxIntensity);
}

// 引擎格号 → 屏幕。引擎 cy = 0 在顶部，固件逻辑 y = 0 在底部，所以唯一的翻转是 y = 7 - cy；x 直接相等。mask 没变就不重画。
static void renderWorld(bool force) {
  const uint64_t m = world.mask();
  if (!force && haveMask && m == lastMask) return;
  lastMask = m;
  haveMask = true;

  if (MX_ROTATION == 0) {
    // setRow 内部会关 update、写整行、flushBufferAll() 一次；每帧 8 次 SPI。不要逐点 setPoint：它每次都会 flushBuffer。
    for (uint8_t cy = 0; cy < 8; cy++) {
      uint8_t bits = 0;
      for (uint8_t cx = 0; cx < 8; cx++) {
        if (m & ((uint64_t)1 << (cy * 8 + cx))) bits |= (uint8_t)(1u << cx);
      }
      const uint8_t y = (uint8_t)(7 - cy);   // 引擎→逻辑的唯一一处 y 翻转
      mx.setRow(y, bits);
    }
  } else {
    // 旋转非 0 时退回逐点兜底，仍然走同一套 logicalToHw()。
    mx.clear();
    for (uint8_t i = 0; i < world.count(); i++) {
      const gw::Block &b = world.block(i);
      lightPoint((uint8_t)b.cx, (uint8_t)(7 - b.cy));
    }
  }
}

// 模式切换只改参数，不重建内核，种子与转动原点保持不变，也不清空现有粒子——同一个初态可以直接看两种物理的差别。
static void applyMode(uint8_t mode) {
  simMode = mode;
  gravityPerG = (mode == MODE_WATER) ? GRAVITY_PER_G_WATER : GRAVITY_PER_G_INIT;
  spinScale = (mode == MODE_WATER) ? SPIN_SCALE_WATER : SPIN_SCALE_INIT;
  if (mode == MODE_WATER) {
    world.setPlateFrictionQ16(0);
    world.setPairFrictionQ16(0);
    world.setRestitutionQ16(fx::fromFloat(WATER_RESTITUTION));
    world.setCohesionFloat(WATER_COHESION_INIT);
    world.setCohesionDiagFloat(WATER_COHESION_DIAG);
    world.setCohesionDampingFloat(WATER_COHESION_DAMP);
    world.setHydrostaticFloat(WATER_HYDROSTATIC_INIT);
    world.setDampingQ16(fx::fromFloat(WATER_DAMPING));
  } else {
    world.setPlateFrictionQ16(fx::fromFloat(PLATE_FRICTION_INIT));
    world.setPairFrictionQ16(fx::fromFloat(PAIR_FRICTION_INIT));
    world.setRestitutionQ16(fx::fromFloat(PARTICLE_RESTITUTION));
    world.setDampingQ16(fx::fromFloat(PARTICLE_DAMPING));
    world.setCohesionFloat(0.0);
    world.setCohesionDampingFloat(0.0);
    world.setHydrostaticFloat(0.0);
  }
  // 按新模式的重力标度立刻刷新；IMU 就绪时下一个 5 ms 节拍会用真实姿态覆盖。切换模式本来就会清空粒子，停转没有副作用。
  applyFallbackGravity();
  renderWorld(true);
}

// ============================ 物理节拍 ============================

static void tickPhysics() {
  const uint32_t now = micros();
  uint8_t done = 0;
  bool frameDone = false;

  while ((int32_t)(now - nextSubUs) >= 0 && done < MAX_CATCHUP) {
    world.substep();
    nextSubUs += SUBSTEP_US;
    done++;
    if (++subInFrame >= world.params().substeps) {
      subInFrame = 0;
      frameDone = true;
    }
  }

  if ((int32_t)(now - nextSubUs) >= 0) {
    // 补帧上限用尽仍然落后：丢弃积压，防止一次卡顿之后追帧追到失控。
    nextSubUs = now + SUBSTEP_US;
    subInFrame = 0;
    droppedSubsteps++;
  }

  if (frameDone) renderWorld(false);
}

// ============================ SW3 慢速生成 ============================

static void stopDrip(const char *why) {
  if (!dripActive) return;
  dripActive = false;
  char buf[40];
  snprintf(buf, sizeof(buf), "stop %s n=%u", why, (unsigned)world.count());
  eventLine("[DRIP]", buf);
}

// 加一颗并报位置。满 64 颗（addRandom 返回 −1）就结束任务，不每分钟空转。
static bool dripAddOne() {
  const int16_t idx = world.addRandom();
  if (idx < 0) {
    stopDrip("full");
    return false;
  }
  dripAdded++;
  const gw::Block &b = world.block((uint8_t)idx);
  char buf[40];
  snprintf(buf, sizeof(buf), "add (%d,%d) n=%u", (int)b.cx, 7 - (int)b.cy, (unsigned)world.count());
  eventLine("[DRIP]", buf);
  return true;
}

// 按下 SW3：清空、立刻补一颗、重置计时。任务进行中再按等于重开，所以先停旧的任务。
static void startDrip(uint32_t nowMs) {
  if (dripActive) stopDrip("restart");
  world.clear();
  dripActive = true;
  dripNextMs = nowMs + dripIntervalMs;
  dripAdded = 0;
  char buf[48];
  snprintf(buf, sizeof(buf), "start next=%lus", (unsigned long)(dripIntervalMs / 1000));
  eventLine("[DRIP]", buf);
  dripAddOne();
  renderWorld(true);
}

static void tickDrip(uint32_t nowMs) {
  if (!dripActive) return;
  if ((int32_t)(nowMs - dripNextMs) < 0) return;
  // 下一个点从“现在”重算而不是累加：k 这类阻塞命令会让 loop 停几秒，
  // 累加的话恢复后会连补好几颗；重算保证一次最多补一颗。
  dripNextMs = nowMs + dripIntervalMs;
  if (dripAddOne()) renderWorld(true);
}

// ============================ 微动 ============================

static void scanButtons(uint32_t nowMs) {
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    ButtonState &b = buttons[i];
    const bool raw = (digitalRead(b.pin) == LOW);   // 低电平有效

    if (raw != b.lastRaw) {
      b.lastRaw = raw;
      b.lastEdgeMs = nowMs;
      continue;                                     // 刚变化，先等它稳下来
    }
    if (raw == b.stable || (nowMs - b.lastEdgeMs) < DEBOUNCE_MS) continue;

    b.stable = raw;
    if (!raw) continue;                             // 只处理按下沿

    eventLine(b.tag, "press");
    if (b.pin == PIN_SW3) {
      startDrip(nowMs);
    } else if (b.pin == PIN_SW2) {
      // SW2 只切模式，不打断 SW3 任务：applyMode 保留现有粒子，慢速生成继续按新模式加。
      const uint8_t next = (simMode == MODE_WATER) ? MODE_PARTICLES : MODE_WATER;
      applyMode(next);                              // 切换保留现有粒子，只恢复该模式的默认参数
      eventLine("[MODE]", simModeName());
    } else {
      stopDrip("key");                              // ENCSW 先打断 SW3 任务，再清空
      world.clear();
      renderWorld(true);
      eventLine("[ENCSW]", "clear");
    }
  }
}

// ============================ 编码器 ============================

static void handleDetent(int8_t rawDir) {
  stopDrip("encoder");   // 手动转了编码器，就不再让定时任务继续加粒子
  const int8_t dir = (int8_t)(rawDir * ENCODER_SIGN);
  char buf[40];

  switch (encFunction) {
    case ENC_FN_PARTICLES: {
      const bool add = CW_ADDS ? (dir > 0) : (dir < 0);
      if (add) {
        const int16_t idx = world.addRandom();
        if (idx >= 0) {
          const gw::Block &b = world.block((uint8_t)idx);
          snprintf(buf, sizeof(buf), "add (%d,%d) n=%u", (int)b.cx, 7 - (int)b.cy, (unsigned)world.count());
        } else {
          snprintf(buf, sizeof(buf), "full n=%u", (unsigned)world.count());
        }
      } else {
        if (world.count() > 0) {
          const gw::Block &nb = world.block((uint8_t)(world.count() - 1));
          const int x = nb.cx;
          const int y = 7 - nb.cy;
          world.removeNewest();
          snprintf(buf, sizeof(buf), "del (%d,%d) n=%u", x, y, (unsigned)world.count());
        } else {
          snprintf(buf, sizeof(buf), "empty");
        }
      }
      eventLine("[PHY]", buf);
      renderWorld(true);
      break;
    }
    default:
      // 其余映射只留接口，这一版不实现。
      snprintf(buf, sizeof(buf), "fn=%u not implemented", (unsigned)encFunction);
      eventLine("[ENC]", buf);
      break;
  }
}

static void scanEncoder() {
  const uint8_t curr = (uint8_t)((digitalRead(PIN_ENCA) << 1) | digitalRead(PIN_ENCB));
  const int8_t step = ENC_TABLE[(encPrev << 2) | curr];
  encPrev = (int8_t)curr;
  if (step == 0) return;

  encAccum += step;

  while (encAccum >= ENC_STEPS_PER_DETENT) {
    encAccum -= ENC_STEPS_PER_DETENT;
    encDetent++;
    handleDetent(+1);
  }
  while (encAccum <= -ENC_STEPS_PER_DETENT) {
    encAccum += ENC_STEPS_PER_DETENT;
    encDetent--;
    handleDetent(-1);
  }
}

// ============================ 串口命令 ============================

static void ackPlain(const char *what) { eventLine("[CMD]", what); }

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

static void ackErr(const char *why) {
  eventLine("[CMD]", why);
  Serial.println(F("  命令格式不对，敲 ? 看清单。"));
}

static void printHelp() {
  Serial.println();
  Serial.println(F("命令不区分大小写、以回车结束。坐标是数学系逻辑坐标 0~7：原点在左下角，x 向右、y 向上，x 在前。"));
  Serial.println(F("  n               添加一个粒子，随机落在空白格（等于顺时针转一格）"));
  Serial.println(F("  d               删除最新添加的粒子（等于逆时针转一格）"));
  Serial.println(F("  p x y           在逻辑坐标 (x,y) 添加一个粒子，用于确定性摆位"));
  Serial.println(F("  c               清空所有粒子"));
  Serial.println(F("  i [0-15]        设亮度；不带参数则报当前值。0 是最暗档，不是关屏"));
  Serial.println(F("  f [0-99]        设板面摩擦 μ_p（百分之几，例如 f 15 = 0.15）；不带参数报当前值"));
  Serial.println(F("  u [0-99]        设块-块 / 块-墙摩擦 μ（百分之几）；不带参数报当前值"));
  Serial.println(F("  g [1-999]       设重力标度（万分之一，例如 g 118 = 0.0118）；水模式默认 120，粒子模式默认 118，m 切换恢复默认"));
  Serial.println(F("  w [0-400]       设转速增益（百分数，例如 w 150；0 等于停转）；水模式默认 200，粒子模式默认 100，m 切换恢复默认"));
  Serial.println(F("  t [1-3600]      设 SW3 慢速生成周期（秒，默认 60，重启恢复）；不带参数报当前值"));
  Serial.println(F("  m [water|particles]  切换模式（保留现有粒子）；不带参数报当前模式"));
  Serial.println(F("  a [0-99]        设内聚力（万分之一，a 10 = 0.0010；水模式默认 4）；不带参数报当前值"));
  Serial.println(F("  j [0-100]       设键轴黏性（百分数；水模式默认 4）；不带参数报当前值"));
  Serial.println(F("  h [0-999]       设静水压强增益（百分数，默认 100；0 = 关闭）；不带参数报当前值"));
  Serial.println(F("  s               打印状态、统计与不变量"));
  Serial.println(F("  k N             同步推进 N 帧（台面测试用，会阻塞输入）"));
  Serial.println(F("  v plain|friction|spin|water [帧数]   确定性 trace；前三种对照 sim/particle/port_trace.py，water 对照 sim/water/port_trace.py"));
  Serial.println(F("  ?               本清单"));
  Serial.printf("当前：mode=%s  n=%u  intensity=%u  seed=%lu\n",
                simModeName(), (unsigned)world.count(), (unsigned)mxIntensity, (unsigned long)world.seed());
  Serial.println(F("水模式 = 零摩擦 + 零恢复 + 静水压强 + 小内聚力 + 小黏性；粒子模式 = 库仑摩擦堆叠。f / u 会改当前模式的摩擦，水模式默认 0，m water 可恢复。"));
  Serial.println(F("顺时针转编码器 = 添加，逆时针 = 删除，按下 = 清空全部粒子。方向反了就改 Gewu.ino 顶部的 ENCODER_SIGN。"));
  Serial.println(F("SW3 = 清空全部粒子并立刻补一颗，之后按周期加一颗；SW3 重按、编码器按下或转动会打断。SW2 = 切换水 / 粒子模式（保留现有粒子），不打断任务。"));
}

static void printStatus() {
  const uint32_t a = world.audit();
  const uint64_t m = world.mask();
  const gw::Stats &st = world.stats();
  const gw::Params &p = world.params();

  Serial.printf("[STA] mode=%s n=%u lit=%u audit=0x%08lX mask=%016llX\n",
                simModeName(), (unsigned)world.count(), (unsigned)world.litCount(),
                (unsigned long)a, (unsigned long long)m);
  Serial.printf("[STA] commits=%lu blocked=%lu wall=%lu adds=%lu removes=%lu addfail=%lu/%lu\n",
                (unsigned long)st.commits, (unsigned long)st.blocked, (unsigned long)st.wall,
                (unsigned long)st.adds, (unsigned long)st.removes,
                (unsigned long)st.addFailFull, (unsigned long)st.addFailBad);
  Serial.printf("[STA] G=%d gx=%d gy=%d gn=%d fdv=%d gscale=%.4f mup=%.2f mu=%.2f damp=%.4f vmax=%.3f coh=%.4f cohdiag=%.2f cohvisc=%.2f press=%.2f\n",
                (int)world.gravityTotalQ16(), (int)world.gravityXQ16(), (int)world.gravityYQ16(),
                (int)world.normalQ16(), (int)world.fdvQ16(), gravityPerG,
                fx::toFloat(world.plateFrictionQ16()), fx::toFloat(world.pairFrictionQ16()),
                fx::toFloat(world.dampingQ16()), fx::toFloat(world.vmaxQ16()),
                p.cohesion, p.cohesionDiag, p.cohesionDamping, p.hydrostatic);
  Serial.printf("[STA] imu=%s fail=%u gxS=%+.3f gyS=%+.3f gzS=%+.3f\n",
                imuReady ? imuModelName(imuKind) : "NONE", (unsigned)imuFailCount,
                lastGx, lastGy, lastGz);
  Serial.printf("[STA] imu_hz=%.1f imu_us=%lu imu_max_us=%lu imu_n=%lu\n",
                imuRateHz, (unsigned long)imuReadUs, (unsigned long)imuReadMaxUs,
                (unsigned long)imuSamples);
  Serial.printf("[STA] wscale=%u%% wS=%+.2f %+.2f %+.2f dps wdS=%+.1f %+.1f %+.1f dps2 spin_updates=%lu spin_fail=%u spin_rejects=%lu\n",
                (unsigned)spinScale, lastWx, lastWy, lastWz,
                lastWdx, lastWdy, lastWdz,
                (unsigned long)spinUpdates, (unsigned)spinFailCount,
                (unsigned long)spinRejects);
  const int32_t dripNextSec = dripActive ? (int32_t)(dripNextMs - millis()) / 1000 : 0;
  Serial.printf("[STA] seed=%lu detent=%ld drops=%lu loopmax_us=%lu frame_hz=%.2f substeps=%u\n",
                (unsigned long)world.seed(), (long)encDetent, (unsigned long)droppedSubsteps,
                (unsigned long)loopMaxUs, p.substepHz / (double)p.substeps,
                (unsigned)p.substeps);
  Serial.printf("[STA] drip=%s next=%lds added=%u interval=%lus\n",
                dripActive ? "on" : "off", (long)dripNextSec,
                (unsigned)dripAdded, (unsigned long)(dripIntervalMs / 1000));
}

static void traceHeader(const char *kind, uint32_t frames) {
  Serial.printf("TRACE begin %s frames=%lu G=%d gx=%d gy=%d gn=%d fdv=%d mup=%d mu=%d\n",
                kind, (unsigned long)frames,
                (int)traceWorld.gravityTotalQ16(), (int)traceWorld.gravityXQ16(),
                (int)traceWorld.gravityYQ16(), (int)traceWorld.normalQ16(),
                (int)traceWorld.fdvQ16(), (int)traceWorld.plateFrictionQ16(),
                (int)traceWorld.pairFrictionQ16());
}

static void runTrace(const char *kind, uint32_t frames) {
  const bool isSpin = eqNoCase(kind, "spin");
  const bool isFriction = eqNoCase(kind, "friction");
  const bool isPlain = eqNoCase(kind, "plain");
  const bool isWater = eqNoCase(kind, "water");
  if (!isSpin && !isFriction && !isPlain && !isWater) {
    ackErr("err v plain|friction|spin|water");
    return;
  }
  if (frames == 0 || frames > TRACE_MAX_FRAME) {
    ackErr("err v frames");
    return;
  }

  gw::Params tp;
  if (isSpin) {
    tp.gravity = 0.0;
    tp.plateFriction = 0.0;
    tp.friction = 0.2;
  }
  if (isWater) {
    tp.gravity = 0.0;
    tp.gravityTotal = 0.0;
    tp.plateFriction = 0.0;
    tp.friction = 0.0;
    tp.restitution = 0.0;
    tp.cohesion = WATER_TRACE_COHESION;
    tp.cohesionDiag = WATER_TRACE_COHESION_DIAG;
    tp.cohesionDamping = WATER_TRACE_COHESION_DAMP;
  }
  if (!traceWorld.init(tp)) {
    ackErr("err trace init");
    return;
  }

  // 居中 4×4，行优先，与 sim/particle/port_trace.py 的 center_cells() 顺序一致。
  for (int y = 2; y < 6; y++) {
    for (int x = 2; x < 6; x++) traceWorld.addAt(x, y);
  }

  traceWorld.setFrictionAuto(true);
  traceWorld.setGravityTotalQ16(isWater ? 0 : 79);

  if (isSpin) {
    traceWorld.setGravityVectorQ16(0, 0);
    traceWorld.setSpinQ16(0, 0, 3277, 0, 0, 0);   // 0.05 弧度/子步
  } else if (isFriction) {
    traceWorld.setGravityVectorQ16(0, 33);        // G = 79，g∥ = 33，N ≈ 72，fdv > 0
  } else if (isWater) {
    traceWorld.setGravityVectorQ16(0, 0);         // 零重力，只压内聚与键轴阻尼
  } else {
    traceWorld.setGravityVectorQ16(0, 79);        // 默认 fdv = 0 的场景
  }

  traceHeader(kind, frames);

  for (uint32_t f = 0; f < frames; f++) {
    traceWorld.frame();
    const uint64_t h = traceWorld.stateHash();
    const uint64_t m = traceWorld.mask();
    const uint32_t a = traceWorld.audit();
    const gw::Stats &st = traceWorld.stats();
    Serial.printf("TRACE f=%lu h=%016llX m=%016llX n=%u k=%lld a=%u c=%lu b=%lu w=%lu\n",
                  (unsigned long)f, (unsigned long long)h, (unsigned long long)m,
                  (unsigned)traceWorld.count(), (long long)traceWorld.kineticRaw(),
                  (unsigned)(a ? 1u : 0u),
                  (unsigned long)st.commits, (unsigned long)st.blocked, (unsigned long)st.wall);
  }
  Serial.println(F("TRACE END"));
  loopMaxUs = 0;   // trace 是台面命令，不该污染正常运行的循环耗时统计
}

static void runCommand(char *line) {
  char *w[4];
  const uint8_t nw = splitWords(line, w, 4);
  if (nw == 0) return;

  const char cmd = lowerAscii(w[0][0]);
  if (w[0][1] != '\0') { ackErr("err unknown"); return; }

  uint8_t a = 0, b = 0;

  switch (cmd) {
    case 'n': {
      if (nw != 1) { ackErr("err n"); return; }
      const int16_t idx = world.addRandom();
      if (idx < 0) { ackErr("err n full"); return; }
      const gw::Block &blk = world.block((uint8_t)idx);
      char buf[24];
      snprintf(buf, sizeof(buf), "n (%d,%d) %u", (int)blk.cx, 7 - (int)blk.cy, (unsigned)world.count());
      eventLine("[CMD]", buf);
      renderWorld(true);
      break;
    }

    case 'd': {
      if (nw != 1) { ackErr("err d"); return; }
      if (world.count() == 0) { ackErr("err d empty"); return; }
      const gw::Block &nb = world.block((uint8_t)(world.count() - 1));
      const int x = nb.cx;
      const int y = 7 - nb.cy;
      world.removeNewest();
      char buf[24];
      snprintf(buf, sizeof(buf), "d (%d,%d) %u", x, y, (unsigned)world.count());
      eventLine("[CMD]", buf);
      renderWorld(true);
      break;
    }

    case 'p': {
      if (nw != 3 || !parseU8(w[1], 7, &a) || !parseU8(w[2], 7, &b)) { ackErr("err p x y"); return; }
      if (world.addAt(a, 7 - b) < 0) { ackErr("err p occupied/full"); return; }
      ack2("p", a, b);
      renderWorld(true);
      break;
    }

    case 'c':
      if (nw != 1) { ackErr("err c"); return; }
      world.clear();
      renderWorld(true);
      ackPlain("c");
      break;

    case 'i':
      if (nw > 2) { ackErr("err i n"); return; }
      if (nw == 2) {
        if (!parseU8(w[1], 15, &a)) { ackErr("err i 0..15"); return; }
        setIntensity(a);
      }
      ack1("i", mxIntensity);
      break;

    case 'f':
      if (nw > 2) { ackErr("err f N"); return; }
      if (nw == 2) {
        if (!parseU8(w[1], 99, &a)) { ackErr("err f 0..99"); return; }
        world.setPlateFrictionQ16(fx::fromFloat((double)a / 100.0));
      }
      {
        char buf[24];
        snprintf(buf, sizeof(buf), "f mu_p=%.2f", fx::toFloat(world.plateFrictionQ16()));
        eventLine("[CMD]", buf);
      }
      break;

    case 'u':
      if (nw > 2) { ackErr("err u N"); return; }
      if (nw == 2) {
        if (!parseU8(w[1], 99, &a)) { ackErr("err u 0..99"); return; }
        world.setPairFrictionQ16(fx::fromFloat((double)a / 100.0));
      }
      {
        char buf[24];
        snprintf(buf, sizeof(buf), "u mu=%.2f", fx::toFloat(world.pairFrictionQ16()));
        eventLine("[CMD]", buf);
      }
      break;

    case 'g':
      if (nw > 2) { ackErr("err g N"); return; }
      if (nw == 2) {
        uint32_t v = 0;
        if (!parseU32(w[1], 999, &v) || v == 0) { ackErr("err g 1..999"); return; }
        gravityPerG = (double)v / 10000.0;
        if (!imuReady) applyFallbackGravity();   // IMU 没就绪时立刻按新标度更新回退重力；就绪时下一个 5 ms 节拍会自己刷新
      }
      {
        char buf[24];
        snprintf(buf, sizeof(buf), "g scale=%.4f", gravityPerG);
        eventLine("[CMD]", buf);
      }
      break;

    case 'w':
      if (nw > 2) { ackErr("err w N"); return; }
      if (nw == 2) {
        uint32_t v = 0;
        if (!parseU32(w[1], 400, &v)) { ackErr("err w 0..400"); return; }
        spinScale = (uint16_t)v;
        if (spinScale == 0) world.stopSpin();   // 立刻停，不等下一个 5 ms 节拍
      }
      {
        char buf[24];
        snprintf(buf, sizeof(buf), "w scale=%u%%", (unsigned)spinScale);
        eventLine("[CMD]", buf);
      }
      break;

    case 't':
      if (nw > 2) { ackErr("err t N"); return; }
      if (nw == 2) {
        uint32_t v = 0;
        if (!parseU32(w[1], 3600, &v) || v == 0) { ackErr("err t 1..3600"); return; }
        dripIntervalMs = v * 1000UL;
        // 任务进行中改周期，按新周期重排下一个点，免得旧的长周期还压着。
        if (dripActive) dripNextMs = millis() + dripIntervalMs;
      }
      {
        char buf[24];
        snprintf(buf, sizeof(buf), "t %lus", (unsigned long)(dripIntervalMs / 1000));
        eventLine("[CMD]", buf);
      }
      break;

    case 'm':
      if (nw > 2) { ackErr("err m water|particles"); return; }
      if (nw == 2) {
        if (eqNoCase(w[1], "water")) {
          applyMode(MODE_WATER);
        } else if (eqNoCase(w[1], "particles")) {
          applyMode(MODE_PARTICLES);
        } else {
          ackErr("err m water|particles");
          return;
        }
      }
      {
        char buf[24];
        snprintf(buf, sizeof(buf), "m %s", simModeName());
        eventLine("[CMD]", buf);
      }
      break;

    case 'a':
      if (nw > 2) { ackErr("err a N"); return; }
      if (nw == 2) {
        uint32_t v = 0;
        if (!parseU32(w[1], 99, &v)) { ackErr("err a 0..99"); return; }
        world.setCohesionFloat((double)v / 10000.0);
      }
      {
        char buf[32];
        snprintf(buf, sizeof(buf), "a %.4f", world.cohesion());
        eventLine("[CMD]", buf);
      }
      break;

    case 'j':
      if (nw > 2) { ackErr("err j N"); return; }
      if (nw == 2) {
        uint32_t v = 0;
        if (!parseU32(w[1], 100, &v)) { ackErr("err j 0..100"); return; }
        world.setCohesionDampingFloat((double)v / 100.0);
      }
      {
        char buf[32];
        snprintf(buf, sizeof(buf), "j %.2f", world.cohesionDamping());
        eventLine("[CMD]", buf);
      }
      break;

    case 'h':
      if (nw > 2) { ackErr("err h N"); return; }
      if (nw == 2) {
        uint32_t v = 0;
        if (!parseU32(w[1], 999, &v)) { ackErr("err h 0..999"); return; }
        world.setHydrostaticFloat((double)v / 100.0);
      }
      {
        char buf[32];
        snprintf(buf, sizeof(buf), "h %.2f", world.hydrostatic());
        eventLine("[CMD]", buf);
      }
      break;

    case 's':
      if (nw != 1) { ackErr("err s"); return; }
      printStatus();
      break;

    case 'k': {
      if (nw != 2) { ackErr("err k N"); return; }
      uint32_t frames = 0;
      if (!parseU32(w[1], K_MAX, &frames) || frames == 0) { ackErr("err k N"); return; }
      for (uint32_t i = 0; i < frames; i++) world.frame();
      renderWorld(true);
      loopMaxUs = 0;   // k 是同步台面命令，阻塞的这一段不该算进正常运行的单圈耗时
      char buf[24];
      snprintf(buf, sizeof(buf), "k %lu", (unsigned long)frames);
      eventLine("[CMD]", buf);
      break;
    }

    case 'v': {
      if (nw != 2 && nw != 3) { ackErr("err v plain|friction|spin [frames]"); return; }
      uint32_t frames = eqNoCase(w[1], "spin") ? 600 : 240;
      if (nw == 3 && (!parseU32(w[2], TRACE_MAX_FRAME, &frames) || frames == 0)) {
        ackErr("err v frames");
        return;
      }
      runTrace(w[1], frames);
      break;
    }

    case '?':
      printHelp();
      break;

    default:
      ackErr("err unknown");
      break;
  }
}

static void pollSerial() {
  while (Serial.available()) {
    const int ch = Serial.read();
    if (ch < 0) break;

    if (ch == '\r' || ch == '\n') {
      if (lineOverflow) {
        lineOverflow = false;
        lineLen = 0;
        ackErr("err line long");
      } else if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        runCommand(lineBuf);
        lineLen = 0;
      }
      continue;
    }

    if (ch == 0x08 || ch == 0x7F) {
      if (lineLen > 0 && !lineOverflow) lineLen--;
      continue;
    }

    if (ch < 0x20 || ch > 0x7E) continue;
    if (lineOverflow) continue;
    if (lineLen + 1 >= SERIAL_LINE_MAX) {
      lineOverflow = true;
      continue;
    }
    lineBuf[lineLen++] = (char)ch;
  }
}

// ============================ 初始化 ============================

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(921600);
  delay(300);

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F(" Gewu 正式固件 v0.2"));
  Serial.println(F(" 板型 AirM2M CORE ESP32C3 / 串口 921600"));
  Serial.println(F(" 引擎 v2 格子锚定 / Q16.16 定点 / 8x8 / 容量 64"));
  Serial.println(F(" 物理 120 子步每秒 x 2 = 60 帧每秒"));
  Serial.println(F(" 模式：水（零摩擦 + 零恢复 + 静水压强，重力 0.0120，转速增益 200% = 粒子 2×）。m particles 切回粒子模式"));
  Serial.println(F(" 顺时针转编码器 = 添加一格水，逆时针 = 删除最新，按下 = 清空全部"));
  Serial.println(F(" SW2 = 切换水 / 粒子模式（保留粒子），SW3 = 清空后每 60 秒加一格"));
  Serial.println(F(" 命令：? 看清单（m 模式 / a 内聚 / j 黏性 / h 压强 / n 加 / d 删 / p x y 定点添加 / c 清空 / g 重力 / w 转速增益 / s 状态 / v trace）"));
  Serial.println(F("========================================"));

  mx.begin();
  mx.control(MD_MAX72XX::SHUTDOWN, false);
  mx.control(MD_MAX72XX::DECODE, (uint8_t)0);
  mx.control(MD_MAX72XX::SCANLIMIT, (uint8_t)7);
  setIntensity(MX_INTENSITY);
  mx.clear();
  Serial.printf("MAX7219 已初始化  DIN=%u CLK=%u CS=%u  亮度=%u/15\n",
                PIN_DIN, PIN_CLK, PIN_CS, (unsigned)mxIntensity);

  pinMode(PIN_SW2, INPUT);
  pinMode(PIN_SW3, INPUT);
  pinMode(PIN_ENCSW, INPUT);
  pinMode(PIN_ENCA, INPUT);
  pinMode(PIN_ENCB, INPUT);
  encPrev = (int8_t)((digitalRead(PIN_ENCA) << 1) | digitalRead(PIN_ENCB));

  gw::Params p;
  // 每帧 4 子步、子步率 240 Hz，帧率仍是 60 Hz。只改这两项不动重力标度，屏幕上的运动速度翻倍（时间尺度从约 6.5 倍慢放收到约 3.3 倍），堆叠稳定性不变。
  // Params 的默认值仍是 2 子步 / 120 Hz，专供 v plain|friction|spin|water 的逐位对照；两处不要混成一个默认值。
  p.substeps = 4;
  p.substepHz = 240.0;
  p.plateFriction = 0.0;
  p.friction = 0.0;
  p.restitution = WATER_RESTITUTION;
  p.damping = WATER_DAMPING;
  p.cohesion = WATER_COHESION_INIT;
  p.cohesionDiag = WATER_COHESION_DIAG;
  p.cohesionDamping = WATER_COHESION_DAMP;
  p.hydrostatic = WATER_HYDROSTATIC_INIT;
  if (!world.init(p)) {
    Serial.println(F("物理内核初始化失败——参数越界，检查 Gewu.ino 顶部的可调参数"));
  }
  simMode = MODE_WATER;
  gravityPerG = GRAVITY_PER_G_WATER;
  spinScale = SPIN_SCALE_WATER;
  world.setSeed(esp_random());
  // 转动原点 = 加速度计的实际安装位置，不是板心。BoardProfile 里存的是引擎连续坐标（逻辑格 (4,7) 的格心）。
  world.setPivotQ16(fx::fromFloat(IMU_PIVOT_ENGINE_X), fx::fromFloat(IMU_PIVOT_ENGINE_Y));
  applyFallbackGravity();   // IMU 就绪前先按平放处理，避免上电瞬间整屏粒子滑走

  if (imuInit()) {
    imuReady = true;
    Serial.printf("%s OK  AD0=0x%02X  accel=%s  gyro=%s\n",
                  imuModelName(imuKind), MPU_ADDR, accelRangeStr(), gyroRangeStr());
  } else {
    imuReady = false;
    imuRetryMs = millis();
    Serial.println(F("IMU 初始化失败——重力先按平放处理，每 5 秒重试一次"));
  }

  nextSubUs = micros();
  renderWorld(true);

  Serial.printf("物理内核就绪：容量 %u，初始粒子 0（不预置任何水格），种子 %lu\n",
                (unsigned)gw::PhysicsWorld::MAX_BLOCKS, (unsigned long)world.seed());
  Serial.println(F("开始：转编码器添加或删除水格；倾斜时零摩擦让水流向低处，竖着拿时水柱会塌、液面会平。"));
  Serial.println(F("内聚 0.0004、黏性 0.04 只负责低速时把水粘成一片，a / j 可现场改；h 调静水压强（默认 1.00），m particles 切回粒子模式。"));
  Serial.println(F("按 SW2 切换水 / 粒子模式（保留现有粒子）；按 SW3 进入慢速生成：清空后立刻补一格，之后每 60 秒加一格。"));
  Serial.println();
}

// ============================ 主循环 ============================

void loop() {
  const uint32_t loopStart = micros();
  const uint32_t nowMs = millis();

  pollSerial();          // 永不阻塞；命令响应越快越顺手
  scanButtons(nowMs);    // 微动：SW3 启动慢速生成，SW2 切换模式（不打断），ENCSW 打断并清空
  scanEncoder();         // 每轮都扫，不按模式门控
  tickDrip(nowMs);       // SW3 任务的定时加粒子
  tickImu(nowMs);        // 200 Hz 更新重力
  tickPhysics();         // 240 Hz 子步，每 4 个子步渲染一帧

  const uint32_t dt = micros() - loopStart;
  if (dt > loopMaxUs) loopMaxUs = dt;
}
