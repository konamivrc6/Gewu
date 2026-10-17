/*
 * BoardProfile —— 这台格物板子实测出来的档案
 * ==========================================
 *
 * 【分组规则】只存「实测结论」和「标定成立的前提」，不存可以随手调的旋钮。
 * 一行东西如果改了就代表这台板子的实测结论变了，它在这里；如果改了只是「我想让它跑快一点 / 亮一点」，
 * 它留在 .ino 的「可调参数」一节。轴映射不是数而是逻辑，所以在 .ino 的 gravityInScreen() 里。
 *
 * 【每条的出处】结论、判据与原始记录都在仓库根目录《标定.md》：
 *   点阵模块的行列走向与朝向        第六节
 *   编码器每格的正交跳变数          第一节
 *   加速度计的零点与增益（六个量）  第二节
 *   陀螺零偏（三个量）              第三节
 *   IMU 轴 → 屏幕坐标的映射          第五节
 *   加速度计的安装位置（转动原点）  实物测量：逻辑格 (4,7)
 *
 * 【不要手改】这些是量出来的，不是调出来的。改了等于伪造标定，而且这种错是**静默**的：
 * 读数照样有、屏幕照样亮，只是每一个数都带一个你自己编的偏置，形态上还像模像样。
 * 要改就去重测，流程与台面工具见《标定.md》，六面标定用 tools/accel_calib.py。
 *
 * 【什么时候整份作废】换板、换点阵模块、换 IMU 模块、改任何器件的安装朝向、
 * 改量程或采样率 / DLPF（下面那组「标定成立的前提」）。
 */
#ifndef BOARD_PROFILE_H
#define BOARD_PROFILE_H

#include <MD_MAX72xx.h>   // 只为 moduleType_t：点阵的「硬件档案」是库里的枚举，不是裸数字

// ============================ 点阵模块（实测） ============================

// 【已实机确认】本板就是 FC16_HW，不用改：串口点出四角和 h 3 / v 3 两条线，
// 四角与两条线的朝向全部与预期一致，即逻辑行确实是横线、逻辑列确实是竖线。依据见《标定.md》六节。
// 若换模块后跑马灯方向不对，按下面四个候选逐个试；四个都不对再动 MX_ROTATION。
//   MD_MAX72XX::FC16_HW      最常见的单块 8×8 模块（FC-16 板型）
//   MD_MAX72XX::PAROLA_HW    Parola 套件用的模块
//   MD_MAX72XX::GENERIC_HW   通用型
//   MD_MAX72XX::ICSTATION_HW ICStation 的模块
static const MD_MAX72XX::moduleType_t MX_HW_TYPE  = MD_MAX72XX::FC16_HW;

// 【已实机确认】0 就是对的：本模块装好后逻辑 y 恰好向上增长，于是 (x, y) 直接就是数学坐标系，不需要旋转。
// 0~3，每加 1 把逻辑坐标顺时针转 90°，实现见 .ino 里的 logicalToHw()。
static const uint8_t                  MX_ROTATION  = 0;

static const uint8_t                  MX_DEVICES  = 1;    // 级联模块数，本板只有一块

// ============================ 编码器（实测） ============================

// EC11 转一格产生的正交跳变数。实测 5 格 → 5 行事件、每行 detent 只动 1，与 4 对得上。
// 转一格刷出 4 行事件就说明这个值不对，改它——但那是重测，不是调参。
static const uint8_t  ENC_STEPS_PER_DETENT = 4;

// ============================ IMU 接线与配置 ============================

static const uint8_t  MPU_ADDR       = 0x68;   // AD0 接地（硬件接线.md 5.4 节）

// 加速度计的安装位置，也就是刚体转动的原点。物理内核要的是引擎连续坐标（x 右、y 下、格子占 [c, c+1]）：
// 逻辑列 4 的格心是 x = 4.5；逻辑顶行 7 对应引擎 cy = 0，格心是 y = 0.5。写成引擎坐标是为了让 .ino 里不再出现一次行翻转。
// 这是实物位置，不是调参旋钮——模块挪了就要重测。
static const float IMU_PIVOT_ENGINE_X = 4.5f;
static const float IMU_PIVOT_ENGINE_Y = 0.5f;

// 【标定成立的前提】下面这五个是**选择的**，不是测出来的——但标定结果只在这一组值下成立，
// 所以和标定产物放在一起。改量程时零点与增益按比例换算（见 IMU_ACCEL_SCALE），改采样率或 DLPF 会改噪声带宽。
//   SMPLRT_DIV 0x04 → 1 kHz / (1+4) = 200 Hz 采样
//   CONFIG     0x03 → 陀螺 DLPF 44 Hz 档
//   ACCEL_CONFIG2 0x01 → 加速度计 DLPF 218 Hz 档（只有 MPU6500 有这一颗寄存器）
//     原来是 0x03（44 Hz），会把手甩那种十几毫秒的脉冲削掉大半。DLPF 只改带宽，不改静态增益与零点，所以这一项不影响标定。
static const uint8_t  IMU_SMPLRT_DIV     = 0x04;
static const uint8_t  IMU_DLPF_CFG       = 0x03;
static const uint8_t  IMU_ACCEL_DLPF_CFG = 0x01;

// IMU 量程。寄存器值和每单位 LSB 数是一对，改一个必须同时改另一个。
//   AFS_SEL 0x00=±2g(16384) 0x08=±4g(8192) 0x10=±8g(4096) 0x18=±16g(2048)
//   FS_SEL  0x00=±250(131)  0x08=±500(65.5) 0x10=±1000(32.8) 0x18=±2000(16.4)  —— 单位 dps
// ±4 g 是为了让猛甩不削顶；代价是每 LSB 对应的 g 值翻倍，静态分辨率减半。
static const uint8_t IMU_ACCEL_AFS = 0x08;
static const float   IMU_ACCEL_LSB = 8192.0f;
static const uint8_t IMU_GYRO_FS   = 0x00;
static const float   IMU_GYRO_LSB  = 131.0f;

// ============================ 标定产物（实测） ============================

// 九个修正量，来历与判据见《标定.md》第二、三节：
//   a_g[i]   = (raw[i] - ACCEL_OFFSET[i]·k) / (ACCEL_GAIN[i]·k)     k = IMU_ACCEL_SCALE
//   w_dps[i] = (raw[i] - GYRO_OFFSET[i])  / IMU_GYRO_LSB
//
// 标定本身是在 ±2 g 量程下做的；换量程后同一物理加速度对应的 raw 计数按量程比例缩放，
// 所以读取时把零点与增益一起乘以 k，标定原值保持不动。
//
// 为什么非修正不可：未修正时静止模长随姿态在 0.990 ~ 1.037 g 之间变，也就是「往哪边偏」既取决于姿态、
// 又取决于误差，单点读数根本分不开这两者。修正后六个标定姿态与一个样本外姿态都落在 1.000 g 的 0.1% 以内。
//
// 误差几乎全压在 Z 轴：X 的增益一分不差、只带 +10 mg 零点；Y 增益高 0.13%；
// Z 增益高 1.8% 又带 -18.6 mg 零点。陀螺零偏在热态稳定时可复现到 0.03 dps（见第三节）。
static const float IMU_ACCEL_CAL_LSB = 16384.0f;
static const float IMU_ACCEL_SCALE   = IMU_ACCEL_LSB / IMU_ACCEL_CAL_LSB;   // ±4 g 时 0.5
static const float ACCEL_GAIN[3]   = {16384.0f, 16404.5f, 16682.1f};
static const float ACCEL_OFFSET[3] = {163.8f, -20.5f, -304.9f};
static const float GYRO_OFFSET[3]  = {-172.0f, 480.0f, -169.0f};

#endif  // BOARD_PROFILE_H
