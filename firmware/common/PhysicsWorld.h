/*
 * PhysicsWorld.h —— 格子锚定物理内核的 C++ 移植
 * ==============================================
 *
 * 这是 sim/world2.py 的逐位移植。每一段运算顺序、每一次取整都与 Python 原型对齐；改这里之前先看 sim/fixed.py 与设计文档 §4.2 的定点偏置说明。头文件即实现（类内定义天然 inline），因为 arduino-cli 不编译 common/ 下的 .cpp，而草图与将来的主机 harness 都要能直接 include。
 *
 * 坐标：x 向右、y **向下**，原点左上角，单位是格；格号 (cx, cy) 是整数，位置 (px, py) 是格内的连续坐标。固件那边的逻辑坐标 y 向上，两处翻转各只出现一次：屏幕→引擎的 y 翻转在 setGravityScreenQ16() 里，引擎→逻辑的 y 翻转在 Gewu.ino 的 renderWorld() 里。
 *
 * 动态块数的铁律：blocks_ 只追加，删除只弹尾部，所以任何幸存块的下标都不变，owner_ 永远不需要重建或压缩，最新添加的块恒为 blocks_[count_ - 1]。不要改成 swap-remove。
 */
#ifndef GEWU_PHYSICS_WORLD_H
#define GEWU_PHYSICS_WORLD_H

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "fixed_q16.h"

namespace gw {

enum GateOrder : uint8_t { GATE_PEN = 0, GATE_INDEX = 1 };
enum AxisOrder : uint8_t { AXIS_VEL = 0, AXIS_XY = 1, AXIS_YX = 2, AXIS_ALT = 3 };

struct Params {
  uint8_t worldW;
  uint8_t worldH;
  double gravity;          // 平面内重力大小，只有 setTiltDeg 路径用它
  double gravityTotal;     // 总重力 G。板面法向压力 N = sqrt(G² − g∥²)
  double plateFriction;    // 板面库仑系数 μ_p
  double restitution;
  double restitutionSlop;
  double friction;         // 块-块与块-墙的库仑系数 μ
  double damping;
  double vmax;             // 格/子步，必须 < 1
  double cohesion;         // 八邻域内聚加速度，格/子步²；0 = 关闭。水模式的核心项
  double cohesionDiag;     // 对角键相对轴向的权重，0..1
  double cohesionDamping;  // 沿键轴的相对速度阻尼，0..1。等效黏性
  double hydrostatic;      // 静水压强增益，0 = 关闭。横向加速度 = 增益 · g∥ · 两侧水头差。补上内聚力缺的「重量变横向推力」那一环
  double pivotX;           // 转动原点 = 加速度计位置，格坐标
  double pivotY;
  uint8_t substeps;
  double substepHz;
  GateOrder gateOrder;
  AxisOrder axisOrder;

  Params()
      : worldW(8), worldH(8), gravity(0.0012), gravityTotal(0.0012),
        plateFriction(0.35), restitution(0.20), restitutionSlop(0.004),
        friction(0.55), damping(0.9995), vmax(0.6),
        cohesion(0.0), cohesionDiag(0.5), cohesionDamping(0.0), hydrostatic(0.0),
        pivotX(4.0), pivotY(4.0), substeps(2), substepHz(120.0),
        gateOrder(GATE_PEN), axisOrder(AXIS_VEL) {}
};

struct Block {
  int32_t px, py, vx, vy;   // Q16.16
  int16_t cx, cy;
};

// 八邻域扫描顺序与 sim/world2.py 的 _COH_OFFSETS 逐项一致：dy 外层、dx 内层，跳过 (0,0)。
// 方向只取格偏移，不取粒子的连续位置：八邻域全满时向量和为零，净力只出现在边界，体内没有剪切刚度。
struct CohesionOffset {
  int8_t dx, dy;
  int32_t ux, uy;
  bool diag;
};
static const CohesionOffset COHESION_OFFSETS[8] = {
  {-1, -1, -46341, -46341, true},
  { 0, -1,      0, -65536, false},
  { 1, -1,  46341, -46341, true},
  {-1,  0, -65536,      0, false},
  { 1,  0,  65536,      0, false},
  {-1,  1, -46341,  46341, true},
  { 0,  1,      0,  65536, false},
  { 1,  1,  46341,  46341, true},
};

// 成对遍历用的半平面偏移，对应 COHESION_OFFSETS 的下标：(1,0) (0,1) (1,1) (-1,1)。
// 每条相邻边只从一端访问一次，所以这四个方向加上各自的反向量恰好覆盖全部 8 条边，不重复也不遗漏。
static const uint8_t COH_PAIR_IDX[4] = {4, 6, 7, 5};

struct Stats {
  uint32_t commits, blocked, wall;
  uint32_t adds, removes;
  uint32_t addFailFull, addFailBad;
  Stats() : commits(0), blocked(0), wall(0), adds(0), removes(0), addFailFull(0), addFailBad(0) {}
};

enum AuditFlag : uint32_t {
  AUDIT_CAP = 1u << 0,
  AUDIT_DUP_CELL = 1u << 1,
  AUDIT_OOB = 1u << 2,
  AUDIT_PHASE_X = 1u << 3,
  AUDIT_PHASE_Y = 1u << 4,
  AUDIT_OWNER_FWD = 1u << 5,
  AUDIT_OWNER_REV = 1u << 6,
  AUDIT_LIT = 1u << 7,
};

inline void fnvMix(uint64_t &h, uint32_t v) {
  h ^= (uint64_t)v;
  h *= 1099511628211ULL;
}

class PhysicsWorld {
 public:
  enum { MAX_BLOCKS = 64, MAX_CELLS = 64 };

  PhysicsWorld() { init(Params()); }

  bool init(const Params &p);
  void resetStats();

  // ---------- 动态增删 ----------
  int16_t addAt(int cx, int cy);
  int16_t addRandom();
  bool removeNewest();
  void clear();
  uint8_t count() const { return count_; }
  bool isFull() const { return count_ >= (uint8_t)MAX_BLOCKS; }
  void setSeed(uint32_t s) { rng_ = s ? s : 1u; }
  uint32_t seed() const { return rng_; }

  // ---------- 重力 ----------
  void setTiltDeg(double deg);
  void setGravityScreenQ16(int32_t gxUp, int32_t gyUp, int32_t Gq);
  void setGravityUnitQ16(int32_t uxUp, int32_t uyUp, int32_t Gq);
  void setGravityVectorQ16(int32_t gxEngine, int32_t gyEngine);
  void setGravityTotalQ16(int32_t Gq);
  void setGravityTotalFloat(double G);
  void setFrictionAuto(bool on);
  void recomputeFriction();

  int32_t gravityXQ16() const { return gx_; }
  int32_t gravityYQ16() const { return gy_; }
  int32_t gravityTotalQ16() const { return fx::fromFloat(G_); }
  int32_t normalQ16() const { return gn_; }
  int32_t fdvQ16() const { return fdv_; }
  int32_t plateFrictionQ16() const { return mup_; }
  int32_t pairFrictionQ16() const { return mu_; }
  int32_t dampingQ16() const { return damp_; }
  int32_t vmaxQ16() const { return vmax_; }

  // ---------- 转动 ----------
  bool setSpin(double rps, double rpsDot, const double *axis, const double *pivot);
  bool setSpinQ16(int32_t wx, int32_t wy, int32_t wz,
                  int32_t wdx, int32_t wdy, int32_t wdz);
  bool setSpinScreenQ16(int32_t wx, int32_t wy, int32_t wz,
                        int32_t wdx, int32_t wdy, int32_t wdz);
  void stopSpin();
  void setPivotQ16(int32_t x, int32_t y);

  // ---------- 参数调节（其余编码器映射以后接这里） ----------
  void setPlateFrictionQ16(int32_t mu);
  void setPairFrictionQ16(int32_t mu) { mu_ = mu; }
  void setRestitutionQ16(int32_t e) { e_ = e; }
  void setRestitutionSlopQ16(int32_t s) { eslop_ = s; }
  void setDampingQ16(int32_t d) { damp_ = d; }
  // e 只存在 e_ 里、不回写 p_.restitution，所以读当前值必须走这个 getter，不能读 params()。
  int32_t restitutionQ16() const { return e_; }
  // ---------- 内聚（水模式） ----------
  bool setCohesionFloat(double c);
  bool setCohesionDiagFloat(double d);
  bool setCohesionDampingFloat(double d);
  double cohesion() const { return p_.cohesion; }
  double cohesionDiag() const { return p_.cohesionDiag; }
  double cohesionDamping() const { return p_.cohesionDamping; }
  // ---------- 静水压强（水模式） ----------
  bool setHydrostaticFloat(double h);
  double hydrostatic() const { return p_.hydrostatic; }
  bool setVmaxQ16(int32_t v);
  void setSubsteps(uint8_t n) { if (n > 0) p_.substeps = n; }
  void setGateOrder(GateOrder o) { p_.gateOrder = o; }
  void setAxisOrder(AxisOrder o) { p_.axisOrder = o; }

  // ---------- 步进 ----------
  void substep();
  void frame();
  void advance(uint32_t frames);

  // ---------- 观测 ----------
  uint64_t mask() const;
  uint8_t litCount() const;
  int64_t kineticRaw() const;
  double kinetic() const;
  uint64_t stateHash() const;
  uint32_t audit() const;
  const Stats &stats() const { return stats_; }
  const Block &block(uint8_t i) const { return blocks_[i]; }
  int16_t ownerAt(int cx, int cy) const;
  const Params &params() const { return p_; }
  double frameHz() const { return p_.substepHz / (double)p_.substeps; }

 private:
  uint32_t nextRand();
  bool rebuildCohesion();
  void cohesionPairs(int32_t *ax, int32_t *ay) const;
  void updatePressure(int32_t gx, int32_t gy);
  void buildPressureDepth();
  void pressureAccel(uint8_t i, int32_t &fx, int32_t &fy) const;
  void plateFriction(Block &b);
  int32_t pen(uint8_t i) const;
  void gate(uint8_t i, uint8_t axis);
  void wallContact(Block &b, uint8_t axis, int8_t s);
  void wallFriction(Block &b, uint8_t taxis, int32_t vn);
  void pairContact(uint8_t i, uint8_t j, uint8_t axis, int8_t s);
  void pairFriction(Block &a, Block &b, uint8_t taxis, int32_t jn);

  Params p_;
  uint8_t W_, H_;
  uint8_t count_;
  Block blocks_[MAX_BLOCKS];
  int16_t owner_[MAX_CELLS];   // -1 = 空；否则是 blocks_ 的下标
  Stats stats_;
  uint32_t rng_;
  uint8_t axisFlip_;

  int32_t gx_, gy_;            // 平面内重力分量，引擎系（y 向下）
  int32_t g_;                  // Params.gravity 的定点值，setTiltDeg 用它
  double G_;                   // 总重力，保留双精度以对齐 Python 的 math.sqrt
  int32_t gn_, mup_, fdv_;
  int32_t e_, eslop_, mu_, damp_, vmax_, eps_;
  bool frictionAuto_;
  int32_t aco_, acod_, cdamp_;      // 轴向 / 对角键力，键轴阻尼
  int32_t cohForce_[8][2];          // 预计算的八个方向力
  bool cohesionOn_;
  int32_t press_;                   // 静水压强增益，Q16.16
  bool pressOn_;                    // 增益非零，才需要每子步算压强
  bool pressActive_;                // 本子步有效（增益与 g∥ 都非零）
  int32_t pressKg_;                 // 增益 · |g∥|
  int32_t pressUx_, pressUy_;       // 真实重力单位向量（Q16.16，指向重力一侧）
  int32_t pressRayX_, pressRayY_;   // -0.5 · 单位重力，半格步长
  int32_t pressDepth_[MAX_CELLS];   // 水头表，单位格，Q16.16；空格为 0

  int32_t pivx_, pivy_;
  int32_t omega_[3];
  int32_t omegaDot_[3];
};

inline bool PhysicsWorld::init(const Params &p) {
  if (p.worldW == 0 || p.worldH == 0) return false;
  if ((uint16_t)p.worldW * (uint16_t)p.worldH > (uint16_t)MAX_CELLS) return false;
  if (p.substeps == 0) return false;
  if (!(p.substepHz > 0.0)) return false;
  if (!(p.vmax > 0.0 && p.vmax < 1.0)) return false;
  if (p.gateOrder != GATE_PEN && p.gateOrder != GATE_INDEX) return false;
  if (p.axisOrder != AXIS_VEL && p.axisOrder != AXIS_XY &&
      p.axisOrder != AXIS_YX && p.axisOrder != AXIS_ALT) return false;

  p_ = p;
  W_ = p.worldW;
  H_ = p.worldH;
  count_ = 0;
  memset(owner_, 0xFF, sizeof(owner_));   // 0xFFFF = -1，补码
  memset(blocks_, 0, sizeof(blocks_));
  resetStats();
  rng_ = 1;
  axisFlip_ = 0;

  g_ = fx::fromFloat(p.gravity);
  G_ = p.gravityTotal;
  gx_ = 0;
  gy_ = g_;
  // 初始化这一步必须与 Python 的 __init__ 逐位一致：gn = from_float(sqrt(max(0, G² − to_float(g)²)))。G 用双精度，不先量化；动态路径才用量化后的 G。
  const double gfl = fx::toFloat(g_);
  const double n0 = sqrt(fmax(0.0, p.gravityTotal * p.gravityTotal - gfl * gfl));
  gn_ = fx::fromFloat(n0);

  mup_ = fx::fromFloat(p.plateFriction);
  fdv_ = fx::mul(mup_, gn_);
  e_ = fx::fromFloat(p.restitution);
  eslop_ = fx::fromFloat(p.restitutionSlop);
  mu_ = fx::fromFloat(p.friction);
  damp_ = fx::fromFloat(p.damping);
  vmax_ = fx::fromFloat(p.vmax);
  if (vmax_ <= 0 || vmax_ >= fx::ONE) return false;
  eps_ = fx::fromFloat(1e-4);
  frictionAuto_ = true;
  if (!rebuildCohesion()) return false;
  if (p.hydrostatic < 0.0) return false;
  press_ = fx::fromFloat(p.hydrostatic);
  pressOn_ = (press_ != 0);
  pressActive_ = false;
  pressKg_ = 0;
  pressUx_ = 0;
  pressUy_ = fx::ONE;
  pressRayX_ = 0;
  pressRayY_ = -(fx::ONE >> 1);
  memset(pressDepth_, 0, sizeof(pressDepth_));

  pivx_ = fx::fromFloat(p.pivotX);
  pivy_ = fx::fromFloat(p.pivotY);
  omega_[0] = omega_[1] = omega_[2] = 0;
  omegaDot_[0] = omegaDot_[1] = omegaDot_[2] = 0;
  return true;
}

inline void PhysicsWorld::resetStats() { stats_ = Stats(); }

inline uint32_t PhysicsWorld::nextRand() {
  uint32_t x = rng_;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  rng_ = x;
  return x;
}

inline int16_t PhysicsWorld::addAt(int cx, int cy) {
  if (count_ >= (uint8_t)MAX_BLOCKS) { stats_.addFailFull++; return -1; }
  if (cx < 0 || cx >= (int)W_ || cy < 0 || cy >= (int)H_) { stats_.addFailBad++; return -1; }
  const int16_t idx = (int16_t)(cx + cy * (int)W_);
  if (owner_[idx] != -1) { stats_.addFailBad++; return -1; }

  Block &b = blocks_[count_];
  b.cx = (int16_t)cx;
  b.cy = (int16_t)cy;
  b.px = fx::fromFloat((double)cx + 0.5);
  b.py = fx::fromFloat((double)cy + 0.5);
  b.vx = 0;
  b.vy = 0;
  owner_[idx] = (int16_t)count_;
  stats_.adds++;
  count_++;
  return (int16_t)(count_ - 1);
}

inline int16_t PhysicsWorld::addRandom() {
  if (count_ >= (uint8_t)MAX_BLOCKS) { stats_.addFailFull++; return -1; }
  const uint16_t total = (uint16_t)((uint16_t)W_ * (uint16_t)H_);
  const uint16_t freeCount = (uint16_t)(total - count_);
  uint16_t pick = (uint16_t)(nextRand() % (uint32_t)freeCount);
  for (uint16_t idx = 0; idx < total; idx++) {
    if (owner_[idx] != -1) continue;
    if (pick == 0) return addAt((int)(idx % W_), (int)(idx / W_));
    pick--;
  }
  stats_.addFailBad++;   // 理论上到不了这里；到了说明 owner_ 与 count_ 已经不一致
  return -1;
}

inline bool PhysicsWorld::removeNewest() {
  if (count_ == 0) return false;
  count_--;
  const Block &b = blocks_[count_];
  owner_[b.cx + b.cy * (int)W_] = -1;
  stats_.removes++;
  return true;
}

inline void PhysicsWorld::clear() {
  count_ = 0;
  memset(owner_, 0xFF, sizeof(owner_));
}

// ---------- 重力 ----------

inline void PhysicsWorld::setTiltDeg(double deg) {
  const double r = deg * 0.017453292519943295769;
  gx_ = fx::mul(g_, fx::fromFloat(sin(r)));
  gy_ = fx::mul(g_, fx::fromFloat(cos(r)));
  // 明确不调用 recomputeFriction()：Python 的 set_tilt() 也只改方向，gn/fdv 停在初始化值。要实时 N 用下面的 setGravityScreenQ16()。
}

inline void PhysicsWorld::setGravityScreenQ16(int32_t gxUp, int32_t gyUp, int32_t Gq) {
  if (Gq < 0) Gq = 0;
  gx_ = gxUp;
  gy_ = fx::neg(gyUp);   // 屏幕系 y 向上 → 引擎 y 向下，全工程唯一一处重力 y 翻转
  G_ = (double)Gq / 65536.0;
  if (frictionAuto_) recomputeFriction();
}

inline void PhysicsWorld::setGravityUnitQ16(int32_t uxUp, int32_t uyUp, int32_t Gq) {
  if (Gq < 0) Gq = 0;
  double ux = (double)uxUp / 65536.0;
  double uy = (double)uyUp / 65536.0;
  const double m2 = ux * ux + uy * uy;
  if (m2 > 1.0) {
    const double s = 1.0 / sqrt(m2);
    ux *= s;
    uy *= s;
  }
  gx_ = fx::mul(Gq, fx::fromFloat(ux));
  gy_ = fx::neg(fx::mul(Gq, fx::fromFloat(uy)));   // 同上，屏幕系 y 向上 → 引擎 y 向下
  G_ = (double)Gq / 65536.0;
  if (frictionAuto_) recomputeFriction();
}

inline void PhysicsWorld::setGravityVectorQ16(int32_t gxEngine, int32_t gyEngine) {
  gx_ = gxEngine;
  gy_ = gyEngine;
  if (frictionAuto_) recomputeFriction();
}

inline void PhysicsWorld::setGravityTotalQ16(int32_t Gq) {
  if (Gq < 0) Gq = 0;
  G_ = (double)Gq / 65536.0;
  if (frictionAuto_) recomputeFriction();
}

inline void PhysicsWorld::setGravityTotalFloat(double G) {
  G_ = G > 0.0 ? G : 0.0;
  if (frictionAuto_) recomputeFriction();
}

inline void PhysicsWorld::setFrictionAuto(bool on) {
  frictionAuto_ = on;
  if (on) recomputeFriction();
}

inline void PhysicsWorld::recomputeFriction() {
  const double gx = (double)gx_ / 65536.0;
  const double gy = (double)gy_ / 65536.0;
  const double gp2 = gx * gx + gy * gy;
  const double G2 = G_ * G_;
  const double n = (gp2 < G2) ? sqrt(G2 - gp2) : 0.0;   // 法向压力不能为负；|g∥| ≥ G 时摩擦关闭
  gn_ = fx::fromFloat(n);
  fdv_ = fx::mul(mup_, gn_);
}

inline void PhysicsWorld::setPlateFrictionQ16(int32_t mu) {
  mup_ = mu;
  fdv_ = fx::mul(mup_, gn_);
}

inline bool PhysicsWorld::setVmaxQ16(int32_t v) {
  if (v <= 0 || v >= fx::ONE) return false;
  vmax_ = v;
  return true;
}

// ---------- 内聚 ----------

inline bool PhysicsWorld::rebuildCohesion() {
  if (p_.cohesion < 0.0) return false;
  if (p_.cohesionDiag < 0.0 || p_.cohesionDiag > 1.0) return false;
  if (p_.cohesionDamping < 0.0 || p_.cohesionDamping > 1.0) return false;
  aco_ = fx::fromFloat(p_.cohesion);
  acod_ = fx::fromFloat(p_.cohesion * p_.cohesionDiag);
  cdamp_ = fx::fromFloat(p_.cohesionDamping);
  cohesionOn_ = (cdamp_ != 0);
  for (uint8_t k = 0; k < 8; k++) {
    const int32_t a = COHESION_OFFSETS[k].diag ? acod_ : aco_;
    cohForce_[k][0] = fx::mul(a, COHESION_OFFSETS[k].ux);
    cohForce_[k][1] = fx::mul(a, COHESION_OFFSETS[k].uy);
    if (cohForce_[k][0] != 0 || cohForce_[k][1] != 0) cohesionOn_ = true;
  }
  return true;
}

inline bool PhysicsWorld::setCohesionFloat(double c) {
  if (c < 0.0) return false;
  p_.cohesion = c;
  return rebuildCohesion();
}

inline bool PhysicsWorld::setCohesionDiagFloat(double d) {
  if (d < 0.0 || d > 1.0) return false;
  p_.cohesionDiag = d;
  return rebuildCohesion();
}

inline bool PhysicsWorld::setCohesionDampingFloat(double d) {
  if (d < 0.0 || d > 1.0) return false;
  p_.cohesionDamping = d;
  return rebuildCohesion();
}

// 块与块之间的内聚力与键轴阻尼，成对施加：每条相邻边只算一次，两个块同时更新。
//
// 逐位等价于原来的「每个块各自扫八邻域」，依据三条：
//   1. 定点累加是整数加法，可交换、可结合，求和顺序不影响结果；
//   2. fx::mul() 的取整向远离零方向，严格奇，mul(−a, b) == −mul(a, b)；
//   3. 阻尼项在反方向上恰好是同一个数：u' = −u，而 vrel' = (v_j − v_i)·u' = vrel，
//      所以 d' = d，两个方向的推力 d·u' = −(d·u) 等大反向。
// 力项不用额外论证：cohForce_ 里每一对相反方向的向量本来就是精确的相反数。
//
// 省下来的是：每条边的阻尼乘法只做一次（原来两个块各做一次），以及一半的邻格查表。
inline void PhysicsWorld::cohesionPairs(int32_t *ax, int32_t *ay) const {
  for (uint8_t i = 0; i < count_; i++) {
    const Block &b = blocks_[i];
    const int16_t cx = b.cx;
    const int16_t cy = b.cy;
    for (uint8_t m = 0; m < 4; m++) {
      const uint8_t k = COH_PAIR_IDX[m];
      const CohesionOffset &o = COHESION_OFFSETS[k];
      const int16_t nx = (int16_t)(cx + o.dx);
      const int16_t ny = (int16_t)(cy + o.dy);
      if (nx < 0 || nx >= (int16_t)W_ || ny < 0 || ny >= (int16_t)H_) continue;
      const int16_t j = owner_[nx + ny * (int16_t)W_];
      if (j < 0) continue;
      const int32_t cfx = cohForce_[k][0];
      const int32_t cfy = cohForce_[k][1];
      ax[i] = fx::add(ax[i], cfx);
      ay[i] = fx::add(ay[i], cfy);
      ax[j] = fx::sub(ax[j], cfx);
      ay[j] = fx::sub(ay[j], cfy);
      if (cdamp_ != 0) {
        const Block &o2 = blocks_[j];
        const int32_t vrel = fx::add(fx::mul(fx::sub(b.vx, o2.vx), o.ux),
                                     fx::mul(fx::sub(b.vy, o2.vy), o.uy));
        const int32_t d = fx::mul(cdamp_, vrel);
        const int32_t jx = fx::mul(d, o.ux);
        const int32_t jy = fx::mul(d, o.uy);
        ax[i] = fx::sub(ax[i], jx);
        ay[i] = fx::sub(ay[i], jy);
        ax[j] = fx::add(ax[j], jx);
        ay[j] = fx::add(ay[j], jy);
      }
    }
  }
}

// ---------- 静水压强 ----------

inline bool PhysicsWorld::setHydrostaticFloat(double h) {
  if (h < 0.0) return false;
  p_.hydrostatic = h;
  press_ = fx::fromFloat(h);
  pressOn_ = (press_ != 0);
  if (!pressOn_) pressActive_ = false;
  return true;
}

inline void PhysicsWorld::updatePressure(int32_t gx, int32_t gy) {
  if (!pressOn_) {
    pressActive_ = false;
    return;
  }
  const int32_t gmag = fx::hypot(gx, gy);
  if (gmag == 0) {
    pressActive_ = false;
    return;
  }
  // 真实重力方向，归一化后不再量化：射线可以沿任意角度走。
  pressUx_ = fx::div(gx, gmag);
  pressUy_ = fx::div(gy, gmag);
  pressRayX_ = fx::halve(fx::neg(pressUx_));
  pressRayY_ = fx::halve(fx::neg(pressUy_));
  pressKg_ = fx::mul(press_, gmag);
  pressActive_ = (pressKg_ != 0);
  if (pressActive_) buildPressureDepth();
}

inline void PhysicsWorld::buildPressureDepth() {
  memset(pressDepth_, 0, sizeof(pressDepth_));
  const int32_t half = fx::ONE >> 1;
  for (uint8_t i = 0; i < count_; i++) {
    const Block &b = blocks_[i];
    int32_t px = fx::add(fx::fromFloat((double)b.cx), half);
    int32_t py = fx::add(fx::fromFloat((double)b.cy), half);
    uint8_t n = 0;
    for (uint8_t k = 0; k < 24; k++) {
      const int32_t qx = fx::add(px, pressRayX_);
      const int32_t qy = fx::add(py, pressRayY_);
      const int16_t nx = (int16_t)fx::cellOf(qx);
      const int16_t ny = (int16_t)fx::cellOf(qy);
      if (nx < 0 || nx >= (int16_t)W_ || ny < 0 || ny >= (int16_t)H_) break;
      if (owner_[nx + ny * (int16_t)W_] < 0) break;
      px = qx;
      py = qy;
      n++;
    }
    // (px,py) 是最后一个仍有水的采样点。求射线穿过该格边界的比例 f ∈ [0,1]，把水头做成连续值；否则射线跨格时会跳半格。
    const int16_t ox = (int16_t)fx::cellOf(px);
    const int16_t oy = (int16_t)fx::cellOf(py);
    const int16_t cx = (int16_t)fx::cellOf(fx::add(px, pressRayX_));
    const int16_t cy = (int16_t)fx::cellOf(fx::add(py, pressRayY_));
    int32_t bestF = 0;
    bool haveF = false;
    if (cx != ox && pressRayX_ != 0) {
      const int32_t bx = fx::fromFloat((double)(cx > ox ? ox + 1 : ox));
      const int32_t f = fx::div(fx::sub(bx, px), pressRayX_);
      if (f >= 0) {
        bestF = f;
        haveF = true;
      }
    }
    if (cy != oy && pressRayY_ != 0) {
      const int32_t by = fx::fromFloat((double)(cy > oy ? oy + 1 : oy));
      const int32_t f = fx::div(fx::sub(by, py), pressRayY_);
      if (f >= 0 && (!haveF || f < bestF)) {
        bestF = f;
        haveF = true;
      }
    }
    const int32_t nq = fx::add(fx::fromFloat((double)n), haveF ? bestF : 0);   // 半格计数，含小数
    pressDepth_[b.cx + b.cy * (int16_t)W_] =
        (nq > fx::ONE) ? fx::halve(fx::sub(nq, fx::ONE)) : 0;
  }
}

inline void PhysicsWorld::pressureAccel(uint8_t i, int32_t &fx, int32_t &fy) const {
  const Block &b = blocks_[i];
  const int32_t own = pressDepth_[b.cx + b.cy * (int16_t)W_];
  // 墙外返回自身水头：墙的法向反作用平衡掉这一侧，不计压差。
  auto neighbor = [&](int16_t x, int16_t y) -> int32_t {
    if (x < 0 || x >= (int16_t)W_ || y < 0 || y >= (int16_t)H_) return own;
    const int16_t idx = (int16_t)(x + y * (int16_t)W_);
    return (owner_[idx] >= 0) ? pressDepth_[idx] : 0;
  };
  const int32_t gcx = fx::sub(neighbor((int16_t)(b.cx + 1), b.cy), neighbor((int16_t)(b.cx - 1), b.cy));
  const int32_t gcy = fx::sub(neighbor(b.cx, (int16_t)(b.cy + 1)), neighbor(b.cx, (int16_t)(b.cy - 1)));
  // 压力梯度里沿重力那一半正好抵消重力，已经由接触与法向支撑处理；这里只留垂直分量。
  const int32_t gpar = fx::add(fx::mul(gcx, pressUx_), fx::mul(gcy, pressUy_));
  const int32_t tx = fx::sub(gcx, fx::mul(gpar, pressUx_));
  const int32_t ty = fx::sub(gcy, fx::mul(gpar, pressUy_));
  fx = fx::neg(fx::mul(pressKg_, tx));
  fy = fx::neg(fx::mul(pressKg_, ty));
  // 两侧都是自由面、水头也相等时是真正的对称简并：按「朝板心一侧倒」做确定性破缺。
  if (tx == 0 && ty == 0 && own > 0) {
    const int32_t sx = fx::neg(pressUy_);
    const int32_t sy = pressUx_;
    int32_t bestP = 0, bestM = 0;
    int8_t pdx = 0, pdy = 0, mdx = 0, mdy = 0;
    bool haveP = false, haveM = false;
    for (uint8_t k = 0; k < 8; k++) {
      const CohesionOffset &o = COHESION_OFFSETS[k];
      const int32_t dot = fx::add(fx::mul(fx::fromFloat((double)o.dx), sx),
                                  fx::mul(fx::fromFloat((double)o.dy), sy));
      if (!haveP || dot > bestP) {
        bestP = dot;
        pdx = o.dx;
        pdy = o.dy;
        haveP = true;
      }
      if (!haveM || dot < bestM) {
        bestM = dot;
        mdx = o.dx;
        mdy = o.dy;
        haveM = true;
      }
    }
    auto empty = [&](int16_t x, int16_t y) -> bool {
      return x >= 0 && x < (int16_t)W_ && y >= 0 && y < (int16_t)H_ &&
             owner_[x + y * (int16_t)W_] < 0;
    };
    if (haveP && haveM && empty((int16_t)(b.cx + pdx), (int16_t)(b.cy + pdy)) &&
        empty((int16_t)(b.cx + mdx), (int16_t)(b.cy + mdy))) {
      const int32_t dcx = fx::sub(fx::fromFloat((W_ - 1) * 0.5), fx::fromFloat((double)b.cx));
      const int32_t dcy = fx::sub(fx::fromFloat((H_ - 1) * 0.5), fx::fromFloat((double)b.cy));
      const int32_t toward = fx::add(fx::mul(dcx, sx), fx::mul(dcy, sy));
      const int32_t f = fx::mul(pressKg_, toward > 0 ? own : fx::neg(own));
      fx = fx::add(fx, fx::mul(f, sx));
      fy = fx::add(fy, fx::mul(f, sy));
    }
  }
}

// ---------- 转动 ----------

inline bool PhysicsWorld::setSpin(double rps, double rpsDot, const double *axis, const double *pivot) {
  const double hz = p_.substepHz;
  if (!(hz > 0.0)) return false;
  if (pivot != 0) {
    pivx_ = fx::fromFloat(pivot[0]);
    pivy_ = fx::fromFloat(pivot[1]);
  }
  const double ax = (axis != 0) ? axis[0] : 0.0;
  const double ay = (axis != 0) ? axis[1] : 0.0;
  const double az = (axis != 0) ? axis[2] : 1.0;
  const double n = sqrt(ax * ax + ay * ay + az * az);
  if (n == 0.0) return false;
  const double ux = ax / n, uy = ay / n, uz = az / n;
  const double w = 2.0 * 3.14159265358979323846 * rps / hz;
  const double wd = 2.0 * 3.14159265358979323846 * rpsDot / (hz * hz);
  if (fabs(w) >= 1.0 || fabs(wd) >= 10.0) return false;
  omega_[0] = fx::fromFloat(w * ux);
  omega_[1] = fx::fromFloat(w * uy);
  omega_[2] = fx::fromFloat(w * uz);
  omegaDot_[0] = fx::fromFloat(wd * ux);
  omegaDot_[1] = fx::fromFloat(wd * uy);
  omegaDot_[2] = fx::fromFloat(wd * uz);
  return true;
}

inline bool PhysicsWorld::setSpinQ16(int32_t wx, int32_t wy, int32_t wz,
                                     int32_t wdx, int32_t wdy, int32_t wdz) {
  if (fx::iabs(wx) >= fx::ONE || fx::iabs(wy) >= fx::ONE || fx::iabs(wz) >= fx::ONE) return false;
  if (fx::iabs(wdx) >= 10 * fx::ONE || fx::iabs(wdy) >= 10 * fx::ONE || fx::iabs(wdz) >= 10 * fx::ONE) return false;
  omega_[0] = wx;
  omega_[1] = wy;
  omega_[2] = wz;
  omegaDot_[0] = wdx;
  omegaDot_[1] = wdy;
  omegaDot_[2] = wdz;
  return true;
}

// 屏幕系 → 引擎系。屏幕系 x 右、y 上、n 出屏；引擎系 x 右、y 下、z 入屏。
// 两者都是右手系，这一变换是绕 x 轴 180°，det = +1，所以角速度赝矢量按普通矢量同样翻 y、z。
// 引擎 +z 就是屏幕上的顺时针，与 sim/world2.py 的默认转轴约定一致。
inline bool PhysicsWorld::setSpinScreenQ16(int32_t wx, int32_t wy, int32_t wz,
                                           int32_t wdx, int32_t wdy, int32_t wdz) {
  return setSpinQ16(wx, fx::neg(wy), fx::neg(wz), wdx, fx::neg(wdy), fx::neg(wdz));
}

inline void PhysicsWorld::stopSpin() {
  omega_[0] = omega_[1] = omega_[2] = 0;
  omegaDot_[0] = omegaDot_[1] = omegaDot_[2] = 0;
}

inline void PhysicsWorld::setPivotQ16(int32_t x, int32_t y) {
  pivx_ = x;
  pivy_ = y;
}

// ---------- 一个子步 ----------

inline void PhysicsWorld::plateFriction(Block &b) {
  if (fdv_ <= 0) return;
  const int32_t vx = b.vx;
  const int32_t vy = b.vy;
  const int32_t s = fx::hypot(vx, vy);
  if (s <= fdv_) {
    b.vx = 0;
    b.vy = 0;
    return;
  }
  const int32_t k = fx::div(fx::sub(s, fdv_), s);
  b.vx = fx::mul(vx, k);
  b.vy = fx::mul(vy, k);
}

inline int32_t PhysicsWorld::pen(uint8_t i) const {
  const Block &b = blocks_[i];
  const int32_t mid = fx::ONE >> 1;
  const int32_t cxm = fx::add(fx::fromFloat((double)b.cx), mid);
  const int32_t cym = fx::add(fx::fromFloat((double)b.cy), mid);
  return fx::add(fx::iabs(fx::sub(b.px, cxm)), fx::iabs(fx::sub(b.py, cym)));
}

inline void PhysicsWorld::substep() {
  const int32_t gx = gx_;
  const int32_t gy = gy_;
  const int32_t wx = omega_[0], wy = omega_[1], wz = omega_[2];
  const int32_t wdx = omegaDot_[0], wdy = omegaDot_[1], wdz = omegaDot_[2];
  const bool spin = (wx != 0 || wy != 0 || wz != 0 || wdx != 0 || wdy != 0 || wdz != 0);

  int32_t twz = 0, cross = 0;
  if (spin) {
    twz = fx::add(wz, wz);
    cross = fx::mul(wx, wy);
  }

  // 内聚力与静水压强必须先用全体块的旧状态算完，再统一施加。边算边改会让先处理到的块污染邻居用于阻尼的速度，结果会依赖块的存储顺序。
  // 内聚走成对路径，一条边只算一次，所以结果是整场数组而不是逐块返回值；压强仍是逐块量，按「内聚在前、压强在后」叠加。
  updatePressure(gx, gy);
  const bool extraOn = (cohesionOn_ || pressActive_);
  int32_t extX[MAX_BLOCKS], extY[MAX_BLOCKS];
  if (extraOn) {
    memset(extX, 0, sizeof(extX));
    memset(extY, 0, sizeof(extY));
    if (cohesionOn_) cohesionPairs(extX, extY);
    if (pressActive_) {
      for (uint8_t i = 0; i < count_; i++) {
        int32_t px = 0, py = 0;
        pressureAccel(i, px, py);
        extX[i] = fx::add(extX[i], px);
        extY[i] = fx::add(extY[i], py);
      }
    }
  }

  for (uint8_t i = 0; i < count_; i++) {
    Block &b = blocks_[i];
    if (spin) {
      const int32_t rx = fx::sub(b.px, pivx_);
      const int32_t ry = fx::sub(b.py, pivy_);
      // 离心：平面内是 [[Ωy²+Ωz², −ΩxΩy], [−ΩxΩy, Ωx²+Ωz²]]·r。对角项用 mulSq 一次取整，避免 ω 小时 ω² 被量化抹掉。
      const int32_t cenX = fx::sub(fx::add(fx::mulSq(wy, rx), fx::mulSq(wz, rx)), fx::mul(cross, ry));
      const int32_t cenY = fx::sub(fx::add(fx::mulSq(wx, ry), fx::mulSq(wz, ry)), fx::mul(cross, rx));
      // 科氏 +2Ωz(v_y, −v_x)；欧拉 +Ω̇z(r_y, −r_x)。Ωx、Ωy、Ω̇x、Ω̇y 的那部分垂直纸面，被支持力吃掉。
      int32_t ax = fx::add(fx::add(gx, cenX), fx::add(fx::mul(twz, b.vy), fx::mul(wdz, ry)));
      int32_t ay = fx::sub(fx::add(gy, cenY), fx::add(fx::mul(twz, b.vx), fx::mul(wdz, rx)));
      if (extraOn) {
        ax = fx::add(ax, extX[i]);
        ay = fx::add(ay, extY[i]);
      }
      b.vx = fx::clamp(fx::mul(fx::add(b.vx, ax), damp_), -vmax_, vmax_);
      b.vy = fx::clamp(fx::mul(fx::add(b.vy, ay), damp_), -vmax_, vmax_);
    } else {
      int32_t ax = gx;
      int32_t ay = gy;
      if (extraOn) {
        ax = fx::add(ax, extX[i]);
        ay = fx::add(ay, extY[i]);
      }
      b.vx = fx::clamp(fx::mul(fx::add(b.vx, ax), damp_), -vmax_, vmax_);
      b.vy = fx::clamp(fx::mul(fx::add(b.vy, ay), damp_), -vmax_, vmax_);
    }
    plateFriction(b);       // 体表力，在跨格门之前
    b.px = fx::add(b.px, b.vx);
    b.py = fx::add(b.py, b.vy);
  }

  // 跨格门：唯一改写格号的地方。pen 排序必须稳定降序，Python 的 sorted(reverse=True) 对相等键保持原序；std::sort 不稳定，不能用。
  uint8_t order[MAX_BLOCKS];
  if (p_.gateOrder == GATE_PEN) {
    int32_t key[MAX_BLOCKS];
    for (uint8_t i = 0; i < count_; i++) {
      order[i] = i;
      key[i] = pen(i);   // 键必须在门处理开始前快照，门处理会改 px/py
    }
    for (uint8_t i = 1; i < count_; i++) {
      const uint8_t v = order[i];
      const int32_t kv = key[v];
      int j = (int)i - 1;
      while (j >= 0 && key[order[j]] < kv) {   // 严格小于：相等不交换
        order[j + 1] = order[j];
        --j;
      }
      order[j + 1] = v;
    }
  } else {
    for (uint8_t i = 0; i < count_; i++) order[i] = i;
  }

  const AxisOrder mode = p_.axisOrder;
  uint8_t ax0 = 0, ax1 = 1;
  if (mode == AXIS_ALT) {
    ax0 = axisFlip_ ? 1 : 0;
    ax1 = (uint8_t)(1 - ax0);
    axisFlip_ = axisFlip_ ? 0 : 1;
  } else if (mode == AXIS_YX) {
    ax0 = 1;
    ax1 = 0;
  }

  for (uint8_t k = 0; k < count_; k++) {
    const uint8_t i = order[k];
    if (mode == AXIS_VEL) {
      const Block &b = blocks_[i];
      if (fx::iabs(b.vx) >= fx::iabs(b.vy)) {
        gate(i, 0);
        gate(i, 1);
      } else {
        gate(i, 1);
        gate(i, 0);
      }
    } else {
      gate(i, ax0);
      gate(i, ax1);
    }
  }
}

inline void PhysicsWorld::gate(uint8_t i, uint8_t axis) {
  Block &b = blocks_[i];
  const int16_t cur = (axis == 0) ? b.cx : b.cy;
  const int32_t pos = (axis == 0) ? b.px : b.py;
  const int16_t ncells = (axis == 0) ? (int16_t)W_ : (int16_t)H_;
  const int16_t t = (int16_t)fx::cellOf(pos);

  if (t == cur) {
    // 左/上边界落在 cell 0 的闭端：位置恰好走到 0 时格号仍是 0，门看不见，但物理上已经贴墙。补一次墙判定，否则左右镜像不再等变。
    if (pos == 0) {
      const int32_t vn = (axis == 0) ? fx::neg(b.vx) : fx::neg(b.vy);
      if (vn > 0) {
        if (axis == 0) b.px = eps_; else b.py = eps_;
        wallContact(b, axis, -1);
        stats_.wall++;
      }
    }
    return;
  }
  const int8_t s = (t > cur) ? 1 : -1;

  // 目标在界外：实心墙
  if (t < 0 || t >= ncells) {
    const int32_t bound = fx::fromFloat(t < 0 ? 0.0 : (double)ncells);
    const int32_t npos = (s < 0) ? fx::add(bound, eps_) : fx::sub(bound, eps_);
    if (axis == 0) b.px = npos; else b.py = npos;
    wallContact(b, axis, s);
    stats_.wall++;
    return;
  }

  // 目标格属于另一个块：实心面
  const int16_t other = (axis == 0) ? b.cy : b.cx;
  const int16_t curIdx = (axis == 0) ? (int16_t)(cur + other * (int)W_) : (int16_t)(other + cur * (int)W_);
  const int16_t tgtIdx = (axis == 0) ? (int16_t)(t + other * (int)W_) : (int16_t)(other + t * (int)W_);
  const int16_t j = owner_[tgtIdx];
  if (j == -1) {
    owner_[curIdx] = -1;
    if (axis == 0) b.cx = t; else b.cy = t;
    owner_[tgtIdx] = (int16_t)i;
    stats_.commits++;
    return;
  }

  // 被挡：贴到本格的边上（格边落在整数处）
  const int32_t bound = fx::fromFloat((double)(cur + (s > 0 ? 1 : 0)));
  const int32_t npos = (s > 0) ? fx::sub(bound, eps_) : fx::add(bound, eps_);
  if (axis == 0) b.px = npos; else b.py = npos;
  pairContact(i, (uint8_t)j, axis, s);
  stats_.blocked++;
}

inline void PhysicsWorld::wallContact(Block &b, uint8_t axis, int8_t s) {
  if (axis == 0) {
    const int32_t vn = (s > 0) ? b.vx : fx::neg(b.vx);
    if (vn > 0) {
      b.vx = (vn > eslop_) ? fx::neg(fx::mul(b.vx, e_)) : 0;
      wallFriction(b, 1, vn);
    }
  } else {
    const int32_t vn = (s > 0) ? b.vy : fx::neg(b.vy);
    if (vn > 0) {
      b.vy = (vn > eslop_) ? fx::neg(fx::mul(b.vy, e_)) : 0;
      wallFriction(b, 0, vn);
    }
  }
}

inline void PhysicsWorld::wallFriction(Block &b, uint8_t taxis, int32_t vn) {
  const int32_t vt = (taxis == 0) ? b.vx : b.vy;
  const int32_t lim = fx::mul(mu_, vn);
  const int32_t d = (fx::iabs(vt) <= lim) ? vt : fx::mul((vt > 0) ? fx::ONE : -fx::ONE, lim);
  if (taxis == 0) b.vx = fx::sub(b.vx, d);
  else b.vy = fx::sub(b.vy, d);
}

inline void PhysicsWorld::pairContact(uint8_t i, uint8_t j, uint8_t axis, int8_t s) {
  Block &a = blocks_[i];
  Block &b = blocks_[j];
  const int32_t fs = fx::fromFloat((double)s);
  if (axis == 0) {
    const int32_t vn = fx::mul(fx::sub(a.vx, b.vx), fs);
    if (vn <= 0) return;
    const int32_t e = (vn > eslop_) ? e_ : 0;
    const int32_t jn = fx::halve(fx::mul(fx::add(fx::ONE, e), vn));
    a.vx = fx::sub(a.vx, fx::mul(fs, jn));
    b.vx = fx::add(b.vx, fx::mul(fs, jn));
    pairFriction(a, b, 1, jn);
  } else {
    const int32_t vn = fx::mul(fx::sub(a.vy, b.vy), fs);
    if (vn <= 0) return;
    const int32_t e = (vn > eslop_) ? e_ : 0;
    const int32_t jn = fx::halve(fx::mul(fx::add(fx::ONE, e), vn));
    a.vy = fx::sub(a.vy, fx::mul(fs, jn));
    b.vy = fx::add(b.vy, fx::mul(fs, jn));
    pairFriction(a, b, 0, jn);
  }
}

inline void PhysicsWorld::pairFriction(Block &a, Block &b, uint8_t taxis, int32_t jn) {
  const int32_t va = (taxis == 0) ? a.vx : a.vy;
  const int32_t vb = (taxis == 0) ? b.vx : b.vy;
  const int32_t vrel = fx::sub(va, vb);
  const int32_t lim = fx::mul(mu_, jn);
  const int32_t d = (fx::iabs(vrel) <= lim) ? vrel : fx::mul((vrel > 0) ? fx::ONE : -fx::ONE, lim);
  const int32_t h = fx::halve(d);
  if (taxis == 0) {
    a.vx = fx::sub(a.vx, h);
    b.vx = fx::add(b.vx, h);
  } else {
    a.vy = fx::sub(a.vy, h);
    b.vy = fx::add(b.vy, h);
  }
}

inline void PhysicsWorld::frame() {
  for (uint8_t i = 0; i < p_.substeps; i++) substep();
}

inline void PhysicsWorld::advance(uint32_t frames) {
  while (frames-- > 0) frame();
}

// ---------- 观测 ----------

inline uint64_t PhysicsWorld::mask() const {
  uint64_t m = 0;
  for (uint8_t i = 0; i < count_; i++) {
    const Block &b = blocks_[i];
    m |= (uint64_t)1 << (b.cy * (int)W_ + b.cx);
  }
  return m;
}

inline uint8_t PhysicsWorld::litCount() const {
  return (uint8_t)__builtin_popcountll(mask());
}

inline int64_t PhysicsWorld::kineticRaw() const {
  int64_t sum = 0;
  for (uint8_t i = 0; i < count_; i++) {
    const Block &b = blocks_[i];
    sum += fx::add(fx::mul(b.vx, b.vx), fx::mul(b.vy, b.vy));
  }
  return sum;
}

inline double PhysicsWorld::kinetic() const {
  return (double)kineticRaw() / (double)fx::ONE / 2.0;
}

inline uint64_t PhysicsWorld::stateHash() const {
  uint64_t h = 14695981039346656037ULL;
  fnvMix(h, (uint32_t)count_);
  for (uint8_t i = 0; i < count_; i++) {
    const Block &b = blocks_[i];
    fnvMix(h, (uint32_t)b.px);
    fnvMix(h, (uint32_t)b.py);
    fnvMix(h, (uint32_t)b.vx);
    fnvMix(h, (uint32_t)b.vy);
    fnvMix(h, (uint32_t)(int32_t)b.cx);
    fnvMix(h, (uint32_t)(int32_t)b.cy);
  }
  return h;
}

inline int16_t PhysicsWorld::ownerAt(int cx, int cy) const {
  if (cx < 0 || cx >= (int)W_ || cy < 0 || cy >= (int)H_) return -2;
  return owner_[cx + cy * (int)W_];
}

inline uint32_t PhysicsWorld::audit() const {
  uint32_t f = 0;
  if (count_ > (uint8_t)MAX_BLOCKS) return AUDIT_CAP;
  if ((uint16_t)W_ * (uint16_t)H_ > (uint16_t)MAX_CELLS) return AUDIT_CAP;

  uint64_t seen = 0;
  for (uint8_t i = 0; i < count_; i++) {
    const Block &b = blocks_[i];
    if (b.cx < 0 || b.cx >= (int)W_ || b.cy < 0 || b.cy >= (int)H_) {
      f |= AUDIT_OOB;
      continue;
    }
    const uint16_t idx = (uint16_t)(b.cx + b.cy * (int)W_);
    const uint64_t bit = (uint64_t)1 << idx;
    if (seen & bit) f |= AUDIT_DUP_CELL;
    seen |= bit;

    const int32_t x0 = (int32_t)b.cx * fx::ONE;
    const int32_t y0 = (int32_t)b.cy * fx::ONE;
    if (!(b.px >= x0 && b.px < x0 + fx::ONE)) f |= AUDIT_PHASE_X;
    if (!(b.py >= y0 && b.py < y0 + fx::ONE)) f |= AUDIT_PHASE_Y;
    if (owner_[idx] != (int16_t)i) f |= AUDIT_OWNER_FWD;
  }

  const uint16_t total = (uint16_t)((uint16_t)W_ * (uint16_t)H_);
  for (uint16_t idx = 0; idx < total; idx++) {
    const int16_t j = owner_[idx];
    if (j == -1) continue;
    if (j < 0 || j >= (int16_t)count_) { f |= AUDIT_OWNER_REV; continue; }
    const Block &b = blocks_[j];
    if (b.cx + b.cy * (int)W_ != (int)idx) f |= AUDIT_OWNER_REV;
  }

  if (__builtin_popcountll(mask()) != (int)count_) f |= AUDIT_LIT;
  return f;
}

}  // namespace gw

#endif  // GEWU_PHYSICS_WORLD_H
