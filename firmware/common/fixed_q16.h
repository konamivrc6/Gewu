/*
 * fixed_q16.h —— Q16.16 定点原语，逐条对应 sim/fixed.py
 * =====================================================
 *
 * 目标芯片 ESP32-C3 是 RV32IMC，没有硬件浮点单元，浮点走 libgcc 软件模拟。物理热路径全程定点；这里每个函数的取整方向都直接决定镜像等变性能不能成立，所以注释只写“为什么是这一种写法”。
 *
 * 这个头文件不依赖 Arduino，只用 <stdint.h> 与 <math.h>，将来主机端逐位对照程序可以直接 include。
 */
#ifndef GEWU_FIXED_Q16_H
#define GEWU_FIXED_Q16_H

#include <stdint.h>
#include <math.h>

namespace fx {

constexpr int32_t FRAC_BITS = 16;
constexpr int32_t ONE = 1 << FRAC_BITS;

// int32 回绕。Python 原型在 STRICT 模式下会抛异常，固件里只能按 C 的补码语义回绕；所有运算都显式经 int64 中转，避免有符号溢出是未定义行为。
inline int32_t wrap32(int64_t v) {
  return (int32_t)(uint32_t)((uint64_t)v & 0xFFFFFFFFu);
}

inline int32_t add(int32_t a, int32_t b) { return wrap32((int64_t)a + (int64_t)b); }
inline int32_t sub(int32_t a, int32_t b) { return wrap32((int64_t)a - (int64_t)b); }
inline int32_t neg(int32_t a) { return wrap32(-(int64_t)a); }

// 定点乘，就近取整，**.5 向远离零的方向**。不要写回 (p + 0x8000) >> 16：那个写法对负乘积是向下取整，负乘积占所有乘积的一半，世界里每一次乘法都会往负方向拉一点，左右镜像不再等变。
inline int32_t mul(int32_t a, int32_t b) {
  const int64_t p = (int64_t)a * (int64_t)b;
  const int64_t half = 0x8000;
  if (p >= 0) return wrap32((p + half) >> 16);
  return wrap32(-((-p + half) >> 16));
}

// a²·b，三个因子一次乘完、只取整一次。离心项的 ω²·r 必须用它，不能写 mul(mul(w, w), r)：后者先把 ω² 量化一次，低速转动时离心力会被量化抹成 0。
inline int32_t mulSq(int32_t a, int32_t b) {
  return wrap32((((int64_t)a * (int64_t)a * (int64_t)b) + (1LL << 31)) >> 32);
}

// 定点除。C++ 的有符号 / 本来就向零截断，与 Python 的 _trunc_div 一致。不能写 a << 16：负数的有符号左移在 C++20 前是未定义行为。
inline int32_t div(int32_t a, int32_t b) {
  if (b == 0) return 0;
  return wrap32(((int64_t)a * (int64_t)ONE) / (int64_t)b);
}

// 精确 floor(sqrt(n))，逐位二进制开方。不要用 sqrt()：C3 上是软件浮点，而且末位与 Python 的 math.isqrt 不一定一致。
inline uint64_t isqrt64(uint64_t n) {
  uint64_t res = 0;
  uint64_t bit = (uint64_t)1 << 62;
  while (bit > n) bit >>= 2;
  while (bit != 0) {
    if (n >= res + bit) {
      n -= res + bit;
      res = (res >> 1) + bit;
    } else {
      res >>= 1;
    }
    bit >>= 2;
  }
  return res;
}

// 二维模长。vmax = 0.6 时两个分量各不超过 39322，平方和上界 3.09e9，32 位有符号装不下，必须用 uint64 中间量。
inline int32_t hypot(int32_t a, int32_t b) {
  const int64_t aa = (int64_t)a * (int64_t)a;
  const int64_t bb = (int64_t)b * (int64_t)b;
  return wrap32((int64_t)isqrt64((uint64_t)(aa + bb)));
}

// 对称的除二，向零取整。**不要写 a >> 1**：算术右移对奇数负数是向下取整，切向摩擦冲量正好可正可负，差 1 LSB 就破坏镜像等变。
inline int32_t halve(int32_t a) { return a / 2; }

inline int32_t clamp(int32_t v, int32_t lo, int32_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

inline int32_t iabs(int32_t v) { return v < 0 ? -v : v; }

// 与 Python 的 int(round(x * ONE)) 对齐。Python 的 round 在 .5 上取最近偶数，C 的 lround 是远离零；这个世界的格心全在半整数上，转动一开始 ω̇·r 又恰好落在 .5 上，所以半值规则必须逐位复刻。
inline int32_t fromFloat(double x) {
  const double y = x * 65536.0;
  double f = floor(y);
  const double d = y - f;
  if (d > 0.5) {
    f += 1.0;
  } else if (d == 0.5 && (((int64_t)f) & 1)) {
    f += 1.0;
  }
  return wrap32((int64_t)f);
}

inline double toFloat(int32_t a) { return (double)a / 65536.0; }

// 取所在格号。位置由不变量保证非负，但左/上墙分支要求 cell(-1) = -1，所以必须是算术右移；写成 a / 65536 会把 -1 变成 0，左墙晚一个子步响应。
inline int32_t cellOf(int32_t a) { return a >> 16; }

}  // namespace fx

// 这两条是上面那些写法的前提，编译期钉死。
static_assert((-1LL >> 1) == -1LL, "arithmetic right shift required");
static_assert((-3) / 2 == -1, "truncating division required");
static_assert(sizeof(int32_t) == 4, "int32_t must be 32 bits");

#endif  // GEWU_FIXED_Q16_H
