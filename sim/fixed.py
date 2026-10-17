"""Q16.16 定点数原语。

为什么不用 float：目标芯片 ESP32-C3 是 RV32IMC，**没有硬件浮点单元**，浮点运算
由 libgcc 软件模拟，一次乘法几十个周期；定点数走的是硬件乘法器，快一个数量级。
而且定点跨平台逐位一致——将来若换到带 FPU 的 S3，浮点结果会因融合乘加与精度路
径而变，定点不会。

所有运算按 C 的 int32 语义回绕。STRICT 打开时，回绕前先抛异常：原型阶段宁可直
接炸掉，也不要静默算出垃圾数据——溢出几乎总是标度定错了，不是数据本身的问题。
"""

from __future__ import annotations

import math

FRAC_BITS = 16
ONE = 1 << FRAC_BITS

_INT32_MIN = -(1 << 31)
_INT32_MAX = (1 << 31) - 1
_MASK32 = 0xFFFFFFFF

STRICT = True

# 运算计数器。默认关掉，热路径上每个原语多一次字典查找可以忽略；打开后 `ops` 里记着每种原语被调了多少次，C3 的周期预算要靠这个数字对表。
COUNT = False
ops: dict[str, int] = {}


def tick(name: str) -> None:
    ops[name] = ops.get(name, 0) + 1


def reset_counts() -> None:
    ops.clear()


class Overflow(RuntimeError):
    """运算结果超出 int32。"""


def _wrap(x: int) -> int:
    if _INT32_MIN <= x <= _INT32_MAX:
        return x
    if STRICT:
        raise Overflow(f"int32 溢出：{x}（标度多半定错了）")
    x &= _MASK32
    return x - (1 << 32) if x & 0x80000000 else x


def _trunc_div(num: int, den: int) -> int:
    """C 语义的整除：向零截断。Python 的 // 是向下取整，负数上会差一。"""
    q = abs(num) // abs(den)
    return -q if (num < 0) != (den < 0) else q


def add(a: int, b: int) -> int:
    if COUNT:
        tick("add")
    return _wrap(a + b)


def sub(a: int, b: int) -> int:
    if COUNT:
        tick("sub")
    return _wrap(a - b)


def neg(a: int) -> int:
    if COUNT:
        tick("neg")
    return _wrap(-a)


def mul(a: int, b: int) -> int:
    """定点乘，就近取整，**.5 向远离零的方向**。C 里是：

        int64_t p = (int64_t)a * b;
        return (int32_t)(p >= 0 ? (p + 0x8000) >> 16 : -((-p + 0x8000) >> 16));

    **不要写回算术右移的向下取整。** 向下取整对负乘积有 1 LSB 的系统偏置，而负乘积占所有乘积的一半，于是世界里每一次乘法都往负方向拉一点。对一个左右对称的世界，这表现为左右镜像不再等变：实测左右镜像的质心残差在 72° 的雪崩阈值处达到 2.19 格。

    **也不要写成 `(p + 0x8000) >> 16` 那种半值向上。** 那个写法对负乘积是向下取整到最近，`+72.5` 给 73、`−72.5` 给 −72，取负不交换。平局本来该是测度零的，但在这个世界里不是：所有格心都在半整数上，转动一开始 `ω̇·r` 对每个块都恰好落在 .5 上，于是镜像两侧从第一个子步就差 1 LSB，再被混沌放大成完全不同的末态。半值向远离零取整是严格奇的，`mul(−a, b) == −mul(a, b)` 恒成立。
    """
    if COUNT:
        tick("mul")
    p = a * b
    half = 1 << (FRAC_BITS - 1)
    if p >= 0:
        return _wrap((p + half) >> FRAC_BITS)
    return _wrap(-((-p + half) >> FRAC_BITS))


def mul_sq(a: int, b: int) -> int:
    """a²·b，三个因子一次乘完、只取整一次。C 里是 `(int32_t)((((int64_t)a * a * b) + (1LL << 31)) >> 32)`。

    离心项 ω²·r 用它，不用 `mul(mul(w, w), r)`。后者会先把 ω² 量化一次：ω 小的时候 ω² 本身只剩几十个 LSB，相对误差到百分之几，低速转动的离心力会被量化抹成 0。这里误差始终是半个 LSB。

    int64 的余量：|a| ≤ 1.0（65536）、|b| ≤ 8（524288），乘积上界 2.25e15，离 9.2e18 还有三个数量级。
    """
    if COUNT:
        tick("mul_sq")
    return _wrap((a * a * b + (1 << (2 * FRAC_BITS - 1))) >> (2 * FRAC_BITS))


def div(a: int, b: int) -> int:
    if COUNT:
        tick("div")
    if b == 0:
        raise ZeroDivisionError("定点除零")
    return _wrap(_trunc_div(a << FRAC_BITS, b))


def sqrt(a: int) -> int:
    """定点开方。sqrt(v) 的原始值 = isqrt(原始值 << 16)。"""
    if COUNT:
        tick("sqrt")
    if a < 0:
        raise ValueError(f"定点开方收到负数：{a}")
    return _wrap(math.isqrt(a << FRAC_BITS))


def hypot(a: int, b: int) -> int:
    """二维模长，Q16.16 进 Q16.16 出。C 里是 `isqrt64((int64_t)a*a + (int64_t)b*b)`。

    板面摩擦的冲量要沿速度反向，所以每块每子步都要一次模长。用逐轴夹紧能省掉这次开方，但摩擦锥会变成方的：45° 方向的静摩擦阈值比轴向高 √2 倍，倾斜扫描一眼就能看出来。宁可开方。

    量程：vmax = 0.6，两个分量各不超过 39322，平方和上界 3.09e9。32 位有符号装不下，C 里用 uint64 中间量。
    """
    if COUNT:
        tick("hypot")
    return _wrap(math.isqrt(a * a + b * b))


def halve(a: int) -> int:
    """对称的除二，向零取整。C 里就是 `a / 2`。

    **不要用 `a >> 1`。** 算术右移是向下取整，奇数负数会差一；只要这个半量再参与有符号的比较或累加，左右镜像就不再等变。切向摩擦冲量正好是可正可负的。
    """
    return _trunc_div(a, 2)


def clamp(a: int, lo: int, hi: int) -> int:
    return lo if a < lo else (hi if a > hi else a)


def from_float(x: float) -> int:
    return _wrap(int(round(x * ONE)))


def to_float(a: int) -> float:
    return a / ONE


def cell(a: int) -> int:
    """取所在格号。位置一律非负，所以算术右移就是向下取整。"""
    return a >> FRAC_BITS
