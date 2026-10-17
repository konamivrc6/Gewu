"""生成与固件 `v` 命令同格式的确定性 trace。

用途：把 `firmware/Gewu/Gewu.ino` 里的 C++ 内核与 `sim/world2.py` 逐位对照。板上发 `v plain 240`、`v friction 240`、`v spin 600`，这里用同样的场景、同样的重力向量与同样的哈希取词顺序生成期望，逐行 diff。

    python sim/port_trace.py plain 240
    python sim/port_trace.py friction 240
    python sim/port_trace.py spin 600

三种场景与固件里的 runTrace() 一一对应：

- plain：默认参数，居中 4×4 的 16 块，G = 79、g∥ = 79，所以 N = 0、fdv = 0。这是“默认参数下板面摩擦不生效”的场景
- friction：同样的块，G = 79、g∥ = 33，N ≈ 72、fdv = 25，专门压 hypot / div / 板面摩擦
- spin：gravity = 0、plate_friction = 0、friction = 0.2，居中 4×4，omega = (0, 0, 3277)（0.05 弧度/子步），专门压 mul_sq / 科氏 / 欧拉

对照时不要用默认的 World2(Params())：默认参数下 from_float(0.0012) = 79，79/65536 比 0.0012 大，G² − g∥² < 0，原型初始化就把 gn/fdv 抹成 0。这里用 FixedG 子类固定 gvec，并按与 C++ 相同的公式重设 gn/fdv，才能和固件对齐。
"""

from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import checker
import fixed
import world2 as W

FNV_OFFSET = 14695981039346656037
FNV_PRIME = 1099511628211
ONE = fixed.ONE


class FixedG(W.World2):
    """gvec() 返回固定整数重力分量，绕开 set_tilt 的浮点三角。"""

    def __init__(self, *args, **kwargs):
        self._fixed_g = kwargs.pop("fixed_g", (0, 0))
        super().__init__(*args, **kwargs)

    def gvec(self):
        return self._fixed_g


def center_cells() -> list[tuple[int, int]]:
    """与固件 runTrace() 的 for y 2..5 / for x 2..5 行优先顺序完全一致。"""
    return [(x, y) for y in range(2, 6) for x in range(2, 6)]


def build(kind: str):
    if kind == "spin":
        p = W.Params(n=16, gravity=0.0, plate_friction=0.0, friction=0.2)
        g = (0, 0)
    elif kind == "plain":
        p = W.Params(n=16)
        g = (0, 79)
    elif kind == "friction":
        p = W.Params(n=16)
        g = (0, 33)
    else:
        raise ValueError(f"未知场景 {kind!r}")

    w = FixedG(p, cells=center_cells(), fixed_g=g)

    # 与 C++ 的 setGravityTotalQ16(79) + setGravityVectorQ16() 对齐。C++ 那边 G_ = 79 / 65536，不是 Params 里的 0.0012。
    G = 79 / ONE
    gx = g[0] / ONE
    gy = g[1] / ONE
    n = math.sqrt(max(0.0, G * G - gx * gx - gy * gy))
    w.gn = fixed.from_float(n)
    w.fdv = fixed.mul(w.mup, w.gn)

    if kind == "spin":
        # 直接赋定点 omega，绕开 set_spin() 的 2π/浮点换算。
        w.omega = (0, 0, 3277)
        w.omega_dot = (0, 0, 0)
        w.pivx = 4 * ONE
        w.pivy = 4 * ONE

    return w, g


def state_hash(w) -> int:
    h = FNV_OFFSET
    words = [len(w.blocks)]
    for b in w.blocks:
        words += [b.px, b.py, b.vx, b.vy, b.cx, b.cy]
    for v in words:
        h ^= v & 0xFFFFFFFF
        h = (h * FNV_PRIME) & 0xFFFFFFFFFFFFFFFF
    return h


def kinetic_raw(w) -> int:
    s = 0
    for b in w.blocks:
        s += fixed.add(fixed.mul(b.vx, b.vx), fixed.mul(b.vy, b.vy))
    return s


def main(argv: list[str]) -> int:
    if len(argv) < 1:
        print(__doc__)
        return 2
    kind = argv[0]
    frames = int(argv[1]) if len(argv) > 1 else (600 if kind == "spin" else 240)

    w, g = build(kind)
    Gq = fixed.from_float(w.p.gravity_total)
    print(f"TRACE begin {kind} frames={frames} G={Gq} gx={g[0]} gy={g[1]} "
          f"gn={w.gn} fdv={w.fdv} mup={w.mup} mu={w.mu}")

    for f in range(frames):
        w.frame()
        bad = 1 if checker.audit(w) else 0
        st = w.stats
        print(f"TRACE f={f} h={state_hash(w):016X} m={w.mask():016X} n={len(w.blocks)} "
              f"k={kinetic_raw(w)} a={bad} c={st['commits']} b={st['blocked']} w={st['wall']}")

    print("TRACE END")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
