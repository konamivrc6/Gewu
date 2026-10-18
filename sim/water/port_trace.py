"""生成固件 `v water` 的确定性 trace，用来逐位对照 C++ 内聚实现。

    python sim/water/port_trace.py 240

场景与固件 runTrace() 的 water 分支一一对应：零重力、居中 4×4 的 16 格、零摩擦、零恢复，只开内聚力与键轴阻尼。每帧打印的 stateHash / mask / 动能 / audit / 接触计数必须与板上 `v water 240` 逐行相同。

对照时先归一化换行：串口是 CRLF，Python 在 Windows 上也是 CRLF，但 capture 工具可能一边剥掉、一边没剥。两边都 `tr -d '\\r'` 再 diff。
"""

from __future__ import annotations

import os
import sys

# 内核与定点原语在 sim/ 上一层。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import checker
import fixed
from world2 import Params, World2

FNV_OFFSET = 14695981039346656037
FNV_PRIME = 1099511628211


def state_hash(w: World2) -> int:
    h = FNV_OFFSET
    words = [len(w.blocks)]
    for b in w.blocks:
        words += [b.px, b.py, b.vx, b.vy, b.cx, b.cy]
    for v in words:
        h ^= v & 0xFFFFFFFF
        h = (h * FNV_PRIME) & 0xFFFFFFFFFFFFFFFF
    return h


def kinetic_raw(w: World2) -> int:
    s = 0
    for b in w.blocks:
        s += fixed.add(fixed.mul(b.vx, b.vx), fixed.mul(b.vy, b.vy))
    return s


def build() -> World2:
    p = Params(
        n=16,
        gravity=0.0,
        gravity_total=0.0,
        plate_friction=0.0,
        friction=0.0,
        restitution=0.0,
        restitution_slop=0.004,
        damping=0.9995,
        vmax=0.6,
        # 逐位基准钉在 2 子步 / 120 Hz，与固件 traceWorld（PhysicsWorld::Params 默认值）对齐；
        # 固件主循环和 sim/world2.py 的默认已经是 4 / 240，这里不跟随。
        substeps=2,
        substep_hz=120.0,
        gate_order="pen",
        axis_order="vel",
        cohesion=0.0010,
        cohesion_diag=0.5,
        cohesion_damping=0.05,
    )
    cells = [(x, y) for y in range(2, 6) for x in range(2, 6)]
    return World2(p, cells=cells)


def main(argv: list[str]) -> int:
    frames = int(argv[0]) if argv else 240
    w = build()
    print(f"TRACE begin water frames={frames} G=0 gx=0 gy=0 "
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
