"""生成固件 `v hydro` / `v hydrod` / `v grain` / `v graind` 的确定性 trace，用来逐位对照 C++ 静水压强与屈服门实现。

    python sim/water/port_trace_hydro.py hydro 240
    python sim/water/port_trace_hydro.py hydrod 240
    python sim/water/port_trace_hydro.py grain 240
    python sim/water/port_trace_hydro.py graind 240

四组场景的初态、参数、帧数完全一致，只在重力向量与屈服 μ 上不同：

- `hydro`：重力 (0, 79)，竖直向下。覆盖平放找平的对称简并分支（`tx = ty = 0` 时按板心方向破缺）、
  贴墙块的法向 `neighbor()` 返回自身水头、自由面块（`own = 0`），以及射线只跨 y 边界的插值。
- `hydrod`：重力 (40, 66)，约 31° 斜向。覆盖射线同时跨 x、y 边界时的 min-f 选择、侧向偏移对的
  对角选项，以及水头随方向连续变化。
- `grain`：与 `hydro` 同场景同重力，`yield_mu = 0.45`。竖直重力下平顶的坡度恰好为 0，屈服门会把
  有侧向支撑的块全部挡住（实测 240 帧里挡住 2927 次），自由竖列照推（129 次），`own = 0` 早退
  3824 次——三个分支都被压到。
- `graind`：与 `hydrod` 同场景同重力，`yield_mu = 0.45`。斜向重力下侧向偏移是对角对，
  `hypot(tx, ty)` 基本不为零，门真正起作用的阈值比较在这里被压得最重（挡住 3232 次）。
  μ 取 0.45 而不是 0.15，是因为 0.15 在斜向场景里几乎不触发（只挡 116 次），换成豆子预设才能
  把比较的两侧都走到。

参数刻意取零摩擦、零恢复、零内聚，只留静水压强：这样 trace 只压 `buildPressureDepth()` 与
`pressureAccel()` 两条路径，任何一处定点差异都会在 stateHash 上分叉。`grain` 与 `hydro` 的初态、
重力完全相同，两条 diff 可以单独隔离屈服门。

固件侧对应 `runTrace()` 的 hydro / hydrod / grain / graind 分支。对照前先归一化换行：两边都
`tr -d '\\r'` 再 diff。

注意：`v plain|friction|spin` 的 `hydrostatic` 是 0，`v water` 是零重力（`g∥ = 0` 时压强整项关闭），
所以在这四条 trace 之前，压强与屈服门都没有任何跨语言逐位覆盖。
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

# 四组场景的重力向量（Q16.16，引擎坐标，y 向下）与屈服 μ。固件 runTrace() 里是同一组数。
GRAVITY = {
    "hydro": (0, 79),
    "hydrod": (40, 66),
    "grain": (0, 79),
    "graind": (40, 66),
}
YIELD_MU = {
    "hydro": 0.0,
    "hydrod": 0.0,
    "grain": 0.45,
    "graind": 0.45,
}


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


def cells() -> list[tuple[int, int]]:
    """左下 4×3 贴墙块 + 中下 2 格 + 右侧 4 格竖列 + 右上 1 格，共 19 块。

    贴墙、悬空、单列、自由面四种情形都在里面：竖列有 `own > 0` 且两侧无料，自由面块 `own = 0`，
    x = 0 与 x = 7 的块会走到 `neighbor()` 的越界分支。添加顺序与固件逐项一致，stateHash 依赖块序。
    """
    return ([(x, y) for y in range(5, 8) for x in range(4)]
            + [(5, 6), (5, 7)]
            + [(6, y) for y in range(4, 8)]
            + [(7, 4)])


def build(kind: str) -> World2:
    p = Params(
        n=19,
        gravity=0.0,
        gravity_total=0.0,
        plate_friction=0.0,
        friction=0.0,
        restitution=0.0,
        restitution_slop=0.004,
        damping=0.9995,
        vmax=0.6,
        # 逐位基准钉在 2 子步 / 120 Hz，与固件 traceWorld（PhysicsWorld::Params 默认值）对齐。
        substeps=2,
        substep_hz=120.0,
        gate_order="pen",
        axis_order="vel",
        cohesion=0.0,
        cohesion_diag=0.5,
        cohesion_damping=0.0,
        hydrostatic=1.0,
        yield_mu=YIELD_MU[kind],
    )
    w = World2(p, cells=cells())
    w.set_gravity_q16(*GRAVITY[kind])
    return w


def main(argv: list[str]) -> int:
    kind = argv[0] if argv else "hydro"
    if kind not in GRAVITY:
        raise SystemExit(f"未知场景 {kind!r}，可选：{' / '.join(GRAVITY)}")
    frames = int(argv[1]) if len(argv) > 1 else 240
    gx, gy = GRAVITY[kind]
    w = build(kind)
    print(f"TRACE begin {kind} frames={frames} G=0 gx={gx} gy={gy} "
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
