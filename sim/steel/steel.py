"""钢珠模式的定点原型：参数预设、场景、度量与实验。

    python sim/steel/steel.py drop
    python sim/steel/steel.py --n 32 pile
    python sim/steel/steel.py plate
    python sim/steel/steel.py bounce
    python sim/steel/steel.py tilt
    python sim/steel/steel.py --n 24 show

全局选项（--frames / --n / --seed 等）写在子命令之前，与 sim/water/water.py 一致。


钢珠在本内核里仍是「一格一个方形块」，和水、豆子共用 `sim/world2.py` 的格子锚定内核。没有球体、没有多格刚体，也没有旋转自由度。所以这组参数不是几何模型，而是一组材料预设：板面摩擦取滚动阻力系数的量级，块间摩擦取干态钢-钢滑动的低端，恢复系数取钢-钢与钢-硬质墙的折中。方块不会滚，低 μ_p 就是在等效那个缺失的滚动自由度。

引擎坐标沿用 world2：x 向右、y 向下、原点左上。重力方向用 `set_tilt()` 旋转，`tilt=0` 时重力指向 +y（屏幕下方）。

尺度：1 格 = 4 mm，240 Hz 子步，4 子步一帧。G = 0.0118 格/子步² 对应约 1.90 倍慢放。密度不进入自由落体，所以钢珠和豆子共用同一个重力标度；「更重」只能靠低摩擦与高恢复表现。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

# 脚本住在 sim/steel/，内核与定点原语在上一层。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import checker
from fixed import ONE, from_float, to_float
from world2 import Params, World2

# 与固件 Gewu.ino 的 STEEL_* 常量一一对应。改这里时两边一起改。
GRAVITY_DEFAULT = 0.0118      # 与粒子和钢珠模式一致；水的 0.0120 是另一支
PLATE_FRICTION_DEFAULT = 0.003
PAIR_FRICTION_DEFAULT = 0.15
RESTITUTION_DEFAULT = 0.78
DAMPING_DEFAULT = 0.9999
SPIN_SCALE_NOTE = "固件里钢珠的转速增益是 100%，与粒子模式一致"

# 豆子预设只用于对照，数值抄自固件的 PARTICLE_* / PLATE_FRICTION_INIT / PAIR_FRICTION_INIT。
BEAN = dict(plate_friction=0.05, friction=0.45, restitution=0.50, damping=0.9998)


def steel_params(n: int, *,
                 gravity: float = GRAVITY_DEFAULT,
                 gravity_total: float | None = None,
                 plate_friction: float = PLATE_FRICTION_DEFAULT,
                 friction: float = PAIR_FRICTION_DEFAULT,
                 restitution: float = RESTITUTION_DEFAULT,
                 damping: float = DAMPING_DEFAULT,
                 substeps: int = 4,
                 substep_hz: float = 240.0) -> Params:
    """钢珠模式的一组参数。零内聚、零静水压强是定义的一部分，不开放覆盖。"""
    if gravity_total is None:
        gravity_total = max(gravity, GRAVITY_DEFAULT)
    return Params(
        n=n,
        gravity=gravity,
        gravity_total=gravity_total,
        plate_friction=plate_friction,
        friction=friction,
        restitution=restitution,
        restitution_slop=0.004,
        damping=damping,
        vmax=0.6,
        substeps=substeps,
        substep_hz=substep_hz,
        cohesion=0.0,
        cohesion_diag=0.5,
        cohesion_damping=0.0,
        hydrostatic=0.0,
    )


def bean_params(n: int, *, gravity: float = GRAVITY_DEFAULT,
                gravity_total: float | None = None,
                plate_friction: float = BEAN["plate_friction"],
                friction: float = BEAN["friction"],
                restitution: float = BEAN["restitution"],
                damping: float = BEAN["damping"],
                substeps: int = 4, substep_hz: float = 240.0) -> Params:
    return steel_params(n, gravity=gravity, gravity_total=gravity_total,
                        plate_friction=plate_friction, friction=friction,
                        restitution=restitution, damping=damping,
                        substeps=substeps, substep_hz=substep_hz)


def preset_params(name: str, n: int, **kw) -> Params:
    return bean_params(n, **kw) if name == "bean" else steel_params(n, **kw)


# ============================ 场景 ============================

def column_cells(n: int = 1, x: int = 4) -> list[tuple[int, int]]:
    return [(x, y) for y in range(n)]


def top_rows_cells(n: int, rows: int = 4, w: int = 8) -> list[tuple[int, int]]:
    """从顶行起逐行填满，最多 rows 行。钢珠从上方落下，用来观察落堆与反弹。"""
    cells = [(x, y) for y in range(rows) for x in range(w)]
    if n > len(cells):
        raise ValueError(f"n={n} 超过 {rows} 行 × {w} 列")
    return cells[:n]


def blob_cells(n: int, w: int = 8, h: int = 8) -> list[tuple[int, int]]:
    """左下角一块紧凑的料，留出上方与右侧的空腔。"""
    s = math.isqrt(n - 1) + 1
    x0, y0 = 0, h - s
    return [(x0 + i, y0 + j) for j in range(s) for i in range(s)][:n]


# ============================ 度量 ============================

def occupied(world: World2) -> set[tuple[int, int]]:
    m = world.mask()
    W = world.W
    return {(i % W, i // W) for i in range(world.W * world.H) if (m >> i) & 1}


def row_counts(world: World2) -> list[int]:
    m, W = world.mask(), world.W
    return [sum((m >> (y * W + x)) & 1 for x in range(W)) for y in range(world.H)]


def col_counts(world: World2) -> list[int]:
    m, W, H = world.mask(), world.W, world.H
    return [sum((m >> (y * W + x)) & 1 for y in range(H)) for x in range(W)]


def bbox(world: World2) -> tuple[int, int, int, int] | None:
    occ = occupied(world)
    if not occ:
        return None
    xs = [x for x, _ in occ]
    ys = [y for _, y in occ]
    return min(xs), min(ys), max(xs), max(ys)


def centroid(world: World2) -> tuple[float, float] | None:
    occ = occupied(world)
    if not occ:
        return None
    return (sum(x for x, _ in occ) / len(occ), sum(y for _, y in occ) / len(occ))


def render(world: World2) -> str:
    m = world.mask()
    W, H = world.W, world.H
    border = "+" + "-" * W + "+"
    rows = [border]
    for y in range(H):
        rows.append("|" + "".join("#" if (m >> (y * W + x)) & 1 else "." for x in range(W)) + "|")
    rows.append(border)
    return "\n".join(rows)


def line(world: World2, label: str = "") -> str:
    bb = bbox(world)
    cb = centroid(world)
    viol = checker.audit(world)
    audit = "ok" if not viol else f"{len(viol)}VIOL"
    return (f"{label:<10} n={world.lit_count():2d} rows={row_counts(world)} "
            f"bbox={bb} centroid=({cb[0]:.2f},{cb[1]:.2f}) "
            f"ke={world.kinetic():.6f} audit={audit}")


# ============================ 实验 ============================

def free_fall_height(world: World2, frames: int) -> tuple[list[int], int]:
    """单颗落体：返回每帧的 cy 序列与向下→向上的速度反号次数。"""
    seq: list[int] = []
    bounces = 0
    prev_vy = None
    for _ in range(frames):
        world.frame()
        b = world.blocks[0]
        seq.append(b.cy)
        vy = to_float(b.vy)
        if prev_vy is not None and prev_vy > 0 and vy < 0:
            bounces += 1
        prev_vy = vy
    return seq, bounces


def experiment_drop(args) -> None:
    """单颗落体：回弹高度必须符合 e²，钢珠应该比豆子弹得久。"""
    print(f"单颗落体：G={GRAVITY_DEFAULT}，{args.frames} 帧")
    for name in ("steel", "bean"):
        p = preset_params(name, 1)
        world = World2(p, seed=1, cells=column_cells(1))
        seq, bounces = free_fall_height(world, args.frames)
        # 第一次离开底行后的最高行（cy 最小），离底高度 = 7 - 最高行。
        first_up = next((i for i in range(1, len(seq)) if seq[i] < seq[i - 1]), None)
        top_row = min(seq[first_up:first_up + 60]) if first_up is not None else None
        print(f"  {name:<5} e={p.restitution:.2f}  反号={bounces:2d}  "
              f"首跳最高行={'-' if top_row is None else top_row}  "
              f"首跳回弹={'-' if top_row is None else 7 - top_row} 格  "
              f"理论 e²×7={p.restitution ** 2 * 7:.2f} 格")
        print(f"        cy: {''.join(str(v) for v in seq[:120])}")
    print()


def experiment_pile(args) -> None:
    """落堆：看最终堆形、残余动能与不变量。8×8 的盒子放不下真正的休止角，这一项只验稳定性。"""
    cells = top_rows_cells(args.n)
    for name in ("steel", "bean"):
        p = preset_params(name, len(cells))
        world = World2(p, seed=args.seed, cells=cells)
        print(f"落堆：{name}，n={len(cells)}，{args.frames} 帧")
        print(line(world, "t=0"))
        for _ in range(args.frames):
            world.frame()
        print(line(world, "end"))
        print(render(world))
        print()


def experiment_plate(args) -> None:
    """板面启滑阈值：单块、扫 g∥，G 固定，N 随 g∥ 下降。连续解是 g∥ = G·μ/√(1+μ²)。"""
    print("板面启滑：单块，扫 g∥ 到第一档会动。连续解 g∥ = G·μ/√(1+μ²)\n")
    print(f"{'preset':<6} {'μ_p':>6} {'连续解':>10} {'实测':>10} {'LSB':>5} {'角度':>8} {'N@阈值':>9}")
    for name in ("steel", "bean"):
        mu = preset_params(name, 1).plate_friction
        pred = GRAVITY_DEFAULT * mu / math.sqrt(1 + mu * mu)
        gmax = from_float(GRAVITY_DEFAULT)
        hit = None
        for k in range(1, gmax + 1):
            w = World2(preset_params(name, 1, gravity=k / ONE), cells=[(4, 4)])
            w.set_tilt(0.0)
            y0 = w.blocks[0].py
            for _ in range(240):
                w.frame()
            if w.blocks[0].py != y0:
                hit = k
                break
        if hit is None:
            print(f"{name:<6} {mu:6.3f} {pred:10.6f} {'未启滑':>10}")
            continue
        g_hit = hit / ONE
        n_thr = math.sqrt(max(0.0, GRAVITY_DEFAULT ** 2 - g_hit ** 2))
        print(f"{name:<6} {mu:6.3f} {pred:10.6f} {g_hit:10.6f} {hit:5d} "
              f"{math.degrees(math.atan(g_hit / n_thr)):7.3f}° {n_thr:9.6f}")
    print()


def experiment_bounce(args) -> None:
    """扫恢复系数：首跳回弹高度应约等于 e² 乘以落差。用来定 e 的上下限。"""
    print(f"扫恢复系数：单颗从顶行落下，{args.frames} 帧\n")
    print(f"{'e':>5} {'首跳最高行':>10} {'首跳回弹':>9} {'理论 e²×7':>10} {'反号次数':>8}")
    for e in (0.00, 0.30, 0.50, 0.60, 0.70, 0.78, 0.82, 0.90):
        p = steel_params(1, restitution=e)
        world = World2(p, seed=1, cells=column_cells(1))
        seq, bounces = free_fall_height(world, args.frames)
        first_up = next((i for i in range(1, len(seq)) if seq[i] < seq[i - 1]), None)
        if first_up is None:
            print(f"{e:5.2f} {'—':>10} {'—':>9} {e * e * 7:10.2f} {bounces:8d}")
            continue
        top = min(seq[first_up:first_up + 60])
        print(f"{e:5.2f} {top:10d} {7 - top:9d} {e * e * 7:10.2f} {bounces:8d}")
    print()


def experiment_tilt(args) -> None:
    """斜板流动：板面与水平成 φ 角，g∥ = G sinφ，N = G cosφ；再绕屏幕法线转 gravity 角。

    钢珠的 μ_p 只有 0.003，N 再小也几乎不抵抗流动；豆子的 0.05 会在低 φ 时把料钉住。看质心与 bbox 的差别。
    """
    phi = math.radians(args.board)
    g_par = GRAVITY_DEFAULT * math.sin(phi)
    cells = blob_cells(args.n)
    print(f"斜板流动：{args.n} 格左下紧凑块，板面 {args.board}°，g∥={g_par:.6f}，"
          f"N/G={math.cos(phi):.3f}，屏幕内重力角={args.angle}°，{args.frames} 帧")
    for name in ("steel", "bean"):
        p = preset_params(name, len(cells), gravity=g_par)
        world = World2(p, seed=args.seed, cells=cells)
        world.set_tilt(args.angle)
        print(f"--- {name}（μ_p={p.plate_friction:.3f}, μ={p.friction:.2f}, e={p.restitution:.2f}）---")
        print(line(world, "t=0"))
        for _ in range(args.frames):
            world.frame()
        print(line(world, "end"))
        print(render(world))
        print()


def experiment_show(args) -> None:
    """终端播放。"""
    cells = blob_cells(args.n)
    p = preset_params(args.preset, len(cells))
    world = World2(p, seed=args.seed, cells=cells)
    world.set_tilt(args.angle)
    for f in range(args.frames + 1):
        if f % args.every == 0:
            print(f"--- frame {f} ---")
            print(render(world))
            print(line(world))
        if f < args.frames:
            world.frame()


# ============================ CLI ============================

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="钢珠模式定点原型")
    ap.add_argument("--frames", type=int, default=600)
    ap.add_argument("--n", type=int, default=24)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--preset", choices=["steel", "bean"], default="steel",
                    help="只影响 show；drop / pile / plate / bounce / tilt 都会跑两边对照")
    ap.add_argument("--board", type=float, default=2.0,
                    help="tilt 用。板面与水平的夹角，度；g∥ = G sinφ，N = G cosφ。默认 2°：钢珠会流，豆子不动")
    ap.add_argument("--angle", type=float, default=30.0,
                    help="屏幕内重力方向，度；0 = 朝屏幕下方。默认 30° 让重力有横向分量，才看得出流动")
    ap.add_argument("--every", type=int, default=30)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("drop", help="单颗落体与回弹")
    sub.add_parser("pile", help="从顶部落堆")
    sub.add_parser("plate", help="板面启滑阈值")
    sub.add_parser("bounce", help="扫恢复系数")
    sub.add_parser("tilt", help="斜板流动，钢珠对豆子")
    sub.add_parser("show", help="终端播放")

    args = ap.parse_args(argv)
    if args.cmd == "drop":
        experiment_drop(args)
    elif args.cmd == "pile":
        experiment_pile(args)
    elif args.cmd == "plate":
        experiment_plate(args)
    elif args.cmd == "bounce":
        experiment_bounce(args)
    elif args.cmd == "tilt":
        experiment_tilt(args)
    elif args.cmd == "show":
        experiment_show(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
