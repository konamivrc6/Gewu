"""水模式的定点原型：参数预设、场景、度量与实验。

    python sim/water/water.py drop
    python sim/water/water.py wet
    python sim/water/water.py tilt
    python sim/water/water.py sweep
    python sim/water/water.py show --frames 600 --every 30

模型是 `sim/world2.py` 的格子锚定内核加四项：八邻域内聚、沿键轴的相对速度阻尼、壁面虚拟水格（浸润），以及静水压强。板面摩擦与块间摩擦都置零，恢复系数置零。一格一颗保证不可压缩；内聚力只在边界产生净力，体内没有剪切刚度；静水压强补上内聚力缺的那一环——竖直方向上的重量如何变成横向推力，让竖柱会塌、液面会平。

引擎坐标沿用 world2：x 向右、y 向下、原点左上。重力方向用 `set_tilt()` 的倾角旋转，`tilt=0` 时重力指向 +y（屏幕下方）。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

# 库住在 sim/water/，内核与定点原语在上一层。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import checker
from world2 import Params, World2

GRAVITY_DEFAULT = 0.0120      # 与固件水模式的 GRAVITY_PER_G_WATER 一致（粒子模式的 3 倍），格/子步²
COHESION_DEFAULT = 0.0010     # 约为水模式重力的十二分之一
DIAG_DEFAULT = 0.5            # 对角键相对轴向的权重
HYDROSTATIC_DEFAULT = 1.0     # 静水压强增益：横向加速度 = gain · g∥ · 两侧水头差


# ============================ 参数与场景 ============================

def water_params(n: int, *, gravity: float = GRAVITY_DEFAULT,
                 gravity_total: float | None = None,
                 cohesion: float = COHESION_DEFAULT,
                 cohesion_diag: float = DIAG_DEFAULT,
                 cohesion_damping: float = 0.05,
                 hydrostatic: float = HYDROSTATIC_DEFAULT,
                 wetting: float = 0.0,
                 wetting_gain: float = 1.0,
                 wall_adhesion: float = 0.0,
                 damping: float = 0.9995,
                 substeps: int = 4,
                 substep_hz: float = 240.0) -> Params:
    """水模式的一组参数。零摩擦、零恢复系数是定义的一部分，不开放覆盖。"""
    if gravity_total is None:
        gravity_total = gravity
    return Params(
        n=n,
        gravity=gravity,
        gravity_total=gravity_total,
        plate_friction=0.0,
        friction=0.0,
        restitution=0.0,
        restitution_slop=0.004,
        damping=damping,
        vmax=0.6,
        substeps=substeps,
        substep_hz=substep_hz,
        cohesion=cohesion,
        cohesion_diag=cohesion_diag,
        cohesion_damping=cohesion_damping,
        hydrostatic=hydrostatic,
        wetting=wetting,
        wetting_gain=wetting_gain,
        wall_adhesion=wall_adhesion,
    )


def ring_cells(size: int = 5, w: int = 8, h: int = 8) -> list[tuple[int, int]]:
    """居中的空心方环。5×5 的环正好 16 格，零重力下应该被内聚力收成一团。"""
    x0, y0 = (w - size) // 2, (h - size) // 2
    return [(x0 + i, y0 + j) for j in range(size) for i in range(size)
            if i in (0, size - 1) or j in (0, size - 1)]


def block_cells(x0: int, y0: int, bw: int, bh: int) -> list[tuple[int, int]]:
    return [(x0 + i, y0 + j) for j in range(bh) for i in range(bw)]


def bottom_cells(n: int, w: int = 8, h: int = 8) -> list[tuple[int, int]]:
    """从底行起逐行填满，用来摆一池水。整行铺满会让液体无路可流，只适合浸润这类静态实验。"""
    cs = []
    for y in range(h - 1, -1, -1):
        for x in range(w):
            cs.append((x, y))
            if len(cs) == n:
                return cs
    raise ValueError(f"n={n} 超过棋盘 {w}×{h}")


def blob_cells(n: int, w: int = 8, h: int = 8) -> list[tuple[int, int]]:
    """左下角一块紧凑的水，留出上方与右侧的空腔。流动演示必须用它，不能铺满整行。"""
    s = math.isqrt(n - 1) + 1
    x0, y0 = 0, h - s
    return [(x0 + i, y0 + j) for j in range(s) for i in range(s)][:n]


# ============================ 度量 ============================

def occupied(world: World2) -> set[tuple[int, int]]:
    m = world.mask()
    W = world.W
    return {(i % W, i // W) for i in range(world.W * world.H) if (m >> i) & 1}


def components(world: World2) -> tuple[int, int]:
    """按四邻域数连通分量，返回 (分量数, 最大分量)。"""
    occ = occupied(world)
    seen: set[tuple[int, int]] = set()
    sizes = []
    for start in occ:
        if start in seen:
            continue
        stack = [start]
        seen.add(start)
        size = 0
        while stack:
            x, y = stack.pop()
            size += 1
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (x + dx, y + dy)
                if nb in occ and nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        sizes.append(size)
    return len(sizes), (max(sizes) if sizes else 0)


def perimeter(world: World2) -> int:
    """自由表面长度：只数界内空格方向，贴墙不算液-气界面。"""
    occ = occupied(world)
    p = 0
    for (x, y) in occ:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < world.W and 0 <= ny < world.H and (nx, ny) not in occ:
                p += 1
    return p


def centroid(world: World2) -> tuple[float, float] | None:
    occ = occupied(world)
    if not occ:
        return None
    return (sum(x for x, _ in occ) / len(occ), sum(y for _, y in occ) / len(occ))


def bbox(world: World2) -> tuple[int, int, int, int] | None:
    occ = occupied(world)
    if not occ:
        return None
    xs = [x for x, _ in occ]
    ys = [y for _, y in occ]
    return min(xs), min(ys), max(xs), max(ys)


def column_tops(world: World2) -> list[int | None]:
    """每列最上面的水格行号。行号越小越高；空列给 None。"""
    occ = occupied(world)
    tops: list[int | None] = []
    for x in range(world.W):
        ys = [y for (xx, y) in occ if xx == x]
        tops.append(min(ys) if ys else None)
    return tops


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
    ncomp, biggest = components(world)
    cb = centroid(world)
    bb = bbox(world)
    viol = checker.audit(world)
    audit = "ok" if not viol else f"{len(viol)}VIOL"
    return (f"{label:<8} n={world.lit_count():2d} comp={ncomp} big={biggest:2d} "
            f"perim={perimeter(world):2d} bbox={bb} centroid=({cb[0]:.2f},{cb[1]:.2f}) "
            f"audit={audit}")


# ============================ 实验 ============================

def experiment_drop(args) -> None:
    """零重力下空心方环收成一团。这一项只看内聚力本身。"""
    cells = ring_cells(5)
    p = water_params(len(cells), gravity=0.0, cohesion=args.cohesion,
                     cohesion_diag=args.diag, cohesion_damping=args.damping,
                     hydrostatic=args.press)
    world = World2(p, cells=cells)
    print("零重力收缩：5×5 空心环，16 格。目标是一个连通、周长小的液滴")
    print(line(world, "t=0"))
    for f in range(1, args.frames + 1):
        world.frame()
        if f % args.report == 0 or f == args.frames:
            print(line(world, f"f={f}"))
    print()
    print(render(world))


def experiment_wet(args) -> None:
    """一池水贴底，扫浸润系数，看壁面处的液面高度。"""
    cells = bottom_cells(args.n)
    print(f"浸润：{args.n} 格水贴底，重力朝下，扫 wetting，gain={args.line}")
    print("注：gain≈1 是物理量级，但 g=0.004 时一格高的液面爬升需要更大的线力；8~12 才看得见，且是风格化参数。")
    for wet in (-0.5, 0.0, 0.3, 0.6, 0.8, 0.95):
        p = water_params(len(cells), gravity=args.gravity, wetting=wet,
                         wetting_gain=args.line, wall_adhesion=args.adhesion,
                         cohesion=args.cohesion, cohesion_diag=args.diag,
                         cohesion_damping=args.damping, hydrostatic=args.press)
        world = World2(p, cells=cells)
        for _ in range(args.frames):
            world.frame()
        tops = column_tops(world)
        left, right = tops[0], tops[-1]
        mid = [t for t in tops[2:6] if t is not None]
        center = sum(mid) / len(mid) if mid else None
        rise = None if (left is None or center is None) else center - left
        print(f"  wet={wet:+.2f}  tops={tops}  comp={components(world)}  "
              f"perim={perimeter(world):2d}  wall_rise={'-' if rise is None else f'{rise:+.2f}'}")
        if args.show:
            print(render(world))
            print()


def experiment_tilt(args) -> None:
    """倾斜板上水往低处流。零摩擦时应该很快铺到低角。"""
    cells = blob_cells(args.n)
    print(f"倾斜流动：{args.n} 格水（左下角紧凑块），tilt={args.tilt}°，重力 {args.gravity}。看质心 x 与液面跨度")
    for coh in (0.0, 0.0005, 0.0010, 0.0020, 0.0040, 0.0080):
        p = water_params(len(cells), gravity=args.gravity, cohesion=coh,
                         cohesion_diag=args.diag, cohesion_damping=args.damping,
                         hydrostatic=args.press)
        world = World2(p, cells=cells)
        world.set_tilt(args.tilt)
        print(line(world, "t=0"))
        for _ in range(args.frames):
            world.frame()
        print(line(world, f"coh={coh:g}"))
        if args.show:
            print(render(world))
        print()


def experiment_column(args) -> None:
    """竖柱塌落：整列水在重力里必须塌成一滩，而不是原地站着。

    这是静水压强的回归实验。没有这一项时，一根 8 格高的竖柱是完全平衡的：每个粒子都被下面一格挡住，左右两个方向都没有力，内聚力在两侧自由面上又正好对消，柱子会一直立着。真实液体里，柱子底部的压强把水横向挤出去，所以它一定会倒。
    同时跑三个对照：贴左墙的竖柱、板心的竖柱、已经铺平的底行。前两个要塌，第三个必须一动不动。
    """
    cases = [
        ("左墙竖柱", [(0, y) for y in range(8)], 0.0),
        ("板心竖柱", [(4, y) for y in range(8)], 0.0),
        ("底行铺平", [(x, 7) for x in range(8)], 0.0),
        ("倾斜 45° 对角柱", [(i, i) for i in range(8)], 45.0),
    ]
    print(f"竖柱塌落：g={args.gravity}  press={args.press}  frames={args.frames}")
    for label, cells, tilt in cases:
        p = water_params(len(cells), gravity=args.gravity, cohesion=args.cohesion,
                         cohesion_diag=args.diag, cohesion_damping=args.damping,
                         hydrostatic=args.press)
        world = World2(p, cells=cells)
        world.set_tilt(tilt)
        print()
        print(f"--- {label}（tilt={tilt:g}°）---")
        print(line(world, "t=0"))
        for f in range(1, args.frames + 1):
            world.frame()
            if f % args.report == 0 or f == args.frames:
                print(line(world, f"f={f}"))
        print(render(world))


def experiment_sweep(args) -> None:
    """扫 a_c/g：散开 / 液滴 / 刚性团的分界。"""
    cells = ring_cells(5)
    g = args.gravity
    print(f"扫 a_c/g：零重力环收缩 + 倾斜 {args.tilt}° 流动，g={g}")
    print(f"{'a_c/g':>7} {'coh':>9} {'收缩后 comp':>12} {'perim':>6} {'bbox':>14} "
          f"{'倾斜质心x':>10} {'液面跨度':>10}")
    for ratio in (0.0, 0.0625, 0.125, 0.25, 0.5, 1.0, 2.0, 4.0):
        coh = ratio * g
        p = water_params(len(cells), gravity=0.0, cohesion=coh,
                         cohesion_diag=args.diag, cohesion_damping=args.damping,
                         hydrostatic=args.press)
        world = World2(p, cells=cells)
        for _ in range(args.frames):
            world.frame()
        ncomp, biggest = components(world)
        bb = bbox(world)
        n = world.lit_count()

        p2 = water_params(len(cells), gravity=g, cohesion=coh,
                          cohesion_diag=args.diag, cohesion_damping=args.damping,
                          hydrostatic=args.press)
        w2 = World2(p2, cells=cells)
        w2.set_tilt(args.tilt)
        for _ in range(args.frames):
            w2.frame()
        c2 = centroid(w2)
        tops = [t for t in column_tops(w2) if t is not None]
        span = (max(tops) - min(tops)) if tops else 0
        print(f"{ratio:7.4g} {coh:9.5f} {ncomp:>5}/{biggest:<6} {perimeter(world):6d} "
              f"{str(bb):>14} {c2[0]:10.2f} {span:10d}   n={n}")


def experiment_jam(args) -> None:
    """整行铺满的水在斜面上流不动：没有空位，液体被格子锁死。这是“水量上限”的直接演示。"""
    cells = bottom_cells(args.n)
    p = water_params(len(cells), gravity=args.gravity, cohesion=args.cohesion,
                     cohesion_diag=args.diag, cohesion_damping=args.damping,
                     hydrostatic=args.press)
    world = World2(p, cells=cells)
    world.set_tilt(args.tilt)
    print(f"堵塞测试：{args.n} 格水铺满底部整行，tilt={args.tilt}°。零摩擦也流不动")
    print(line(world, "t=0"))
    for _ in range(args.frames):
        world.frame()
    print(line(world, "end"))
    print(render(world))
    print("对照：同样水量改用左下角紧凑块（blob_cells），会流成楼梯状液面。见 tilt 子命令。")


def experiment_show(args) -> None:
    """终端播放。"""
    cells = blob_cells(args.n)
    p = water_params(len(cells), gravity=args.gravity, cohesion=args.cohesion,
                     cohesion_diag=args.diag, cohesion_damping=args.damping,
                     hydrostatic=args.press, wetting=args.wetting,
                     wetting_gain=args.line, wall_adhesion=args.adhesion)
    world = World2(p, cells=cells)
    world.set_tilt(args.tilt)
    if args.ring:
        world = World2(water_params(len(ring_cells(5)), gravity=0.0,
                                    cohesion=args.cohesion, cohesion_diag=args.diag,
                                    cohesion_damping=args.damping,
                                    hydrostatic=args.press),
                       cells=ring_cells(5))
    for f in range(args.frames + 1):
        if f % args.every == 0:
            print(f"--- frame {f} ---")
            print(render(world))
            print(line(world))
        if f < args.frames:
            world.frame()


# ============================ CLI ============================

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="水模式定点原型")
    ap.add_argument("--frames", type=int, default=1200)
    ap.add_argument("--report", type=int, default=200)
    ap.add_argument("--n", type=int, default=24)
    ap.add_argument("--gravity", type=float, default=GRAVITY_DEFAULT)
    ap.add_argument("--cohesion", type=float, default=COHESION_DEFAULT)
    ap.add_argument("--diag", type=float, default=DIAG_DEFAULT)
    ap.add_argument("--visc", "--damping", type=float, default=0.05, dest="damping",
                    help="键轴阻尼（黏性），0..1")
    ap.add_argument("--press", type=float, default=HYDROSTATIC_DEFAULT,
                    help="静水压强增益，0 = 关闭")
    ap.add_argument("--wetting", type=float, default=0.6)
    ap.add_argument("--line", type=float, default=1.0, help="接触线力增益")
    ap.add_argument("--adhesion", type=float, default=0.0, help="壁面虚拟格的法向吸附强度")
    ap.add_argument("--tilt", type=float, default=20.0)
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--ring", action="store_true", help="show 用零重力空心环")
    ap.add_argument("--every", type=int, default=30)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("drop", help="零重力环收缩")
    sub.add_parser("wet", help="浸润系数扫接触面")
    sub.add_parser("tilt", help="倾斜流动")
    sub.add_parser("sweep", help="扫 a_c/g")
    sub.add_parser("jam", help="整行铺满时的堵塞")
    sub.add_parser("column", help="竖柱塌落：静水压强的回归实验")
    sub.add_parser("show", help="终端播放")

    args = ap.parse_args(argv)
    if args.cmd == "drop":
        experiment_drop(args)
    elif args.cmd == "wet":
        experiment_wet(args)
    elif args.cmd == "tilt":
        experiment_tilt(args)
    elif args.cmd == "sweep":
        experiment_sweep(args)
    elif args.cmd == "jam":
        experiment_jam(args)
    elif args.cmd == "column":
        experiment_column(args)
    elif args.cmd == "show":
        experiment_show(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
