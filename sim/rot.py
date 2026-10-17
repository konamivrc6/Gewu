"""转动模式：任意转轴的刚体转动。

平放的时候平面内没有重力，块本来是不动的。绕着屏幕法线转起来，格子坐标系跟着转，块就多出三个伪力（见 `world2.py` 的模块文档）：离心把块往外甩，科氏在块动起来之后把它往侧后方掰，欧拉在转速变化的那一段沿切向推。转速稳定、块也停住之后只剩离心，而离心在墙边有一个切向分量——贴在上墙的块被推向 x=4 以外的一侧，撞到角上。所以"平放、转板、块聚到四角"是这个模型的直接推论，不是额外加的规则。

转轴不限于屏幕法线，`--axis` 给体坐标里的任意方向。一般情形下三个伪力在平面内的分量见 `world2.py` 的模块文档：科氏与欧拉只有 z 分量进平面，离心三个分量都进。所以绕法线转（默认）把料甩向四角，绕平面内的轴转则把料推离那条轴——居中的一坨会分成两条。

三个伪力里离心是 Ω 的偶函数，科氏与欧拉是奇函数。把世界转置（x↔y），同时把 Ω 按赝矢量取反（`Ω' = (−Ωy, −Ωx, −Ωz)`），解应当仍然互为转置——这就是这里的镜像检验，和 `aniso.py` 的倾角镜像同一套路，但多了一个 Ω 的符号，正好用来抓科氏或欧拉写反。

另一个可测的推论：**关掉板面摩擦时，g = 0 下力的比值与转速无关。** 墙边的法向力正比于 ω²，切向驱动也正比于 ω²，块-块与块-墙的摩擦上限同样正比于 ω²，三者一起约掉。所以一个贴墙的块是滑向角还是原地粘住，只由 μ 与几何决定（粗略的滞留带是 |y − 4| < 3.5·μ），不由转多快决定。实测恒定转速下（`--plate-friction 0`）μ = 0.1 / 0.2 的终态角落数在 0.25~2 转/秒全是 12/16，μ = 0.4 时在 8~12 之间浮动——摩擦大了，堵塞的落点更依赖轨迹。

**板面摩擦打开时这条不成立，而且这正是它的用处。** 离心力随 ω² 长，板面摩擦上限 μ_p·N 是常数，于是存在临界转速

    ω_c = sqrt(μ_p·N / r)

低于它，块被静摩擦钉在原地；高于它才开始往外滑。平放、默认 G = 0.0012、μ_p = 0.35 时，r = 4 格处的临界转速约 0.19 转/秒——`--rps 0.1` 一块都不动，`--rps 0.5` 照常聚到四角。这比之前的低速量化底噪干净：那里是 ω²r 掉到 LSB 以下，这里是真实的静摩擦。

但**排列不严格不变**：阻尼和恢复系数的速度阈值都是按子步给的绝对量，不是尺度无关的，轨迹会不同；带上升速斜坡时欧拉项的瞬态还会改变块落在墙上的位置。所以这条只对"角落数"这种粗统计量成立，别拿它当逐位末态的判据。

**有重力时**倾角也要跟着镜像：`T(sinθ, cosθ) = (cosθ, sinθ)`，所以镜像那一侧跑 90° − θ。`--gravity` 非零时脚本自动这么做。

用法：

    python sim/rot.py                              # μ = 0.2，三档转速，恒定转速
    python sim/rot.py --rps 0.2 0.5 1.0 --mu 0.1 0.3 0.5
    python sim/rot.py --show --rps 0.5             # 画出末态
    python sim/rot.py --no-mirror                  # 跳过镜像检验
    python sim/rot.py --gravity 0.0012 --tilt 30   # 倾斜着转：重力与转动同时上
    python sim/rot.py --axis 1 0 0 --show          # 绕板面内的 x 轴转：料被推离转轴，分成两条
    python sim/rot.py --rps 0.1 0.2 0.3 0.5        # 看静摩擦的临界转速：0.2 以下一块都不动
    python sim/rot.py --plate-friction 0           # 关掉板面摩擦，退回到与转速无关的版本
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import unicodedata

import checker
import world2 as V2


def use_utf8() -> None:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError):
            pass


def dw(s: str) -> int:
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in s)


def pad(s: str, width: int) -> str:
    return s + " " * max(0, width - dw(s))


def center_patch() -> list[tuple[int, int]]:
    """居中的 4×4。四个角一共 16 格，正好装得下这 16 块。"""
    return [(x, y) for y in range(2, 6) for x in range(2, 6)]


def l_shape() -> list[tuple[int, int]]:
    """4 列 × 3 行的偏移方块。镜像检验要一个**转置后不一样**的初态，否则检验是平凡的。"""
    return [(x, y) for y in range(1, 4) for x in range(1, 5)]


def transpose(cs):
    return [(y, x) for (x, y) in cs]


def build(cells, mu: float, gravity: float, plate_mu: float = 0.35,
          g_total: float = 0.0012) -> V2.World2:
    p = V2.Params(n=len(cells), friction=mu, gravity=gravity,
                  plate_friction=plate_mu, gravity_total=g_total)
    return V2.World2(p, cells=cells)


def spin_run(cells, rps: float, mu: float, gravity: float, ramp: int, hold: int,
             pivot, tilt: float = 0.0, axis=(0.0, 0.0, 1.0),
             plate_mu: float = 0.35, g_total: float = 0.0012) -> V2.World2:
    """先把转速线性升到 rps，再保持 hold 帧。斜坡那一段 Ω̇ 不为零，欧拉项就在那里起作用。"""
    w = build(cells, mu, gravity, plate_mu, g_total)
    w.set_tilt(tilt)
    w.set_spin(0.0, 0.0, pivot=pivot, axis=axis)
    if ramp > 0:
        ramp_s = ramp / w.frame_hz
        for f in range(ramp):
            w.set_spin(rps * (f + 1) / ramp, rps / ramp_s, axis=axis)
            w.frame()
    w.set_spin(rps, 0.0, axis=axis)
    for _ in range(hold):
        w.frame()
    return w


def mirror_axis(axis):
    """赝矢量的镜像：转置是反射（det = −1），`Ω' = det(R)·RΩ = (−Ωy, −Ωx, −Ωz)`。"""
    return (-axis[1], -axis[0], -axis[2])


def centroid(w) -> tuple[float, float]:
    """格号质心。和 aniso.py 一样，不用相位——相位静止时被钉在格边上。"""
    n = len(w.blocks)
    return (sum(b.cx for b in w.blocks) / n, sum(b.cy for b in w.blocks) / n)


def in_corners(w) -> int:
    """落在四个 2×2 角里的块数。"""
    return sum(1 for b in w.blocks
               if (b.cx <= 1 or b.cx >= 6) and (b.cy <= 1 or b.cy >= 6))


def radii(w) -> tuple[float, float]:
    rs = [math.hypot(b.cx + 0.5 - 4.0, b.cy + 0.5 - 4.0) for b in w.blocks]
    return sum(rs) / len(rs), max(rs)


def mirror_residual(rps: float, mu: float, gravity: float, ramp: int, hold: int,
                    pivot, tilt: float = 0.0, axis=(0.0, 0.0, 1.0),
                    plate_mu: float = 0.35, g_total: float = 0.0012) -> tuple[float, int]:
    """转置 + Ω 按赝矢量取反。有重力时倾角也要映成 90° − θ：`T(sinθ, cosθ) = (cosθ, sinθ)`。"""
    a = spin_run(l_shape(), rps, mu, gravity, ramp, hold, pivot, tilt, axis, plate_mu, g_total)
    b = spin_run(transpose(l_shape()), rps, mu, gravity, ramp, hold, pivot, 90.0 - tilt,
                 mirror_axis(axis), plate_mu, g_total)
    ca, cb = centroid(a), centroid(b)
    res = math.hypot(ca[0] - cb[1], ca[1] - cb[0])
    return res, len(checker.audit(a)) + len(checker.audit(b))


def render(w) -> None:
    rows = w.render()
    print("  ┌" + "─" * len(rows[0]) + "┐")
    for row in rows:
        print("  │" + row + "│")
    print("  └" + "─" * len(rows[0]) + "┘")


def main(argv: list[str]) -> int:
    use_utf8()
    p = argparse.ArgumentParser(description="转动模式：离心聚集与镜像检验")
    p.add_argument("--rps", type=float, nargs="+", default=[0.25, 0.5, 1.0],
                   help="转速，转/秒")
    p.add_argument("--mu", type=float, nargs="+", default=[0.2], help="库仑系数 μ，可以给多个")
    p.add_argument("--gravity", type=float, default=0.0, help="平面内重力，格/子步²。平放为 0")
    p.add_argument("--tilt", type=float, default=0.0, help="重力倾角，度；只在 --gravity 非零时有意义")
    p.add_argument("--ramp", type=int, default=0, help="升速用多少帧")
    p.add_argument("--hold", type=int, default=1200, help="升速后保持多少帧")
    p.add_argument("--plate-friction", type=float, default=0.35,
                   help="板面库仑系数 μ_p。0 关闭；默认 0.35，平放时静摩擦会让低速转动一块都不动")
    p.add_argument("--gravity-total", type=float, default=0.0012,
                   help="总重力 G。板面法向 N = sqrt(G² − gravity²)")
    p.add_argument("--pivot", type=float, nargs=2, default=(4.0, 4.0),
                   help="加速度计的位置，格坐标；转动原点")
    p.add_argument("--axis", type=float, nargs=3, default=(0.0, 0.0, 1.0),
                   help="体坐标里的转轴，默认 (0,0,1) = 屏幕法线。给 (1,0,0) 就是把板子当桨叶翻")
    p.add_argument("--no-mirror", action="store_true", help="跳过镜像检验")
    p.add_argument("--show", action="store_true", help="画出每个转速的末态")
    a = p.parse_args(argv)
    pivot = (a.pivot[0], a.pivot[1])
    t0 = time.time()

    print(f"平面内重力 {a.gravity}  斜坡 {a.ramp} 帧  保持 {a.hold} 帧  转轴 {pivot}  轴 {tuple(a.axis)}")
    print(f"总重力 G {a.gravity_total}  板面 μ_p {a.plate_friction}")
    print(f"帧率按 {120.0 / 2:.0f} Hz（substep_hz = 120，2 子步）\n")

    for mu in a.mu:
        print(f"μ = {mu}")
        head = ["转速", "角落", "平均半径", "最大半径", "末帧点亮", "不变量", "镜像残差"]
        widths = [6, 7, 10, 10, 10, 8, 10]
        print("  " + "".join(pad(h, w) for h, w in zip(head, widths)))
        for rps in a.rps:
            w = spin_run(center_patch(), rps, mu, a.gravity, a.ramp, a.hold, pivot, a.tilt,
                         a.axis, a.plate_friction, a.gravity_total)
            bad = checker.audit(w)
            mean_r, max_r = radii(w)
            res = float("nan")
            mirror_bad = 0
            if not a.no_mirror:
                res, mirror_bad = mirror_residual(rps, mu, a.gravity, a.ramp, a.hold, pivot,
                                                  a.tilt, a.axis, a.plate_friction, a.gravity_total)
            cells_row = [
                f"{rps:.2f}", f"{in_corners(w)}/{len(w.blocks)}",
                f"{mean_r:.2f}", f"{max_r:.2f}",
                f"{w.lit_count()}/{len(w.blocks)}",
                str(len(bad) + mirror_bad),
                "—" if math.isnan(res) else f"{res:.4f}",
            ]
            print("  " + "".join(pad(c, wd) for c, wd in zip(cells_row, widths)))
            if bad:
                for line in bad[:5]:
                    print(f"    ! {line}")
            if a.show:
                render(w)
        print()

    print(f"耗时 {time.time() - t0:.1f} 秒")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
