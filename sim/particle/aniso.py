"""各向异性扫描。

否掉格子气的原始理由是方格点的各向异性调不掉。这版换掉了碰撞原语，但没有换掉**方格的轴**——门是先 x 后 y，接触面全轴对齐。所以这一条必须验。

做法有两层：

1. **原始曲线**：倾角 0° 到 90° 以 1° 步长扫过去，每个角度从同一初态跑到静止，记录堆料质心
2. **镜像检验**：世界是正方形，转置 `T(x, y) = (y, x)` 把世界映到自己，并且把倾角 θ 映成 90° − θ（`T(sinθ, cosθ) = (cosθ, sinθ)`）。如果动力学对 T 等变，那么用 `T(S)` 在 90° − θ 下跑出来的质心，必须等于用 `S` 在 θ 下跑出来的质心的转置。两者的距离就是各向异性的直接度量——它不用人眼去认台阶

**质心用格号算，不用连续相位。** 相位静止时被钉在格边上，拿它算质心会把钉边误读成位移（附录 C 的原话）。

用法：

    python sim/particle/aniso.py                          # μ = 0.55，0~90 度，1 度步长，轴序 vel
    python sim/particle/aniso.py --friction 0.3 0.55 0.8  # 摩擦对照：真实雪崩阈值会跟着 μ 走
    python sim/particle/aniso.py --axis-order xy          # 对照：固定先 x 后 y，残差会回来
    python sim/particle/aniso.py --step 5 --csv aniso.csv
    python sim/particle/aniso.py --table-every 0          # 只看摘要
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time

# 脚本住在 sim/particle/，公共模块（fixed / spawn）在上一层，先把它加进搜索路径。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import checker
import world2 as V2
from fixed import from_float


def use_utf8() -> None:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError):
            pass


def initial_cells() -> list[tuple[int, int]]:
    """4×4 的方块，水平居中靠顶。镜像后仍是合法摆位。"""
    return [(x, y) for y in range(4) for x in range(2, 6)]


def transpose(cs):
    return [(y, x) for (x, y) in cs]


def centroid(w) -> tuple[float, float]:
    n = len(w.blocks)
    return (sum(b.cx for b in w.blocks) / n, sum(b.cy for b in w.blocks) / n)


def settle(w, max_frames: int, need: int, v_eps: float) -> tuple[int, bool]:
    """静止 = 画面连续 need 帧不变，且所有速度低于 v_eps。

    不能要求速度严格为零：堆叠里上层物块每子步拿到的法向冲量只消掉 `(1+e)/2` 的相对速度，稳态残余约 `g/(1−e)`，位置被门钉住但速度不为零。也不能只看画面：刚释放的物块每子步只挪 0.0012 格，能在同一格里待几百帧。两个条件一起用才既不会把下落当成静止，也不会把堆叠的残余当成运动。
    """
    eps = from_float(v_eps)
    last = None
    quiet = 0
    for f in range(max_frames):
        w.frame()
        m = w.mask()
        slow = all(abs(b.vx) < eps and abs(b.vy) < eps for b in w.blocks)
        if m == last and slow:
            quiet += 1
            if quiet >= need:
                return f + 1, True
        else:
            quiet = 0
        last = m
    return max_frames, False


def run(tilt: float, cells, mu: float, max_frames: int, need: int, v_eps: float,
        axis_order: str) -> dict:
    w = V2.World2(V2.Params(n=len(cells), friction=mu, axis_order=axis_order), cells=cells)
    w.set_tilt(tilt)
    frames, ok = settle(w, max_frames, need, v_eps)
    cx, cy = centroid(w)
    return {
        "tilt": tilt,
        "cx": cx,
        "cy": cy,
        "frames": frames,
        "settled": ok,
        "bad": len(checker.audit(w)),
    }


def dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def scan(mu: float, angles: list[int], max_frames: int, need: int, v_eps: float,
         axis_order: str) -> dict:
    s0 = initial_cells()
    s1 = transpose(s0)
    a = {t: run(t, s0, mu, max_frames, need, v_eps, axis_order) for t in angles}
    need_t = sorted({90 - t for t in angles} | set(angles))
    b = {t: run(t, s1, mu, max_frames, need, v_eps, axis_order) for t in need_t}

    for t in angles:
        r = b[90 - t]
        mirrored = (r["cy"], r["cx"])
        a[t]["residual"] = dist((a[t]["cx"], a[t]["cy"]), mirrored)
    return a


def report(mu: float, axis_order: str, rows: dict, angles: list[int], table_every: int) -> int:
    vals = [rows[t] for t in angles]
    worst = max(vals, key=lambda r: r["residual"])
    mean_res = sum(r["residual"] for r in vals) / len(vals)
    zero = sum(1 for r in vals if r["residual"] < 1e-9)

    jumps = []
    for i in range(1, len(angles)):
        p, q = rows[angles[i - 1]], rows[angles[i]]
        jumps.append((dist((p["cx"], p["cy"]), (q["cx"], q["cy"])), angles[i - 1], angles[i]))
    jumps.sort(reverse=True)

    stuck = sum(1 for r in vals if not r["settled"])
    bad = sum(r["bad"] for r in vals)

    print(f"μ = {mu}   轴序 = {axis_order}")
    print(f"  镜像残差    最大 {worst['residual']:.4f} 格 @ {worst['tilt']}°   平均 {mean_res:.4f}   零残差 {zero}/{len(vals)}")
    print(f"  最大跳变    {jumps[0][0]:.3f} 格 @ {jumps[0][1]}°→{jumps[0][2]}°")
    for d, p, q in jumps[1:4]:
        print(f"              {d:.3f} 格 @ {p}°→{q}°")
    print(f"  未静止      {stuck}/{len(vals)}   不变量违反 {bad}")

    if table_every > 0:
        print(f"\n  {'角度':>5}{'质心 x':>9}{'质心 y':>9}{'残差':>9}{'帧数':>7}")
        for t in angles:
            if t % table_every and t not in (0, angles[-1]):
                continue
            r = rows[t]
            print(f"  {t:>5}{r['cx']:>9.2f}{r['cy']:>9.2f}{r['residual']:>9.4f}{r['frames']:>7}")
    print()

    # 返回最大跳变的起始角，供多 μ 对照使用
    return jumps[0][1]


def main(argv: list[str]) -> int:
    use_utf8()
    p = argparse.ArgumentParser(description="物块模式各向异性扫描")
    p.add_argument("--friction", type=float, nargs="+", default=[0.55],
                   help="库仑系数 μ，可以给多个做对照")
    p.add_argument("--step", type=int, default=1, help="倾角步长，度")
    p.add_argument("--max-frames", type=int, default=3000, help="每个角度的帧数上限")
    p.add_argument("--settle-need", type=int, default=30, help="画面连续多少帧不变且速度低于阈值算收敛")
    p.add_argument("--v-eps", type=float, default=0.02, help="判静止的速度阈值，格/子步")
    p.add_argument("--table-every", type=int, default=5, help="每隔几度打一行；0 只打摘要")
    p.add_argument("--axis-order", choices=("vel", "xy", "yx", "alt"), default="vel",
                   help="门先过哪个轴；vel = 按每块速度分量（默认），xy/yx = 固定，alt = 隔子步换")
    p.add_argument("--csv", metavar="FILE", help="逐度数据写成 CSV")
    a = p.parse_args(argv)

    angles = list(range(0, 91, a.step))
    t0 = time.time()
    avalanche = {}
    all_rows = []

    for mu in a.friction:
        rows = scan(mu, angles, a.max_frames, a.settle_need, a.v_eps, a.axis_order)
        avalanche[mu] = report(mu, a.axis_order, rows, angles, a.table_every)
        for t in angles:
            r = rows[t]
            all_rows.append((mu, a.axis_order, t, r["cx"], r["cy"], r["residual"], r["frames"], int(r["settled"]), r["bad"]))

    print("雪崩阈值（质心跳变最大的角度）随 μ 的移动")
    for mu in a.friction:
        print(f"  μ = {mu:<5} {avalanche[mu]:>3}°")
    if len(a.friction) > 1:
        moved = len(set(avalanche.values())) > 1
        print("  判定：" + ("阈值随 μ 移动，是真实的颗粒阈值行为" if moved else "阈值不随 μ 移动，更像量化假象"))

    if a.csv:
        with open(a.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["mu", "axis_order", "tilt", "cx", "cy", "mirror_residual", "frames", "settled", "violations"])
            w.writerows(all_rows)
        print(f"\nCSV 写到 {a.csv}")

    print(f"\n耗时 {time.time() - t0:.1f} 秒")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
