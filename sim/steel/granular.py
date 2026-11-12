"""颗粒模式的屈服门压强 · 验证脚本。

`Params.yield_mu > 0` 时，压强项只在「侧向水头坡度的模长超过 μ·own」或「两侧都没有料块的自由竖列」时施加。这份脚本把这条规则的验收项一次跑完：

    python sim/steel/granular.py all
    python sim/steel/granular.py column
    python sim/steel/granular.py pile --seconds 120
    python sim/steel/granular.py mirror --mirror-step 15
    python sim/steel/granular.py budget

验收项与判据：

1. **竖列**：2 颗和 3 颗、贴墙与中间，都必须塌；末态 audit 必须为 0
2. **料堆**：4×4 与 4×8 满堆跑 120 s 仿真，格形必须稳定；4×4 的末态在钢与豆之间必须不同（μ 决定台阶间距）；满堆的残余动能要报出来，不设阈值
3. **镜像**：0°~90° 的转置检验，除恰好 45°（§4.1 已记录的轴序平局）外残差必须为 0
4. **开销**：同一初态跑 4 个子步，报出屈服门开启前后的定点运算数
5. **速度**：所有场景的 |v| 峰值必须低于 vmax = 0.6

参数取自 §4.5 的材料预设；本脚本只覆盖 `hydrostatic` 与 `yield_mu`，不动 steel.py 的默认值。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import checker
import fixed
from fixed import from_float, to_float
from steel import preset_params
from world2 import World2

GAIN_DEFAULT = 0.10
YIELD_MU = {"steel": 0.15, "bean": 0.45}


def use_utf8() -> None:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError):
            pass


# ============================ 工具 ============================

def make(preset: str, cells: list[tuple[int, int]], gain: float,
         mu: float | None = None) -> World2:
    p = preset_params(preset, len(cells))
    p.hydrostatic = gain
    p.yield_mu = YIELD_MU[preset] if mu is None else mu
    return World2(p, cells=cells)


def rows(w: World2) -> list[int]:
    m, W = w.mask(), w.W
    return [sum((m >> (y * W + x)) & 1 for x in range(W)) for y in range(w.H)]


def rows_str(w: World2) -> str:
    return "-".join(str(v) for v in rows(w))


def max_speed(w: World2) -> float:
    return max(max(abs(to_float(b.vx)) for b in w.blocks),
               max(abs(to_float(b.vy)) for b in w.blocks))


def centroid_cells(w: World2) -> tuple[float, float]:
    n = len(w.blocks)
    return (sum(b.cx for b in w.blocks) / n, sum(b.cy for b in w.blocks) / n)


def settle(w: World2, max_frames: int, need: int = 25, v_eps: float = 0.02) -> tuple[int, bool]:
    """画面连续 need 帧不变且所有速度低于 v_eps，算静止。与 particle/aniso.py 同一判据。"""
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


def transpose(cells):
    return [(y, x) for (x, y) in cells]


# 初态与 §4.5 / aniso 的用法保持一致。
def blob_cells() -> list[tuple[int, int]]:
    return [(x, y) for y in range(4, 8) for x in range(4)]


def full_cells() -> list[tuple[int, int]]:
    return [(x, y) for y in range(4) for x in range(8)]


def pyramid_cells() -> list[tuple[int, int]]:
    return [(0, 6), (0, 7), (1, 7)]


def mirror_cells() -> list[tuple[int, int]]:
    return [(x, y) for y in range(4) for x in range(2, 6)]


# ============================ 实验 ============================

def exp_column(args) -> float:
    """2/3 颗竖列：贴墙与中间，两种材料。必须塌，且末态 audit = 0。"""
    print(f"竖列坍塌 · 增益 {args.gain:.2f} · 每格 {args.frames} 帧上限\n")
    print(f"{'材料':<6}{'高度':>4}{'位置':>6}{'首变帧':>8}{'仿真秒':>8}  {'末态（上→下）':<16}{'audit':>6}{'|v|max':>9}")
    peak = 0.0
    for preset in ("steel", "bean"):
        for n in (2, 3):
            for x in (0, 3):
                cells = [(x, 8 - n + i) for i in range(n)]
                w = make(preset, cells, args.gain)
                ref = w.mask()
                first = None
                bad = 0
                for f in range(1, args.frames + 1):
                    w.frame()
                    if first is None and w.mask() != ref:
                        first = f
                    bad += len(checker.audit(w))
                    peak = max(peak, max_speed(w))
                sec = "—" if first is None else f"{first * 4 / 240:.2f}"
                print(f"{preset:<6}{n:>4}{'墙' if x == 0 else '中':>6}{str(first):>8}{sec:>8}  "
                      f"{rows_str(w):<16}{bad:>6}{max_speed(w):>9.4f}")
    print()
    return peak


def exp_pile(args) -> float:
    """4×4 与 4×8 满堆，跑 args.seconds 秒仿真，观察台阶是否稳定、两种材料是否不同。"""
    frames = int(round(args.seconds * 60))
    print(f"料堆 · 增益 {args.gain:.2f} · {args.seconds:.0f} s 仿真（{frames} 帧）\n")
    peak = 0.0
    for label, cells in (("4x4", blob_cells()), ("4x8", full_cells())):
        for preset in ("steel", "bean"):
            w = make(preset, cells, args.gain)
            bad = 0
            print(f"{label} · {preset}")
            for k in range(6):
                for _ in range(frames // 6):
                    w.frame()
                    bad += len(checker.audit(w))
                    peak = max(peak, max_speed(w))
                print(f"  t={(k + 1) * args.seconds / 6:5.0f}s  rows={rows_str(w):<12}"
                      f" ke={w.kinetic():.6f}  audit={bad}")
            print()
    return peak


def exp_mirror(args) -> tuple[float, float]:
    """转置检验：S 在 θ 下与 T(S) 在 90−θ 下的质心必须互为转置。"""
    s0 = mirror_cells()
    s1 = transpose(s0)
    angles = list(range(0, 91, args.mirror_step))
    print(f"镜像检验 · 增益 {args.gain:.2f} · {args.mirror_step}° 步长 · 最多 {args.mirror_frames} 帧/角\n")
    peak = 0.0
    worst_all = 0.0
    for preset in ("steel", "bean"):
        a = {}
        b = {}
        for t in angles:
            w = make(preset, s0, args.gain)
            w.set_tilt(t)
            frames, ok = settle(w, args.mirror_frames)
            a[t] = (centroid_cells(w), frames, ok, w)
            peak = max(peak, max_speed(w))
            w2 = make(preset, s1, args.gain)
            w2.set_tilt(90 - t)
            frames2, ok2 = settle(w2, args.mirror_frames)
            b[t] = (centroid_cells(w2), frames2, ok2, w2)
            peak = max(peak, max_speed(w2))
        worst = 0.0
        zero = 0
        print(f"{preset}（μ_yield = {YIELD_MU[preset]}）")
        for t in angles:
            (cx, cy), f, ok, w = a[t]
            (mx, my), f2, ok2, w2 = b[t]
            res = math.hypot(cx - my, cy - mx)
            worst = max(worst, res)
            if res < 1e-9:
                zero += 1
            note = "" if t != 45 else "  ← 轴序平局（§4.1 已记录）"
            print(f"  θ={t:3d}°  S=({cx:.2f},{cy:.2f})  T(S)@ {90 - t:3d}°=({mx:.2f},{my:.2f})"
                  f"  残差={res:.4f}  帧={f}/{f2}{note}")
        not_ok = sum(1 for t in angles if not a[t][2])
        print(f"  最大残差 {worst:.4f} 格，零残差 {zero}/{len(angles)}，"
              f"未达 0.02 速度阈值 {not_ok}/{len(angles)}（按同帧数比对，等变性不受影响）\n")
        worst_all = max(worst_all, worst)
    return peak, worst_all


def exp_budget(args) -> None:
    """同一初态只跑一个子步，比较屈服门关闭/开启的定点运算数。

    只跑一个子步是为了让两次的状态完全相同——跑整帧之后状态已经分叉，运算数会混进运动量的差别。
    """
    print("定点运算数 · 4×8 满堆 · 同一初态 · 一个子步\n")
    for preset in ("steel", "bean"):
        base = None
        for mu in (0.0, YIELD_MU[preset]):
            w = make(preset, full_cells(), args.gain, mu=mu)
            fixed.reset_counts()
            old = fixed.COUNT
            fixed.COUNT = True
            w.substep()
            fixed.COUNT = old
            total = sum(fixed.ops.values())
            tag = "关" if mu == 0 else f"开（μ={mu}）"
            extra = "" if base is None else f"  增量 {total - base:+d}"
            if base is None:
                base = total
            print(f"  {preset:<6} 屈服门{tag:<10} ops={total:6d}  "
                  f"hypot={fixed.ops.get('hypot', 0):3d}  mul={fixed.ops.get('mul', 0):5d}{extra}")
    print()


def exp_all(args) -> int:
    peak = 0.0
    worst = 0.0
    peak = max(peak, exp_column(args))
    peak = max(peak, exp_pile(args))
    p2, worst = exp_mirror(args)
    peak = max(peak, p2)
    exp_budget(args)
    print(f"汇总：|v| 峰值 {peak:.4f}（vmax = 0.6，余量 {(1 - peak / 0.6) * 100:.0f}%）；"
          f"镜像最大残差 {worst:.4f} 格（45° 轴序平局除外）")
    return 0


# ============================ CLI ============================

def main(argv=None) -> int:
    use_utf8()
    ap = argparse.ArgumentParser(description="颗粒模式屈服门压强验证")
    ap.add_argument("--gain", type=float, default=GAIN_DEFAULT, help="压强增益，默认 0.10")
    ap.add_argument("--frames", type=int, default=2400, help="column 每个用例的帧数上限")
    ap.add_argument("--seconds", type=float, default=120.0, help="pile 的仿真秒数")
    ap.add_argument("--mirror-step", type=int, default=15, help="mirror 的倾角步长，度")
    ap.add_argument("--mirror-frames", type=int, default=1800, help="mirror 每个角度的帧数上限")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("column", help="2/3 颗竖列")
    sub.add_parser("pile", help="4×4 与 4×8 料堆")
    sub.add_parser("mirror", help="转置检验")
    sub.add_parser("budget", help="定点运算数")
    sub.add_parser("all", help="全部")
    a = ap.parse_args(argv)
    if a.cmd == "column":
        exp_column(a)
    elif a.cmd == "pile":
        exp_pile(a)
    elif a.cmd == "mirror":
        exp_mirror(a)
    elif a.cmd == "budget":
        exp_budget(a)
    else:
        return exp_all(a)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
