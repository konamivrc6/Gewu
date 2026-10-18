"""统一评测脚本。

    python sim/particle/eval.py invariant --engine v2     # 不变量与点亮数（判定性）
    python sim/particle/eval.py invariant --engine v1     # 同样的套件跑基线，应当看到非零
    python sim/particle/eval.py invariant --quick         # 四分之一帧数，改代码时快速自检
    python sim/particle/eval.py kinetic --engine both     # 动能守恒残差（诊断性）
    python sim/particle/eval.py ops --engine v2           # 每帧定点运算数（给 C3 对表）
    python sim/particle/eval.py plate                     # 板面静摩擦的启滑阈值，逐 LSB 扫

**判定性指标**

1. 逐帧点亮数正确率。判定标准只有一条：是否为 100.000%。任何非 100% 就是弱保证，不接受"很少"
2. 不变量累计违反数。用 `checker.audit()`，独立于引擎自己的计数器

**诊断指标**

3. 动能守恒残差。无关重力、阻尼置 1，从同一初态跑；剩下的能量损失只可能来自碰撞与非牛顿操作。它直接量化"修补层吃掉了多少能量"，是能戳穿"廉价地强"那类方案的指标
4. 每帧定点运算数。打开 `fixed.COUNT` 跑一帧。Python 里的耗时没有意义，只有计数能搬到 C3 上对表

各向异性扫描在 `sim/particle/aniso.py`，转动与转置镜像在 `sim/particle/rot.py`；滞后回线还没实现——它要的是手感，不是不变量。
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
import unicodedata

# 脚本住在 sim/particle/，公共模块（fixed / spawn）在上一层，先把它加进搜索路径。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import checker
import fixed
import world as V1
import world2 as V2


def use_utf8() -> None:
    """与 run.py 同因：Windows 控制台接管道时退回 GBK，中文和表格线会糊掉。"""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError):
            pass


def dw(s: str) -> int:
    """显示宽度。中文算两列，否则 `:<38` 对不齐。"""
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in s)


def pad(s: str, width: int, right: bool = False) -> str:
    fill = " " * max(0, width - dw(s))
    return fill + s if right else s + fill


def build(engine: str, kw: dict):
    kw = dict(kw)
    seed = kw.pop("seed", 1)
    scene = kw.pop("scene", "top")
    if engine == "v2":
        return V2.World2(V2.Params(**kw), seed=seed, scene=scene)
    return V1.World(V1.Params(**kw), seed=seed, scene=scene)


# ---------- 倾角轨迹 ----------

def const(deg: float):
    return lambda _f: deg


def walk(seed: int, step: float = 3.0):
    """连续随机游走。状态长在闭包里，每套用例单独 new 一个。"""
    rng = random.Random(seed)
    deg = 0.0

    def f(_frame: int) -> float:
        nonlocal deg
        deg = max(-180.0, min(180.0, deg + rng.uniform(-step, step)))
        return deg

    return f


def sweep(period: float = 60.0, amp: float = 180.0):
    return lambda f: amp * math.sin(2 * math.pi * f / period)


# ---------- 套件 ----------

def cases(quick: bool):
    scale = 0.25 if quick else 1.0

    def fr(x: int) -> int:
        return max(1, int(x * scale))

    out = []

    out.append((
        "原失败配置 tilt 80 / n 6 / seed 5",
        [(dict(n=6, seed=5, scene="top"), fr(400), const(80))],
    ))

    seeds = range(1, 21) if not quick else range(1, 3)
    sub = []
    for s in seeds:
        for t in (0, 30, 60, 75, 80, 90, 135):
            sub.append((dict(n=6, seed=s, scene="top"), fr(800), const(t)))
    out.append((f"{len(seeds)} seed × 7 倾角 × {fr(800)} 帧", sub))

    sub = []
    for n in (1, 2, 4, 8, 16, 32, 48, 64):
        sub.append((dict(n=n, seed=n + 100, scene="full"), fr(600), walk(n + 100)))
    out.append((f"n = 1/2/4/8/16/32/48/64，随机倾角 × {fr(600)} 帧", sub))

    sub = []
    for n in (6, 24, 64):
        sub.append((dict(n=n, seed=n + 200, scene="full"), fr(900), sweep()))
    out.append((f"n = 6/24/64，快速正弦扫掠 × {fr(900)} 帧", sub))

    return out


def run_job(engine: str, kw: dict, frames: int, tilt_fn) -> dict:
    w = build(engine, kw)
    bad = 0
    lit_bad = 0
    first = None
    for f in range(frames):
        w.set_tilt(tilt_fn(f))
        w.frame()
        v = checker.audit(w)
        if v:
            bad += 1
            if first is None:
                first = (f, v[:3])
        if bin(w.mask()).count("1") != w.p.n:
            lit_bad += 1
    return dict(frames=frames, bad=bad, lit_bad=lit_bad, first=first)


def cmd_invariant(a) -> int:
    engines = ["v1", "v2"] if a.engine == "both" else [a.engine]
    for engine in engines:
        if engine == "v1" and not a.quick:
            print("提示：v1 是 O(N²) 加四遍修补，全套件会很慢，通常用 --quick 看基线。",
                  file=sys.stderr)
        print(f"引擎 {engine}\n")
        print(pad("用例", 38) + pad("帧数", 8, True) + pad("违反帧", 9, True) + pad("点亮错误", 10, True))
        print("-" * 66)
        total = dict(frames=0, bad=0, lit_bad=0)
        failures = []
        for label, subs in cases(a.quick):
            acc = dict(frames=0, bad=0, lit_bad=0)
            for kw, frames, tilt_fn in subs:
                r = run_job(engine, kw, frames, tilt_fn)
                for k in acc:
                    acc[k] += r[k]
                if r["first"]:
                    failures.append((label, kw, r["first"]))
            for k in total:
                total[k] += acc[k]
            print(pad(label, 38) + pad(str(acc["frames"]), 8, True)
                  + pad(str(acc["bad"]), 9, True) + pad(str(acc["lit_bad"]), 10, True))
        print("-" * 66)
        print(pad("合计", 38) + pad(str(total["frames"]), 8, True)
              + pad(str(total["bad"]), 9, True) + pad(str(total["lit_bad"]), 10, True))
        ok = total["bad"] == 0 and total["lit_bad"] == 0
        rate = 100.0 * (total["frames"] - total["lit_bad"]) / max(1, total["frames"])
        print(f"\n点亮数正确率 {rate:.3f}%   不变量违反 {total['bad']} 帧")
        print("判定：" + ("通过（100.000%，零违反）" if ok else "不通过——这是弱保证"))
        for label, kw, first in failures[:5]:
            print(f"  首次违反：{label}  {kw}  第 {first[0]} 帧")
            for line in first[1]:
                print(f"    · {line}")
        print()
    return 0


def cmd_kinetic(a) -> int:
    engines = ["v1", "v2"] if a.engine == "both" else [a.engine]
    print(
        f"弹性台球：无重力、阻尼置 1、恢复系数 {a.restitution}、无摩擦，"
        f"n={a.n}，{a.frames} 帧，初速 ±{a.v0}\n"
        "这时能量损失只可能来自非牛顿操作，不会和真实的非弹性碰撞混在一起。\n"
    )
    for engine in engines:
        # 两个引擎的 friction 语义不同：v1 是切向速度的保留比例（1 = 无摩擦），v2 是库仑系数 μ（0 = 无摩擦）。这一项要的是"无摩擦"，得按引擎分别给。
        fric = 1.0 if engine == "v1" else 0.0
        kw = dict(n=a.n, scene=a.scene, gravity=0.0, damping=1.0,
                  restitution=a.restitution, friction=fric, seed=a.seed)
        if engine == "v2":
            kw["plate_friction"] = 0.0   # 这一项要的是"无耗散"，板面摩擦得关掉
        w = build(engine, kw)
        rng = random.Random(a.seed)
        for b in w.blocks:
            b.vx = fixed.from_float(rng.uniform(-a.v0, a.v0))
            b.vy = fixed.from_float(rng.uniform(-a.v0, a.v0))
        w.set_tilt(0.0)
        k0 = w.kinetic()
        for _ in range(a.frames):
            w.frame()
        k1 = w.kinetic()
        kept = k1 / k0 if k0 else 0.0
        print(f"引擎 {engine}")
        print(f"  初态动能        {k0:.6f}")
        print(f"  末态动能        {k1:.6f}")
        print(f"  保留比例        {kept * 100:.2f}%   ← 损失全部来自碰撞、摩擦与非牛顿操作")
        if engine == "v1":
            s = w.stats
            print(f"  配对碰撞        {s.pair_events}")
            print(f"  占用约束触发    {s.cell_events}")
            print(f"  占用约束失败    {s.cell_failures}")
        else:
            print(f"  提交/阻挡/撞墙  {w.stats['commits']} / {w.stats['blocked']} / {w.stats['wall']}")
        print()
    return 0


def cmd_plate(a) -> int:
    """板面静摩擦的启滑阈值：单块、空盘、无墙，逐 LSB 扫平面内重力。

    连续解是 g∥ = G·μ/√(1+μ²)，因为法向 N = √(G²−g∥²) 本身也随倾角变。离散实现里阈值落在 μ_p·N 取整之后，逐 LSB 扫出来和连续解对表，差一两个 LSB 是定点取整，不是模型错。
    """
    print("板面静摩擦：单块、空盘、无墙，扫平面内重力 g∥，看 240 帧内是否移动\n")
    print(f"总重力 G = {a.gravity_total}    帧率按 60 Hz\n")
    head = ["μ_p", "连续解 g∥", "实测阈值", "实测/连续", "阈值处 N"]
    widths = [6, 12, 12, 12, 12]
    print("  " + "".join(pad(h, w) for h, w in zip(head, widths)))
    for mu in a.mu:
        pred = a.gravity_total * mu / math.sqrt(1.0 + mu * mu)
        gmax = fixed.from_float(a.gravity_total)
        hit = None
        for k in range(1, gmax + 1):
            w = V2.World2(
                V2.Params(n=1, gravity=k / fixed.ONE, gravity_total=a.gravity_total,
                          plate_friction=mu, damping=0.9995),
                cells=[(4, 4)],
            )
            w.set_tilt(0.0)
            y0 = w.blocks[0].py
            for _ in range(240):
                w.frame()
            if w.blocks[0].py != y0:
                hit = k / fixed.ONE
                break
        if hit is None:
            print("  " + "".join(pad(c, wd) for c, wd in zip(
                [f"{mu:.2f}", f"{pred:.6f}", "未启滑", "—", "—"], widths)))
            continue
        n_thr = math.sqrt(max(0.0, a.gravity_total ** 2 - hit * hit))
        print("  " + "".join(pad(c, wd) for c, wd in zip(
            [f"{mu:.2f}", f"{pred:.6f}", f"{hit:.6f}", f"{hit / pred:.4f}", f"{n_thr:.6f}"],
            widths)))
    print("\n每档扫到第一档会动的 g∥ 就停；连续解忽略了阻尼因子 0.9995 和 μ_p·N 的取整。")
    return 0


def cmd_ops(a) -> int:
    engines = ["v1", "v2"] if a.engine == "both" else [a.engine]
    print(f"每帧定点运算数（n={a.n}，随机倾角，取第 2 帧）\n")
    for engine in engines:
        w = build(engine, dict(n=a.n, seed=a.seed, scene="full"))
        w.set_tilt(20.0)
        w.frame()  # 预热，不计
        fixed.reset_counts()
        fixed.COUNT = True
        w.frame()
        fixed.COUNT = False
        counts = dict(fixed.ops)
        total = sum(counts.values())
        print(f"引擎 {engine}   每帧 {total} 次（{w.p.substeps} 子步 → 每子步 {total // w.p.substeps} 次）")
        for name in sorted(counts):
            print(f"  {name:<6}{counts[name]:>8}")
        print()
    return 0


def main(argv: list[str]) -> int:
    use_utf8()
    p = argparse.ArgumentParser(description="物块模式物理内核评测")
    sub = p.add_subparsers(dest="cmd", required=True)

    for name in ("invariant", "kinetic", "ops", "plate"):
        q = sub.add_parser(name)
        q.add_argument("--engine", choices=["v1", "v2", "both"], default="v2")
        q.add_argument("--seed", type=int, default=3)
        q.add_argument("--n", type=int, default=6)
        q.add_argument("--frames", type=int, default=600)
        q.add_argument("--v0", type=float, default=0.2)
        q.add_argument("--restitution", type=float, default=1.0,
                       help="kinetic 用。1 为完全弹性，此时损失全部来自非牛顿操作")
        q.add_argument("--scene", choices=("top", "full", "row", "column", "corner"),
                       default="corner", help="kinetic 用。corner 让物块挤在一起，制造碰撞")
        q.add_argument("--quick", action="store_true")
        if name == "plate":
            q.add_argument("--mu", type=float, nargs="+", default=[0.2, 0.35, 0.55, 0.9],
                           help="板面库仑系数 μ_p，可以给多个")
            q.add_argument("--gravity-total", type=float, default=0.0012,
                           help="总重力 G")

    a = p.parse_args(argv)
    return {"invariant": cmd_invariant, "kinetic": cmd_kinetic,
            "ops": cmd_ops, "plate": cmd_plate}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
