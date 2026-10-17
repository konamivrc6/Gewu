"""物块模式仿真入口。

    python sim/run.py                      # 默认 v2（格子锚定），终端里实时播
    python sim/run.py --engine v1          # 跑基线：硬球 + 事后修补
    python sim/run.py --frames 600 --n 20  # 多放几个，看看挤不挤
    python sim/run.py --sweep              # 重力方向来回扫，看堆积会不会跟着倒
    python sim/run.py --scene full         # 全盘随机摆位
    python sim/run.py --scene center --n 16 --gravity 0 --spin-rps 0.5   # 平放、绕板心转，料甩向四角
    python sim/run.py --dump frames.json   # 导出帧序列
    python sim/run.py --interactive        # 方向键实时倾斜（Windows）

两个引擎的场景名共用 `spawn.py`，同一个 `--scene` 在两边给出同一份格号列表——否则 A/B 对比里会混进"初始条件不同"这一项。

只依赖标准库。没有 numpy——N 最多几十，值得上的是定点数而不是向量化。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import checker
import spawn as spawn_mod
import world as V1
import world2 as V2
from fixed import to_float


def use_utf8() -> None:
    """Windows 上 stdout 接管道时会退回 ANSI 代码页（zh-CN 就是 GBK），
    点阵画不出来还会直接抛 UnicodeEncodeError。定点阵要 UTF-8。"""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError):
            pass


ANSI_HOME = "\033[H"
ANSI_CLEAR = "\033[2J"
ANSI_DIM = "\033[2m"
ANSI_BRIGHT = "\033[1;97m"
ANSI_RESET = "\033[0m"


def parse_args(argv: list[str]) -> argparse.Namespace:
    a = argparse.ArgumentParser(description="物块模式物理仿真")
    a.add_argument("--engine", choices=["v1", "v2"], default="v2",
                   help="v2 = 格子锚定（当前方案），v1 = 硬球加事后修补（基线）")
    a.add_argument("--n", type=int, default=8, help="物块数量")
    a.add_argument("--seed", type=int, default=1)
    a.add_argument("--scene", choices=spawn_mod.SCENES, default="top",
                   help="top = 顶部三行随机（历史 default），full = 全盘随机")
    a.add_argument("--frames", type=int, default=240)
    a.add_argument("--fps", type=float, default=30.0, help="终端播放帧率；0 表示不限速")
    a.add_argument("--substeps", type=int, default=None, help="默认用各引擎自己的值")
    a.add_argument("--gravity", type=float, default=None, help="格 / 子步²；默认 0.0012")
    a.add_argument("--restitution", type=float, default=None, help="默认 v1 0.35 / v2 0.20")
    a.add_argument("--friction", type=float, default=None, help="块-块与块-墙的库仑系数；默认 v1 0.88 / v2 0.55")
    a.add_argument("--plate-friction", type=float, default=None,
                   help="板面库仑系数 μ_p（只对 v2 有意义）。默认 0.35；法向压力 N 为 0 时不作用")
    a.add_argument("--gravity-total", type=float, default=None,
                   help="总重力 G（只对 v2 有意义）。板面法向 N = sqrt(G² − 重力²)；默认 0.0012")
    a.add_argument("--radius", type=float, default=None, help="格；只对 v1 有意义")
    a.add_argument("--tilt", type=float, default=0.0, help="重力倾角，度。0 为竖直向下")
    a.add_argument("--spin-rps", type=float, default=0.0,
                   help="转速，转/秒（只对 v2 有意义）。方向由 --spin-axis 给")
    a.add_argument("--spin-ramp", type=int, default=0, help="升速用多少帧；0 表示一开始就是满速")
    a.add_argument("--spin-pivot", type=float, nargs=2, default=(4.0, 4.0),
                   help="转动原点 = 加速度计的位置，格坐标")
    a.add_argument("--spin-axis", type=float, nargs=3, default=(0.0, 0.0, 1.0),
                   help="体坐标里的转轴。默认 (0,0,1) = 屏幕法线，屏幕上顺时针；(1,0,0) = 绕板面内的 x 轴")
    a.add_argument("--sweep", action="store_true", help="倾角按正弦来回扫")
    a.add_argument("--sweep-amp", type=float, default=45.0)
    a.add_argument("--sweep-period", type=float, default=180.0, help="多少帧走完一轮")
    a.add_argument("--no-cell", action="store_true", help="关掉一格一物的占用约束（只对 v1 有意义）")
    a.add_argument("--interactive", action="store_true", help="方向键实时倾斜（Windows）")
    a.add_argument("--render", choices=["ansi", "plain", "none"], default="ansi")
    a.add_argument("--every", type=int, default=1, help="plain 模式下每隔几帧打一次")
    a.add_argument("--dump", metavar="FILE", help="把帧序列写成 JSON")
    a.add_argument("--quiet", action="store_true", help="不打印统计")
    return a.parse_args(argv)


class Runner:
    """两个内核的差异收在这一层，其余代码只认这个接口。"""

    def __init__(self, a: argparse.Namespace):
        self.a = a
        self.engine = a.engine

        if a.engine == "v2":
            if a.no_cell:
                raise SystemExit("--no-cell 只对 v1 有意义：v2 的占用约束是构造性的，没有开关可关")
            if a.radius is not None:
                raise SystemExit("--radius 只对 v1 有意义：v2 的碰撞体就是格子本身")
            kw = {"n": a.n}
            if a.substeps is not None:
                kw["substeps"] = a.substeps
            if a.gravity is not None:
                kw["gravity"] = a.gravity
            if a.restitution is not None:
                kw["restitution"] = a.restitution
            if a.friction is not None:
                kw["friction"] = a.friction
            if a.plate_friction is not None:
                kw["plate_friction"] = a.plate_friction
            if a.gravity_total is not None:
                kw["gravity_total"] = a.gravity_total
            self.w = V2.World2(V2.Params(**kw), seed=a.seed, scene=a.scene)
        else:
            if a.spin_rps:
                raise SystemExit("--spin-rps 只对 v2 有意义：v1 没有转动项，基线不补")
            if a.plate_friction is not None or a.gravity_total is not None:
                raise SystemExit("--plate-friction / --gravity-total 只对 v2 有意义")
            kw = {"n": a.n, "cell_enforce": not a.no_cell}
            if a.substeps is not None:
                kw["substeps"] = a.substeps
            if a.gravity is not None:
                kw["gravity"] = a.gravity
            if a.restitution is not None:
                kw["restitution"] = a.restitution
            if a.friction is not None:
                kw["friction"] = a.friction
            if a.radius is not None:
                kw["radius"] = a.radius
            self.w = V1.World(V1.Params(**kw), seed=a.seed, scene=a.scene)

    # ---------- 统一接口 ----------
    @property
    def n(self) -> int:
        return self.w.p.n

    @property
    def tilt(self) -> float:
        return self.w._tilt_deg if self.engine == "v1" else self.w._tilt

    def frame(self) -> None:
        self.w.frame()

    def set_tilt(self, deg: float) -> None:
        self.w.set_tilt(deg)

    def spin(self, frame: int) -> None:
        """按帧号推转速。斜坡那一段 ω̇ 不为零，欧拉项就在那里起作用。"""
        if not self.a.spin_rps:
            return
        ramp = self.a.spin_ramp
        if ramp > 0 and frame < ramp:
            frac = (frame + 1) / ramp
            rps_dot = self.a.spin_rps / (ramp / self.w.frame_hz)
        else:
            frac, rps_dot = 1.0, 0.0
        self.w.set_spin(self.a.spin_rps * frac, rps_dot, pivot=self.a.spin_pivot,
                        axis=self.a.spin_axis)

    def render(self) -> list[str]:
        return self.w.render()

    def mask(self) -> int:
        return self.w.mask()

    def lit_count(self) -> int:
        return self.w.lit_count()

    def kinetic(self) -> float:
        return self.w.kinetic()

    # ---------- 报告 ----------
    def heading(self, frame: int) -> str:
        s = (
            f"帧 {frame:4d}  点亮 {self.lit_count():2d}/{self.n}  "
            f"动能 {self.kinetic():8.5f}  倾角 {self.tilt:+6.1f}度"
        )
        if self.a.spin_rps:
            ramp = self.a.spin_ramp
            frac = min(1.0, (frame + 1) / ramp) if ramp > 0 else 1.0
            s += f"  转速 {self.a.spin_rps * frac:+5.2f} 转/秒"
        if self.engine == "v1":
            st = self.w.stats
            s += f"  占用约束 {st.cell_events} 次  失败 {st.cell_failures}"
        else:
            st = self.w.stats
            s += f"  提交 {st['commits']}  阻挡 {st['blocked']}  撞墙 {st['wall']}"
        return s

    def summary(self) -> list[str]:
        bad = checker.audit(self.w)
        out = [f"引擎            {self.engine}"]
        if self.engine == "v1":
            s = self.w.stats
            out += [
                f"帧数            {s.frames}",
                f"配对碰撞        {s.pair_events}",
                f"撞墙            {s.wall_events}",
                f"占用约束触发    {s.cell_events}",
                f"占用约束失败    {s.cell_failures}   ← 不为 0 就是'N 个物块 = N 个亮点'被破了",
                f"静止判定        {s.sleep_events}",
                f"速度夹紧        {s.vmax_clamps}   ← 不为 0 就有穿隧风险",
            ]
        else:
            s = self.w.stats
            out += [
                f"提交            {s['commits']}",
                f"阻挡            {s['blocked']}",
                f"撞墙            {s['wall']}",
                f"板面法向 N      {to_float(self.w.gn):.5f}    μ_p {to_float(self.w.mup):.3f}    每子步 Δv {to_float(self.w.fdv):.5f}",
            ]
        out.append(f"末帧点亮        {self.lit_count()} / {self.n}")
        out.append(f"末帧 Σv²        {self.kinetic():.6f}")
        out.append(f"独立检查        {'通过' if not bad else f'{len(bad)} 项违反'}")
        for line in bad[:10]:
            out.append(f"  · {line}")
        return out


def tilt_at(a: argparse.Namespace, frame: int) -> float:
    if not a.sweep:
        return a.tilt
    return a.sweep_amp * math.sin(2 * math.pi * frame / a.sweep_period)


def draw_ansi(r: Runner, frame: int, heading: str) -> None:
    rows = r.render()
    width = len(rows[0]) if rows else 0
    out = [ANSI_HOME + ANSI_CLEAR, heading + "\n"]
    out.append("┌" + "─" * width + "┐\n")
    for row in rows:
        line = "".join(
            (ANSI_BRIGHT + "█" + ANSI_RESET) if ch == "█" else (ANSI_DIM + "·" + ANSI_RESET)
            for ch in row
        )
        out.append("│" + line + "│\n")
    out.append("└" + "─" * width + "┘\n")
    sys.stdout.write("".join(out))
    sys.stdout.flush()


def draw_plain(r: Runner, frame: int, a: argparse.Namespace, heading: str) -> None:
    print(f"--- frame {frame}  {heading}")
    for row in r.render():
        print("   " + row.replace("·", ".").replace("█", "#"))


def run_interactive(r: Runner, a: argparse.Namespace) -> None:
    import msvcrt

    keys = {b"H": -a.sweep_amp, b"P": +a.sweep_amp, b"K": 0}
    tilt = a.tilt
    frame = 0
    try:
        while True:
            while msvcrt.kbhit():
                k = msvcrt.getch()
                if k in (b"\x00", b"\xe0"):
                    k2 = msvcrt.getch()
                    if k2 in keys:
                        tilt = max(-90.0, min(90.0, tilt + keys[k2]))
                elif k in (b"q", b"Q", b"\x1b"):
                    return
            r.set_tilt(tilt)
            r.frame()
            draw_ansi(r, frame, "方向键倾斜 · q 退出")
            frame += 1
            time.sleep(1 / a.fps)
    except KeyboardInterrupt:
        return


def main(argv: list[str]) -> int:
    use_utf8()
    a = parse_args(argv)
    r = Runner(a)

    if a.interactive:
        run_interactive(r, a)
        return 0

    dumped: list[int] = []
    renderer = {"ansi": draw_ansi, "plain": draw_plain, "none": None}[a.render]
    delay = (1.0 / a.fps) if (a.fps > 0 and a.render == "ansi") else 0.0

    for frame in range(a.frames):
        r.set_tilt(tilt_at(a, frame))
        r.spin(frame)
        r.frame()
        if a.dump:
            dumped.append(r.mask())
        if renderer is None:
            continue
        show = a.render == "ansi" or frame % a.every == 0
        if show:
            if renderer is draw_plain:
                draw_plain(r, frame, a, r.heading(frame))
            else:
                draw_ansi(r, frame, r.heading(frame))
        if delay:
            time.sleep(delay)

    if a.render == "ansi":
        print()

    if a.dump:
        with open(a.dump, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "params": {
                        "engine": a.engine,
                        "n": a.n,
                        "scene": a.scene,
                        "world": [8, 8],
                        "frames": a.frames,
                        "fps": a.fps,
                    },
                    "masks": dumped,
                },
                f,
            )

    if not a.quiet:
        for line in r.summary():
            print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
