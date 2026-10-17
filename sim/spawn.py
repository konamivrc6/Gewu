"""统一的初始摆位。

两个引擎必须吃同一份格号列表，否则 `--engine` 的 A/B 对比里会混进"初始条件不同"这一项。场景名也在这里一次定死，不要在引擎里各自解释一遍——`world.py` 和 `world2.py` 原来各有一个 `random`，一个取顶部三行、一个取全盘，名字一样行为不一样，这种差异在对比里是隐形的。

场景：

- `top`     顶部三行随机取 n 格。v1 的历史 default，原失败配置（tilt 80 / n 6 / seed 5）用的是它
- `full`    全盘随机取 n 格。v2 的历史 default
- `row`     从 (0, 1) 起逐行横排
- `column`  第 w//2 列自顶向下
- `corner`  左上 3×3 内行优先
- `center`  居中的正方形块，行优先填满。转动的演示要它：绕板心转，一坨居中的料才能均匀地甩向四角

所有场景在返回前都验一遍：数量对、格号两两不同、都在界内。两个引擎因此可以假定拿到的列表是合法的，不必各写一遍检查。
"""

from __future__ import annotations

import math
import random

SCENES = ("top", "full", "row", "column", "corner", "center")


def cells(scene: str, n: int, w: int = 8, h: int = 8, seed: int = 1) -> list[tuple[int, int]]:
    if n < 1:
        raise ValueError(f"物块数必须为正，收到 {n}")
    if scene == "row":
        cs = [(c % w, 1 + c // w) for c in range(n)]
    elif scene == "column":
        cs = [(w // 2, r) for r in range(n)]
    elif scene == "corner":
        cs = [(c % 3, c // 3) for c in range(n)]
    elif scene == "center":
        # 边长取 ceil(sqrt(n))，整块居中。8×8 没有正中格，所以偶数边长时中心落在格线上——这正是要的，离心力的原点在板心
        s = math.isqrt(n - 1) + 1
        x0, y0 = (w - s) // 2, (h - s) // 2
        cs = [(x0 + c % s, y0 + c // s) for c in range(n)]
    elif scene in ("top", "full"):
        rows = min(3, h) if scene == "top" else h
        pool = [(x, y) for y in range(rows) for x in range(w)]
        random.Random(seed).shuffle(pool)
        cs = pool[:n]
    else:
        raise ValueError(f"未知场景 {scene!r}，可选 {' / '.join(SCENES)}")

    return validate(cs, n, w, h, scene)


def validate(cs, n: int, w: int = 8, h: int = 8, label: str = "自定义") -> list[tuple[int, int]]:
    """校验一份格号列表并原样返回。三个引擎接口都走这里。"""
    cs = list(cs)
    if len(cs) != n:
        raise ValueError(f"{label}：需要 {n} 个格，收到 {len(cs)} 个")
    if len(set(cs)) != n:
        raise ValueError(f"{label} 场景在 n={n} 时产生了重复格")
    for (x, y) in cs:
        if not (0 <= x < w and 0 <= y < h):
            raise ValueError(f"{label} 场景在 n={n} 时越界：({x}, {y})")
    return cs
