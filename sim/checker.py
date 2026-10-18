"""独立于引擎的不变量检查。

**不读引擎自己的计数器。** 引擎里的计数把"正在处理"和"处理失败"混在一起，读不出东西；而且一个由引擎自己报告的违反数，本身就要被怀疑。这里直接扫状态。

两种内核查的东西不一样：

- v2（格子锚定）：格号两两不同；相位落在自己那一格里；`owner` 表与块状态互为逆映射；点亮数等于 N；格号在界内
- v1（硬球 + 修补）：由浮点位置折算的格号两两不同；点亮数等于 N；位置在界内

`audit()` 返回违反项的列表，空列表即通过。这样失败的现场能直接打出来，比一个计数有用。
"""

from __future__ import annotations

import os
import sys

# 内核住在 sim/，定点原语与摆位在同一层；粒子脚本在 sim/particle/ 时会把这个目录加进路径。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fixed


def audit(world) -> list[str]:
    engine = getattr(world, "ENGINE", None)
    if engine == "v2":
        return _audit_v2(world)
    if engine == "v1":
        return _audit_v1(world)
    raise TypeError(f"未知引擎 {engine!r}：引擎类需要声明 ENGINE 属性")


def _audit_v2(w) -> list[str]:
    out: list[str] = []
    n = w.p.n
    blocks = w.blocks
    if len(blocks) != n:
        out.append(f"块数 {len(blocks)} != N {n}")

    seen: dict[tuple[int, int], int] = {}
    for i, b in enumerate(blocks):
        k = (b.cx, b.cy)
        if k in seen:
            out.append(f"块 {i} 与块 {seen[k]} 同格 {k}")
        seen[k] = i
        if not (0 <= b.cx < w.W and 0 <= b.cy < w.H):
            out.append(f"块 {i} 格号越界 {k}")
            continue
        if not (b.cx * fixed.ONE <= b.px < (b.cx + 1) * fixed.ONE):
            out.append(f"块 {i} 的 px={b.px} 不在第 {b.cx} 列（相位跑出本格）")
        if not (b.cy * fixed.ONE <= b.py < (b.cy + 1) * fixed.ONE):
            out.append(f"块 {i} 的 py={b.py} 不在第 {b.cy} 行（相位跑出本格）")
        idx = b.cx + b.cy * w.W
        if w.owner[idx] != i:
            out.append(f"owner[{idx}]={w.owner[idx]}，应为块 {i}")

    # 反方向：owner 表里每个非空项都得指向一个真的待在那格的块
    for idx, j in enumerate(w.owner):
        if j == -1:
            continue
        if not (0 <= j < len(blocks)):
            out.append(f"owner[{idx}]={j} 指向不存在的块")
            continue
        b = blocks[j]
        if b.cx + b.cy * w.W != idx:
            out.append(f"owner[{idx}]={j}，但块 {j} 在 {b.cx + b.cy * w.W}")

    lit = bin(w.mask()).count("1")
    if lit != n:
        out.append(f"点亮 {lit} != N {n}")
    return out


def _audit_v1(w) -> list[str]:
    out: list[str] = []
    n = w.p.n
    blocks = w.blocks
    if len(blocks) != n:
        out.append(f"块数 {len(blocks)} != N {n}")

    seen: dict[tuple[int, int], int] = {}
    for i, b in enumerate(blocks):
        k = (fixed.cell(b.px), fixed.cell(b.py))
        if k in seen:
            out.append(f"块 {i} 与块 {seen[k]} 同格 {k}")
        seen[k] = i
        if not (0 <= k[0] < w.p.world_w and 0 <= k[1] < w.p.world_h):
            out.append(f"块 {i} 越界 {k}")

    lit = bin(w.mask()).count("1")
    if lit != n:
        out.append(f"点亮 {lit} != N {n}")
    return out
