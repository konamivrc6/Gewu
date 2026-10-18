"""物块模式的物理内核。

一像素一个物块，牛顿动力学 + 硬球碰撞，最后叠一层"一格一物"的占用约束。

为什么占用约束必须单独写：**硬球不重叠和"一格一物"互不蕴含**。半径 0.5、格距 1
时，同格的两个球可以分处对角（距离 1.41，不重叠），邻格的两个球可以紧贴（距离
0.2，重叠）。而且不存在任何半径能让两者等价——要让同格任意两点必然重叠需要半径
≥ √2/2（直径 1.41），而直径 1.41 会让相邻格的球必然重叠。前者要小半径，后者要
大半径，无解。所以两条约束都得显式写。

坐标系：x 向右，y **向下**，原点在左上角，单位是格。位置范围 [radius, 8-radius]。
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass

# 脚本住在 sim/particle/，公共模块（fixed / spawn）在上一层，先把它加进搜索路径。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fixed
import spawn
from fixed import ONE, add, div, from_float, mul, neg, sqrt, sub, to_float


@dataclass
class Params:
    n: int = 8
    world_w: int = 8
    world_h: int = 8

    radius: float = 0.5          # 格。0.5 即"一个像素大的物块"
    gravity: float = 0.0012      # 格 / 子步²
    restitution: float = 0.35    # 碰撞恢复系数，1 为完全弹性
    friction: float = 0.88       # 接触时切向速度的保留比例
    damping: float = 0.999       # 每子步的速度保留比例（空气阻力）

    substeps: int = 4            # 每帧的物理子步数
    collision_iters: int = 4     # 碰撞求解迭代次数，影响堆叠稳定性
    vmax: float = 0.4            # 格 / 子步。超过就夹住——防穿隧的最后一道闸
    v_sleep: float = 0.0022      # 低于此速度且**有接触**才判定静止

    cell_enforce: bool = True    # 占用约束开关，关掉才能看清它到底改了多少东西
    cell_passes: int = 6
    cell_eps: float = 0.02

    def compiled(self) -> "Compiled":
        return Compiled.build(self)


@dataclass
class Compiled:
    """预计算成定点数的参数。内核热路径上不做任何浮点转换。"""

    p: Params
    R: int
    two_R: int
    two_R2: int
    g: int
    e1: int          # (1 + restitution)
    e: int
    fric: int
    damp: int
    vmax: int
    v_sleep: int
    eps: int
    R_min: int
    R_max_x: int
    R_max_y: int

    @classmethod
    def build(cls, p: Params) -> "Compiled":
        R = from_float(p.radius)
        e = from_float(p.restitution)
        return cls(
            p=p,
            R=R,
            two_R=from_float(2 * p.radius),
            two_R2=from_float(4 * p.radius * p.radius),
            g=from_float(p.gravity),
            e1=add(ONE, e),
            e=e,
            fric=from_float(p.friction),
            damp=from_float(p.damping),
            vmax=from_float(p.vmax),
            v_sleep=from_float(p.v_sleep),
            eps=from_float(p.cell_eps),
            R_min=R,
            R_max_x=from_float(p.world_w - p.radius),
            R_max_y=from_float(p.world_h - p.radius),
        )


@dataclass
class Block:
    px: int = 0
    py: int = 0
    vx: int = 0
    vy: int = 0


@dataclass
class Stats:
    frames: int = 0
    cell_events: int = 0         # 占用约束触发次数
    cell_failures: int = 0       # 占用约束跑完仍同格的次数（不变量被破坏）
    wall_events: int = 0
    pair_events: int = 0
    sleep_events: int = 0
    vmax_clamps: int = 0
    peak_speed: float = 0.0

    def worst_lit_mismatch(self) -> int:
        return self.cell_failures


class World:
    ENGINE = "v1"

    def __init__(self, params: Params, seed: int = 1, scene: str = "top", cells=None):
        self.p = params
        self.c = params.compiled()
        self.blocks: list[Block] = []
        self.stats = Stats()
        self.touched: set[int] = set()
        self._tilt_deg = 0.0
        self._spawn(scene, seed, cells)

    # ---------- 初始摆位 ----------

    def _spawn(self, scene: str, seed: int, cells=None) -> None:
        cs = (spawn.validate(cells, self.p.n, self.p.world_w, self.p.world_h) if cells is not None
              else spawn.cells(scene, self.p.n, self.p.world_w, self.p.world_h, seed))
        for (cx, cy) in cs:
            self.blocks.append(
                Block(px=from_float(cx + 0.5), py=from_float(cy + 0.5))
            )

    # ---------- 重力方向 ----------

    def set_tilt(self, degrees: float) -> None:
        """重力方向，角度制。0 表示竖直向下，正值表示世界绕原点顺时针转。"""
        self._tilt_deg = degrees

    def gravity_vector(self) -> tuple[int, int]:
        r = math.radians(self._tilt_deg)
        return (
            mul(self.c.g, from_float(math.sin(r))),
            mul(self.c.g, from_float(math.cos(r))),
        )

    # ---------- 一个子步 ----------

    def substep(self) -> None:
        self.touched.clear()
        gx, gy = self.gravity_vector()
        c = self.c

        for b in self.blocks:
            b.vx = mul(add(b.vx, gx), c.damp)
            b.vy = mul(add(b.vy, gy), c.damp)
            if b.vx > c.vmax or b.vx < -c.vmax or b.vy > c.vmax or b.vy < -c.vmax:
                self.stats.vmax_clamps += 1
                b.vx = fixed.clamp(b.vx, -c.vmax, c.vmax)
                b.vy = fixed.clamp(b.vy, -c.vmax, c.vmax)
            b.px = add(b.px, b.vx)
            b.py = add(b.py, b.vy)

        self._walls()
        self._pairs()
        self._limit_speed()
        self._walls()
        if c.p.cell_enforce:
            self._cells()
            self._walls()
            self._check_cells()
        self._sleep()

    def _walls(self) -> None:
        c = self.c
        for i, b in enumerate(self.blocks):
            hit = False
            if b.px < c.R_min:
                b.px = c.R_min
                if b.vx < 0:
                    b.vx = -mul(b.vx, c.e)
                b.vy = mul(b.vy, c.fric)
                hit = True
            elif b.px > c.R_max_x:
                b.px = c.R_max_x
                if b.vx > 0:
                    b.vx = -mul(b.vx, c.e)
                b.vy = mul(b.vy, c.fric)
                hit = True
            if b.py < c.R_min:
                b.py = c.R_min
                if b.vy < 0:
                    b.vy = -mul(b.vy, c.e)
                b.vx = mul(b.vx, c.fric)
                hit = True
            elif b.py > c.R_max_y:
                b.py = c.R_max_y
                if b.vy > 0:
                    b.vy = -mul(b.vy, c.e)
                b.vx = mul(b.vx, c.fric)
                hit = True
            if hit:
                self.touched.add(i)
                self.stats.wall_events += 1

    def _pairs(self) -> None:
        """等质量硬球的冲量解算。暴力配对——N 最多几十，不值得上空间哈希。"""
        c = self.c
        blocks = self.blocks
        for _ in range(c.p.collision_iters):
            for i in range(len(blocks)):
                a = blocks[i]
                for j in range(i + 1, len(blocks)):
                    b = blocks[j]
                    dx = sub(a.px, b.px)
                    dy = sub(a.py, b.py)
                    d2 = add(mul(dx, dx), mul(dy, dy))
                    if d2 >= c.two_R2:
                        continue
                    if d2 == 0:
                        dx, dy, d2 = ONE, 0, ONE  # 完全重合时取一个确定方向，避免除零
                    d = sqrt(d2)
                    nx = div(dx, d)
                    ny = div(dy, d)

                    # 位置修正：各退一半
                    push = fixed.clamp(sub(c.two_R, d), 0, c.two_R) >> 1
                    if push:
                        a.px = add(a.px, mul(nx, push))
                        a.py = add(a.py, mul(ny, push))
                        b.px = sub(b.px, mul(nx, push))
                        b.py = sub(b.py, mul(ny, push))

                    # 冲量：等质量，j = -(1+e)·vn / 2。法向 n 由 b 指向 a，所以 vn < 0
                    # 表示正在接近，此时 j 必须为**正**——漏掉那个负号会让每次碰撞
                    # 都加速撞进去，新相对速度变成 (2+e)·vn，能量指数爆炸。
                    vn = add(mul(sub(a.vx, b.vx), nx), mul(sub(a.vy, b.vy), ny))
                    if vn < 0:
                        jimp = mul(c.e1, neg(vn)) >> 1
                        a.vx = add(a.vx, mul(jimp, nx))
                        a.vy = add(a.vy, mul(jimp, ny))
                        b.vx = sub(b.vx, mul(jimp, nx))
                        b.vy = sub(b.vy, mul(jimp, ny))
                        self.stats.pair_events += 1
                        self.touched.add(i)
                        self.touched.add(j)

    def _limit_speed(self) -> None:
        """配对解算之后再夹一次速度。

        积分阶段的夹紧管不到碰撞冲量——一次解算就能把速度顶到 vmax 之上。而穿隧的
        条件是**相对**位移超过一个直径：两边各夹在 0.4 格/子步，相对最多 0.8，小于
        直径 1.0。所以只要每次解算后都补夹一次，穿隧在结构上就不可能发生。
        """
        c = self.c
        for b in self.blocks:
            if b.vx > c.vmax or b.vx < -c.vmax or b.vy > c.vmax or b.vy < -c.vmax:
                self.stats.vmax_clamps += 1
                b.vx = fixed.clamp(b.vx, -c.vmax, c.vmax)
                b.vy = fixed.clamp(b.vy, -c.vmax, c.vmax)

    def _cells(self) -> None:
        """占用约束：同一格里只留一个，多的那个顶出去。

        这是一个**非物理**的位移——牛顿动力学收敛不到"一格一物"上去。代价就在这
        里：它抹掉一部分速度（切向分量直接归零），也就是凭空吃掉能量。所以它触发
        的频率和成败都要计数，一旦很高就说明这个模型跟这条约束在打架。
        """
        c = self.c
        blocks = self.blocks
        for _ in range(c.p.cell_passes):
            seen: dict[tuple[int, int], int] = {}
            moved = False
            for i, b in enumerate(blocks):
                key = (fixed.cell(b.px), fixed.cell(b.py))
                if key not in seen:
                    seen[key] = i
                    continue

                other = blocks[seen[key]]
                dx = sub(b.px, other.px)
                dy = sub(b.py, other.py)
                if dx == 0 and dy == 0:
                    dx, dy = ONE, 0

                self.stats.cell_events += 1
                if not self._eject(b, key, dx, dy):
                    # 这堵墙挡住了去路，改推另一个，方向相反
                    if not self._eject(other, key, -dx, -dy):
                        self.stats.cell_failures += 1
                        seen[key] = i
                        continue
                moved = True
                seen[(fixed.cell(b.px), fixed.cell(b.py))] = i
            if not moved:
                return

    def _eject(self, b: Block, key: tuple[int, int], dx: int, dy: int) -> bool:
        """把 b 沿 (dx,dy) 顶出 key 格。顶不动（会出界）就返回 False。"""
        c = self.c
        cx, cy = key
        if abs(dx) >= abs(dy):
            target = from_float(cx + 1 + c.p.cell_eps) if dx > 0 else from_float(cx - c.p.cell_eps)
            if target < c.R_min or target > c.R_max_x:
                return False
            b.px = target
            b.vx = 0
        else:
            target = from_float(cy + 1 + c.p.cell_eps) if dy > 0 else from_float(cy - c.p.cell_eps)
            if target < c.R_min or target > c.R_max_y:
                return False
            b.py = target
            b.vy = 0
        return True

    def _check_cells(self) -> None:
        """占用约束跑完之后复查。这一条如果不为 0，说明'N 个物块 = N 个亮点'被破了。"""
        occupied = {(fixed.cell(b.px), fixed.cell(b.py)) for b in self.blocks}
        if len(occupied) != len(self.blocks):
            self.stats.cell_failures += len(self.blocks) - len(occupied)

    def _sleep(self) -> None:
        """只让**有接触**的慢速物块停下。

        不加"有接触"这个条件会出大问题：一个刚开始下落的物块速度低于阈值就被归零，
        而每个子步重力只给它 +g，只要 g < 阈值它就永远醒不过来——悬在半空。
        """
        c = self.c
        for i, b in enumerate(self.blocks):
            if i not in self.touched:
                continue
            if -c.v_sleep < b.vx < c.v_sleep and -c.v_sleep < b.vy < c.v_sleep:
                if b.vx or b.vy:
                    self.stats.sleep_events += 1
                b.vx = 0
                b.vy = 0

    # ---------- 一帧 ----------

    def frame(self) -> None:
        for _ in range(self.p.substeps):
            self.substep()
        self.stats.frames += 1

    # ---------- 观测 ----------

    def mask(self) -> int:
        """64 位点阵图，bit(y*8+x)。"""
        m = 0
        w = self.p.world_w
        for b in self.blocks:
            m |= 1 << (fixed.cell(b.py) * w + fixed.cell(b.px))
        return m

    def lit_count(self) -> int:
        return bin(self.mask()).count("1")

    def kinetic(self) -> float:
        return sum(to_float(add(mul(b.vx, b.vx), mul(b.vy, b.vy))) for b in self.blocks) / 2

    def speed(self, b: Block) -> float:
        return to_float(sqrt(add(mul(b.vx, b.vx), mul(b.vy, b.vy))))

    def render(self) -> list[str]:
        w, h = self.p.world_w, self.p.world_h
        grid = [["·"] * w for _ in range(h)]
        for b in self.blocks:
            grid[fixed.cell(b.py)][fixed.cell(b.px)] = "█"
        return ["".join(row) for row in grid]
