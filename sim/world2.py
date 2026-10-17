"""格子锚定的连续动力学。

核心想法：**把"一格一物"从事后修补的约束，提升为状态的不变量。**

- 每块显式持有一个整数格号 `(cx, cy)`，以及格内的连续位置 `(px, py)`
- 不变量：格号两两不同。**任何代码路径都不许在目标格被占时改写格号**
- 碰撞形状就是格子本身（边与格线对齐的单位正方形）。于是"同格"和"重叠"是**同一个谓词**，不再是两条互相打架的约束。原文档里"没有半径能调和两者"的证明只对圆盘成立，换成正方形它自己就化了

位置保持连续，速度保持连续，重力方向仍然来自 MPU6050 的任意角度。穿越格边是一个**事件**：目标格为空则提交（位置连续，无瞬移）；目标格被占则该面成为实心接触（法向冲量交换 + 库仑摩擦）。

坐标系：x 向右，y **向下**，原点左上角，单位是格。固件的数学系 y **向上**，所以移植时要在边界翻转 `py`（而且只能有一处翻）。`mask()` 因此是 `bit(cy * 8 + cx)`，cy = 0 在顶部。

**转动**：板子在转，格子坐标系是非惯性系。角速度 `Ω = (Ωx, Ωy, Ωz)`、角加速度 `Ω̇`，块的平面内位置 `r = (rx, ry, 0)`、相对速度 `v = (vx, vy, 0)`，三个伪力（单位质量）在平面内的分量是：

    离心  −Ω×(Ω×r) = ( (Ωy² + Ωz²) rx − ΩxΩy ry ,  (Ωx² + Ωz²) ry − ΩxΩy rx )
    科氏  −2Ω×v     = ( 2Ωz vy , −2Ωz vx )
    欧拉  −Ω̇×r      = ( Ω̇z ry , −Ω̇z rx )

科氏与欧拉的平面内分量只由 **z 分量**给出；`Ωx`、`Ωy` 的科氏与欧拉项**垂直纸面**，被底板的支持力吃掉，平面内看不见。离心不一样，三个分量都有平面内贡献——转轴躺在板面里时（比如把板子当桨叶翻），块会被推离转轴。

注意"转轴在平面内"和"倾斜"不是一回事：倾斜是**重力方向**在体坐标里变了，由 `set_tilt` 管；这里的 `Ω` 管的是非惯性系的伪力。要模拟一段自洽的翻板动作，两个都要按同一条轨迹更新。

转动原点是**加速度计**，不是板心。加速度计读的是 `a_spec = a_原点 − g`，而惯性系里 `a_块 = a_原点 + Ω×(Ω×r) + Ω̇×r + 2Ω×v + a_相对`；消掉 `a_原点` 之后，剩下的正好是 `−a_spec` 加上那三个伪力。原点因此自动落在加速度计上——**不需要知道手在绕哪一点转**。板子绕桌角转、绕腕关节转、平放在桌上原地拧，同一套式子都成立；加速度计偏心带来的那份离心读数并不是误差，它正是把原点从板心搬到加速度计的那一项。模拟里 `pivot` 就是"加速度计落在哪一格"，默认板心。

三个伪力里离心是 Ω 的偶函数，科氏与欧拉是奇函数。所以把世界转置（x↔y）并把 **Ω 这个赝矢量整体取反**（`Ω' = (−Ωy, −Ωx, −Ωz)`），解应当仍然互为转置——`sim/rot.py` 的镜像检验就靠这条。

**板面摩擦**：块躺在底板上，法向压力由总重力与平面内分量之差给出：

    N = sqrt(G² − g∥²)

`G = gravity_total`，`g∥ = gravity`。板子竖直（g∥ = G）时法向压力为零，块其实是在一个竖直的框里滑；板子平放（g∥ = 0）时压力最大。同一个式子把两种姿态接起来，不需要第二套参数。

摩擦按库仑模型每个子步扣一次速度：

    |v| > μ_p·N  →  v ← v·(|v| − μ_p·N)/|v|
    |v| ≤ μ_p·N  →  v ← 0

第二行就是静摩擦。它不是单独的"静止判定"，而是同一条库仑规则的零速端：本子步能提供的冲量足够把速度抵消时，块就停在原地。切向用精确模长，摩擦锥是圆的——逐轴夹紧会得到方锥，45° 方向的阈值高 √2 倍。

只有一个系数，μ_s = μ_k。真实材料的静摩擦系数通常比动摩擦高 10%~50%，这里不区分；要分就得再加一个参数和一条"是否从零起滑"的分支。

板面摩擦是**耗散**项，`sim/rot.py` 里那条"g = 0 时力的比值与转速无关"只在 `plate_friction = 0` 时成立。开着摩擦时离心力随 ω² 长、摩擦力不随 ω 变，于是存在一个临界转速：低于它块被静摩擦钉住，高于它才甩得动。

**表示不了的东西**：块被约束在平面内，所有垂直纸面的加速度都记在底板支持力上，不能为负。快速翻板时 `−2Ω×v` 与 `−Ω̇×r` 的垂直分量可能超过重力，块会真的离开板面——那是三维运动，这个二维模型没有状态可以放它。转动产生的离面分量也没有计入 N：平放绕法线转时它们为零，绕平面内的轴转时会让一部分块压得更紧、另一部分松脱，这里一律按压紧处理。
"""

from __future__ import annotations

import math

import checker
import spawn
from fixed import ONE, add, div, hypot, mul, mul_sq, neg, sub, to_float, from_float, clamp, cell, halve


class Params:
    def __init__(self, **kw):
        self.n = 6
        self.world_w = 8
        self.world_h = 8
        self.gravity = 0.0012           # 平面内重力分量 g∥
        self.gravity_total = 0.0012     # 总重力 G。板面法向压力 N = sqrt(G² − g∥²)
        self.plate_friction = 0.35      # 板面库仑系数 μ_p。N 为 0 时不作用
        self.restitution = 0.20
        self.restitution_slop = 0.004   # 低于这个接近速度按 e = 0 处理
        self.friction = 0.55            # 块-块与块-墙的库仑系数 μ，等于休止角的正切
        self.damping = 0.9995
        self.vmax = 0.6                 # 格 / 子步。必须 < 1，见 __init__ 的检查
        self.substeps = 2
        self.substep_hz = 120.0         # 每秒子步数。只用来把 转/秒 换算成 弧度/子步；固件按 60 Hz 帧 × 2 子步对表
        self.pivot = (4.0, 4.0)         # 加速度计在格坐标里的位置，也就是转动的原点。默认板心
        self.gate_order = "pen"         # "pen" = 离格心最远的先过门
        self.axis_order = "vel"         # "vel"（先过速度大的轴，转置等变）/ "xy" / "yx" / "alt"（隔子步换）
        for k, v in kw.items():
            setattr(self, k, v)


class Block:
    __slots__ = ("px", "py", "vx", "vy", "cx", "cy")

    def __init__(self):
        self.px = self.py = 0
        self.vx = self.vy = 0
        self.cx = self.cy = 0


class World2:
    ENGINE = "v2"

    def __init__(self, params: Params, seed: int = 1, scene: str = "top", cells=None):
        self.p = params
        self.W, self.H = params.world_w, params.world_h
        # 门里 s = ±1 默认了 |t - cur| <= 1，这条靠「每子步位移 < 1 格」保证。把 vmax 提到 1 以上，门会跳过中间那一格直接提交：中间格是否被占完全不检查，等于瞬移穿过一个物块，不变量当场破掉。所以这里硬卡住。
        if not (0 < params.vmax < 1):
            raise ValueError(f"vmax 必须在 (0, 1) 内，收到 {params.vmax}")
        self.g = from_float(params.gravity)
        # 板面法向压力。这是初始化时算一次的常数，不走热路径，用浮点开方就够——Q16.16 存不下 G²（默认值只有 1.5e-6，远小于 1.5e-5 的 LSB），先平方再开方会把压力抹成 0。C 里同样是一次性的常量。
        _gn = math.sqrt(max(0.0, float(params.gravity_total) ** 2 - to_float(self.g) ** 2))
        self.gn = from_float(_gn)
        self.mup = from_float(params.plate_friction)
        # 每子步能抵消的速度：μ_p·N。dt = 1 子步，所以这里就是加速度本身。
        self.fdv = mul(self.mup, self.gn)
        self.e = from_float(params.restitution)
        self.eslop = from_float(params.restitution_slop)
        self.mu = from_float(params.friction)
        self.damp = from_float(params.damping)
        self.vmax = from_float(params.vmax)
        self.eps = from_float(1e-4)
        self.blocks = []
        self.owner = [-1] * (self.W * self.H)
        self.stats = {"commits": 0, "blocked": 0, "wall": 0}
        self._seed = seed
        self._tilt = 0.0
        self._axis_flip = False
        self.omega = (0, 0, 0)          # 弧度 / 子步，体坐标
        self.omega_dot = (0, 0, 0)      # 弧度 / 子步²，体坐标
        self.pivx = from_float(params.pivot[0])
        self.pivy = from_float(params.pivot[1])
        if params.axis_order not in ("xy", "yx", "alt", "vel"):
            raise ValueError(f"axis_order 只能是 xy / yx / alt / vel，收到 {params.axis_order!r}")
        self._spawn(scene, cells)

    # ---------- 摆位：格号两两不同，不变量在构造时成立 ----------
    def _spawn(self, scene: str, cells=None) -> None:
        w = self.W
        cs = (spawn.validate(cells, self.p.n, self.W, self.H) if cells is not None
              else spawn.cells(scene, self.p.n, self.W, self.H, self._seed))
        for (cx, cy) in cs:
            b = Block()
            b.cx, b.cy = cx, cy
            b.px, b.py = from_float(cx + 0.5), from_float(cy + 0.5)
            self.blocks.append(b)
            self.owner[cx + cy * w] = len(self.blocks) - 1

    def set_tilt(self, deg: float) -> None:
        self._tilt = deg

    @property
    def frame_hz(self) -> float:
        return self.p.substep_hz / self.p.substeps

    def set_spin(self, rps: float = 0.0, rps_dot: float = 0.0, pivot=None,
                 axis=(0.0, 0.0, 1.0)) -> None:
        """设定刚体转动。单位是 转/秒 与 转/秒²，`axis` 是体坐标里的转轴方向。

        默认 `axis = (0, 0, 1)`：+x 转向 +y，也就是屏幕上的顺时针。
        转轴在平面内时（例如 `(1, 0, 0)`，把板子当桨叶翻），平面内会多出离心的轴向外推；科氏与欧拉的那部分垂直纸面，见模块文档。

        `axis` 会被归一化，转轴方向与转速分开给。`pivot` 是加速度计的位置（格坐标），给 None 表示不改。
        """
        hz = float(self.p.substep_hz)
        if not hz > 0:
            raise ValueError(f"substep_hz 必须为正，收到 {hz}")
        if pivot is not None:
            self.pivx = from_float(pivot[0])
            self.pivy = from_float(pivot[1])
        n = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
        if n == 0:
            raise ValueError("转轴不能是零向量")
        ux, uy, uz = axis[0] / n, axis[1] / n, axis[2] / n
        w = 2.0 * math.pi * rps / hz
        wd = 2.0 * math.pi * rps_dot / (hz * hz)
        # 上限是防呆不是物理：Ω 到 1 弧度/子步时离心加速度已经到每子步几格，世界会在一个子步内被甩散，定点也快溢出了
        if abs(w) >= 1.0:
            raise ValueError(f"转速上限 {hz / (2.0 * math.pi):.1f} 转/秒（1 弧度/子步），收到 {rps}")
        if abs(wd) >= 10.0:
            raise ValueError(f"角加速度过大：{rps_dot} 转/秒²，上限 {10.0 * hz * hz / (2.0 * math.pi):.1f}")
        self.omega = (from_float(w * ux), from_float(w * uy), from_float(w * uz))
        self.omega_dot = (from_float(wd * ux), from_float(wd * uy), from_float(wd * uz))

    def gvec(self):
        r = math.radians(self._tilt)
        return (mul(self.g, from_float(math.sin(r))),
                mul(self.g, from_float(math.cos(r))))

    # ---------- 一个子步 ----------
    def substep(self) -> None:
        gx, gy = self.gvec()
        wx, wy, wz = self.omega
        wdx, wdy, wdz = self.omega_dot
        spin = (wx != 0 or wy != 0 or wz != 0 or wdx != 0 or wdy != 0 or wdz != 0)
        if spin:
            twz = add(wz, wz)
            cross = mul(wx, wy)     # 离心交叉项系数 Ωx·Ωy
            pivx, pivy = self.pivx, self.pivy
        for b in self.blocks:
            if spin:
                rx = sub(b.px, pivx)
                ry = sub(b.py, pivy)
                # 离心：平面内是 [[Ωy²+Ωz², −ΩxΩy], [−ΩxΩy, Ωx²+Ωz²]]·r。对角项用 mul_sq 一次取整，避免 Ω 小时 Ω² 被量化抹掉
                cx = sub(add(mul_sq(wy, rx), mul_sq(wz, rx)), mul(cross, ry))
                cy = sub(add(mul_sq(wx, ry), mul_sq(wz, ry)), mul(cross, rx))
                # 科氏 +2Ωz(v_y, −v_x)；欧拉 +Ω̇z(r_y, −r_x)。Ωx、Ωy、Ω̇x、Ω̇y 的那部分垂直纸面，被支持力吃掉
                ax = add(add(gx, cx), add(mul(twz, b.vy), mul(wdz, ry)))
                ay = sub(add(gy, cy), add(mul(twz, b.vx), mul(wdz, rx)))
                b.vx = clamp(mul(add(b.vx, ax), self.damp), -self.vmax, self.vmax)
                b.vy = clamp(mul(add(b.vy, ay), self.damp), -self.vmax, self.vmax)
            else:
                b.vx = clamp(mul(add(b.vx, gx), self.damp), -self.vmax, self.vmax)
                b.vy = clamp(mul(add(b.vy, gy), self.damp), -self.vmax, self.vmax)
            # 板面摩擦在跨格门之前：它是体表力，和后面格子面上的法向冲量是两回事
            self._plate_friction(b)
            b.px = add(b.px, b.vx)
            b.py = add(b.py, b.vy)

        # 跨格门：**唯一**改写格号的地方。
        if self.p.gate_order == "pen":
            order = sorted(range(len(self.blocks)), key=self._pen, reverse=True)
        else:
            order = range(len(self.blocks))
        # 轴序。固定先 x 后 y 会让动力学在 x↔y 转置下不等变，也就是格子仍然各向异性。"alt" 每个子步换一次；"vel" 按每块自己的速度分量决定——后者在转置下是真等变的，因为转置把 vx、vy 对调，判断结果也跟着对调。
        mode = self.p.axis_order
        if mode == "alt":
            first = 1 if self._axis_flip else 0
            self._axis_flip = not self._axis_flip
            base_axes = (first, 1 - first)
        elif mode == "yx":
            base_axes = (1, 0)
        else:
            base_axes = (0, 1)

        for i in order:
            if mode == "vel":
                b = self.blocks[i]
                axes = (0, 1) if abs(b.vx) >= abs(b.vy) else (1, 0)
            else:
                axes = base_axes
            for ax in axes:
                self._gate(i, ax)

    def _plate_friction(self, b: Block) -> None:
        """板面库仑摩擦：本子步最多把速度的模减少 fdv = μ_p·N。

        静摩擦不是单独的分支，而是 `s <= fdv` 时把速度整体归零——本子步能提供的冲量足够抵消，块就不动。已经滑起来的块走同一行，所以动摩擦与静摩擦共用一个系数。

        切向必须用精确模长。逐轴夹紧（vx、vy 各自减 fdv）得到的是方锥，45° 方向的阈值高 √2 倍，倾斜扫描会直接看出来。
        """
        if self.fdv <= 0:
            return
        vx, vy = b.vx, b.vy
        s = hypot(vx, vy)
        if s <= self.fdv:
            b.vx = 0
            b.vy = 0
            return
        k = div(sub(s, self.fdv), s)
        b.vx = mul(vx, k)
        b.vy = mul(vy, k)

    def _pen(self, i: int) -> int:
        """离格心的曼哈顿距离，排序用。名字沿用 "pen"，但它不是穿透深度。"""
        b = self.blocks[i]
        mid = ONE >> 1
        return (abs(sub(b.px, add(from_float(b.cx), mid))) +
                abs(sub(b.py, add(from_float(b.cy), mid))))

    def _gate(self, i: int, axis: int) -> None:
        b = self.blocks[i]
        own = self.owner
        cur = b.cx if axis == 0 else b.cy
        pos = b.px if axis == 0 else b.py
        ncells = self.W if axis == 0 else self.H
        t = cell(pos)
        if t == cur:
            # 左/上边界落在 cell 0 的闭端：位置恰好走到 0 时格号仍是 0，门看不见，但物理上已经贴墙。漏掉这一下，左墙会比右墙晚一个子步响应，左右镜像就不再等变。补一次墙判定，把位置按墙的规则推回 eps 处。
            if pos == 0:
                vn = neg(b.vx) if axis == 0 else neg(b.vy)   # s = −1：>0 表示撞向左/上墙
                if vn > 0:
                    if axis == 0:
                        b.px = self.eps
                    else:
                        b.py = self.eps
                    self._wall_contact(b, axis, -1)
                    self.stats["wall"] += 1
            return
        s = 1 if t > cur else -1

        # ---- 目标在界外：实心墙 ----
        if t < 0 or t >= ncells:
            bound = from_float(0 if t < 0 else ncells)
            npos = add(bound, self.eps) if s < 0 else sub(bound, self.eps)
            if axis == 0:
                b.px = npos
            else:
                b.py = npos
            self._wall_contact(b, axis, s)
            self.stats["wall"] += 1
            return

        # ---- 目标格属于另一个块：实心面 ----
        other = b.cy if axis == 0 else b.cx
        cur_idx = cur + other * self.W if axis == 0 else other + cur * self.W
        tgt_idx = t + other * self.W if axis == 0 else other + t * self.W
        j = own[tgt_idx]
        if j == -1:
            own[cur_idx] = -1
            if axis == 0:
                b.cx = t
            else:
                b.cy = t
            own[tgt_idx] = i
            self.stats["commits"] += 1
            return

        # 被挡：贴到本格的边上（格边落在整数处）
        bound = from_float(cur + 1) if s > 0 else from_float(cur)
        npos = sub(bound, self.eps) if s > 0 else add(bound, self.eps)
        if axis == 0:
            b.px = npos
        else:
            b.py = npos
        self._pair_contact(i, j, axis, s)
        self.stats["blocked"] += 1

    # ---------- 接触 ----------
    def _wall_contact(self, b: Block, axis: int, s: int) -> None:
        if axis == 0:
            vn = b.vx if s > 0 else neg(b.vx)      # > 0 表示正在撞进墙里
            if vn > 0:
                b.vx = neg(mul(b.vx, self.e)) if vn > self.eslop else 0
                self._wall_friction(b, 1, vn)
        else:
            vn = b.vy if s > 0 else neg(b.vy)
            if vn > 0:
                b.vy = neg(mul(b.vy, self.e)) if vn > self.eslop else 0
                self._wall_friction(b, 0, vn)

    def _wall_friction(self, b: Block, taxis: int, vn: int) -> None:
        vt = b.vx if taxis == 0 else b.vy
        lim = mul(self.mu, vn)
        d = vt if abs(vt) <= lim else mul(from_float(1 if vt > 0 else -1), lim)
        if taxis == 0:
            b.vx = sub(b.vx, d)
        else:
            b.vy = sub(b.vy, d)

    def _pair_contact(self, i: int, j: int, axis: int, s: int) -> None:
        a, b = self.blocks[i], self.blocks[j]
        fs = from_float(s)
        if axis == 0:
            vn = mul(sub(a.vx, b.vx), fs)
            if vn <= 0:
                return
            e = self.e if vn > self.eslop else 0
            jn = halve(mul(add(ONE, e), vn))
            a.vx = sub(a.vx, mul(fs, jn))
            b.vx = add(b.vx, mul(fs, jn))
            self._pair_friction(a, b, 1, jn)
        else:
            vn = mul(sub(a.vy, b.vy), fs)
            if vn <= 0:
                return
            e = self.e if vn > self.eslop else 0
            jn = halve(mul(add(ONE, e), vn))
            a.vy = sub(a.vy, mul(fs, jn))
            b.vy = add(b.vy, mul(fs, jn))
            self._pair_friction(a, b, 0, jn)

    def _pair_friction(self, a: Block, b: Block, taxis: int, jn: int) -> None:
        """库仑摩擦：切向冲量上限 μ·|jn|，两边等量反向地施加。"""
        va = a.vx if taxis == 0 else a.vy
        vb = b.vx if taxis == 0 else b.vy
        vrel = sub(va, vb)
        lim = mul(self.mu, jn)
        d = vrel if abs(vrel) <= lim else mul(from_float(1 if vrel > 0 else -1), lim)
        h = halve(d)
        if taxis == 0:
            a.vx = sub(a.vx, h)
            b.vx = add(b.vx, h)
        else:
            a.vy = sub(a.vy, h)
            b.vy = add(b.vy, h)

    def frame(self) -> None:
        for _ in range(self.p.substeps):
            self.substep()

    # ---------- 观测 ----------
    def mask(self) -> int:
        m = 0
        for b in self.blocks:
            m |= 1 << (b.cy * self.W + b.cx)
        return m

    def lit_count(self) -> int:
        return bin(self.mask()).count("1")

    def check(self) -> int:
        """不变量自检，返回违反项数。必须恒为 0。

        交给 `checker.audit()`——它直接扫状态，不信任这个类的计数器。原来的实现从块状态重建了一张新表，从来没和 `self.owner` 对过，也没查相位是否还在自己那一格里，而设计文档宣称这两项都验过。
        """
        return len(checker.audit(self))

    def kinetic(self) -> float:
        return sum(to_float(add(mul(b.vx, b.vx), mul(b.vy, b.vy))) for b in self.blocks) / 2

    def render(self) -> list:
        # 字符与 world.py 保持一致，run.py 的两种终端渲染器才能共用
        g = [["·"] * self.W for _ in range(self.H)]
        for b in self.blocks:
            g[b.cy][b.cx] = "█"
        return ["".join(r) for r in g]
