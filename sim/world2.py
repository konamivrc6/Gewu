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

三个伪力里离心是 Ω 的偶函数，科氏与欧拉是奇函数。所以把世界转置（x↔y）并把 **Ω 这个赝矢量整体取反**（`Ω' = (−Ωy, −Ωx, −Ωz)`），解应当仍然互为转置——`sim/particle/rot.py` 的镜像检验就靠这条。

**板面摩擦**：块躺在底板上，法向压力由总重力与平面内分量之差给出：

    N = sqrt(G² − g∥²)

`G = gravity_total`，`g∥ = gravity`。板子竖直（g∥ = G）时法向压力为零，块其实是在一个竖直的框里滑；板子平放（g∥ = 0）时压力最大。同一个式子把两种姿态接起来，不需要第二套参数。

摩擦按库仑模型每个子步扣一次速度：

    |v| > μ_p·N  →  v ← v·(|v| − μ_p·N)/|v|
    |v| ≤ μ_p·N  →  v ← 0

第二行就是静摩擦。它不是单独的"静止判定"，而是同一条库仑规则的零速端：本子步能提供的冲量足够把速度抵消时，块就停在原地。切向用精确模长，摩擦锥是圆的——逐轴夹紧会得到方锥，45° 方向的阈值高 √2 倍。

只有一个系数，μ_s = μ_k。真实材料的静摩擦系数通常比动摩擦高 10%~50%，这里不区分；要分就得再加一个参数和一条"是否从零起滑"的分支。

板面摩擦是**耗散**项，`sim/particle/rot.py` 里那条"g = 0 时力的比值与转速无关"只在 `plate_friction = 0` 时成立。开着摩擦时离心力随 ω² 长、摩擦力不随 ω 变，于是存在一个临界转速：低于它块被静摩擦钉住，高于它才甩得动。

**表示不了的东西**：块被约束在平面内，所有垂直纸面的加速度都记在底板支持力上，不能为负。快速翻板时 `−2Ω×v` 与 `−Ω̇×r` 的垂直分量可能超过重力，块会真的离开板面——那是三维运动，这个二维模型没有状态可以放它。转动产生的离面分量也没有计入 N：平放绕法线转时它们为零，绕平面内的轴转时会让一部分块压得更紧、另一部分松脱，这里一律按压紧处理。

**水模式（可选）**：`Params.cohesion` 打开八邻域内聚力，`cohesion_damping` 是沿键轴的相对速度阻尼，`wetting` 是接触线力，`wall_adhesion` 是壁面虚拟格的法向吸附。四个量默认全 0，粒子模式的路径逐位不变。方向只取格偏移而不取连续位置：八邻域全满时向量和为零，净力只出现在边界，体内没有剪切刚度。一格一颗保证不可压缩，所以内聚力只负责液-气界面的收缩，不需要密度与压力求解。原型与实验在 `sim/water/water.py`。

**静水压强（`Params.hydrostatic`）**：内聚力只让液-气界面收缩，它不传递竖直方向上的重量，所以一根竖立的水柱在格点模型里是完全平衡的——下面的粒子各自被下面一格挡住，上面那份重量没有任何通道变成横向推力。真实液体里这份重量就是压强：`p = ρ g h`，两个相邻液柱的底面压强差会把水从高柱推向低柱。这里把这一项显式补上：每格沿**真实重力反方向**（不是量化到格轴）以半格步长射线前进，找到最后一个还有水的采样点，再用射线穿过最后一格边界的位置把水头插值成连续值；然后对四邻域的水头做中心差分，扣掉沿重力分量，只保留垂直分量作为横向加速度，系数与 `g∥` 成正比。重力方向全程连续，手上慢慢转板子时力的方向不会跳。两侧都是自由面且水头相等时是个真正的对称简并（真实液体靠热涨落破缺），这里按确定性规则朝板心一侧倒，保证左右镜像等变、trace 可复现。重力为零时这一项恒为零，零重力水珠仍只由内聚力决定。
"""

from __future__ import annotations

import math
import os
import sys

# 内核住在 sim/，定点原语与摆位在同一层；粒子脚本在 sim/particle/ 时会把这个目录加进路径。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import checker
import spawn
from fixed import ONE, add, div, hypot, mul, mul_sq, neg, sub, to_float, from_float, clamp, cell, halve

# 八邻域扫描顺序固定：dy 外层、dx 内层，跳过 (0,0)。Python 与 C++ 必须逐项一致，否则定点加法的求和顺序变了，逐位 trace 就对不上。
# 每项是 (dx, dy, ux, uy, 是否对角)。ux/uy 是格偏移的单位向量，对角分量为 1/√2 ≈ 46341/65536。
_COH_OFFSETS = (
    (-1, -1, -46341, -46341, True),
    (0, -1, 0, -ONE, False),
    (1, -1, 46341, -46341, True),
    (-1, 0, -ONE, 0, False),
    (1, 0, ONE, 0, False),
    (-1, 1, -46341, 46341, True),
    (0, 1, 0, ONE, False),
    (1, 1, 46341, 46341, True),
)

# 成对遍历用的半平面偏移，对应 _COH_OFFSETS 的下标：(1,0) (0,1) (1,1) (-1,1)。
# 每条相邻边只从一端访问一次，所以这四个方向恰好覆盖全部 8 条边，不会重复也不会漏。
_COH_PAIR_IDX = (4, 6, 7, 5)


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
        self.substeps = 4               # 每帧子步数。与固件的 setup() 对齐，改这里就是改默认的时间尺度
        self.substep_hz = 240.0         # 每秒子步数。只用来把 转/秒 换算成 弧度/子步；240 / 4 = 60 Hz 帧率
        # 例外：sim/particle/port_trace.py 与 sim/water/port_trace.py 显式传 2 / 120，那是固件 v 命令的逐位基准，
        # 固件的 traceWorld 用的是 PhysicsWorld::Params 的默认值（2 / 120），两边必须一起留在原地。
        self.pivot = (4.0, 4.0)         # 加速度计在格坐标里的位置，也就是转动的原点。默认板心
        self.gate_order = "pen"         # "pen" = 离格心最远的先过门
        self.axis_order = "vel"         # "vel"（先过速度大的轴，转置等变）/ "xy" / "yx" / "alt"（隔子步换）
        # 水模式。四个量默认全 0，粒子模式的路径必须逐位不变。
        self.cohesion = 0.0             # 相邻格之间的内聚加速度，格/子步²。轴向键的强度
        self.cohesion_diag = 0.5        # 对角键相对轴向的权重，0 = 只用四邻域，1 = 与轴向等强
        self.cohesion_damping = 0.0     # 沿键轴的相对速度阻尼，0..1。等效黏性，压住内聚力的振铃
        self.wetting = 0.0              # 接触线力强度，相对 cohesion。≈ cos θ，负值 = 疏水
        self.wetting_gain = 1.0         # 接触线力增益。格点模型缺了连续介质的集体压力，1 往往推不动接触线
        self.wall_adhesion = 0.0        # 壁面虚拟水格的法向吸附强度，相对 cohesion。高值会把水横向压向中间，先默认关
        self.hydrostatic = 0.0          # 静水压强增益，0 = 关闭。横向加速度 = gain · g∥ · 两侧水柱的深度差
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
        # 内聚力的预计算。cohesion 默认 0 时这些向量全为零，substep 里整段跳过，粒子 trace 不受影响。
        if not (0.0 <= params.cohesion_diag <= 1.0):
            raise ValueError(f"cohesion_diag 必须在 [0, 1] 内，收到 {params.cohesion_diag}")
        if not (0.0 <= params.cohesion_damping <= 1.0):
            raise ValueError(f"cohesion_damping 必须在 [0, 1] 内，收到 {params.cohesion_damping}")
        if params.cohesion < 0.0:
            raise ValueError(f"cohesion 不能为负，收到 {params.cohesion}")
        if params.hydrostatic < 0.0:
            raise ValueError(f"hydrostatic 不能为负，收到 {params.hydrostatic}")
        self.aco = from_float(params.cohesion)
        self.acod = from_float(params.cohesion * params.cohesion_diag)
        self.cdamp = from_float(params.cohesion_damping)
        self.wet = from_float(params.wetting)
        self.wline = from_float(params.wetting_gain)
        # 接触线力：Young 方程里的 γ_sv − γ_sl。虚拟壁面格只给法向吸引，这条沿壁面的线力才是接触角的来源。
        self.line_force = mul(mul(self.aco, self.wet), self.wline)
        self.coh_force = []
        self.wet_force = []
        for _dx, _dy, _ux, _uy, _diag in _COH_OFFSETS:
            _a = self.acod if _diag else self.aco
            self.coh_force.append((mul(_a, _ux), mul(_a, _uy)))
            _w = from_float(params.cohesion * (params.cohesion_diag if _diag else 1.0) * params.wall_adhesion)
            self.wet_force.append((mul(_w, _ux), mul(_w, _uy)))
        self.cohesion_on = (self.cdamp != 0 or self.line_force != 0 or
                            any(v != (0, 0) for v in self.coh_force) or
                            any(v != (0, 0) for v in self.wet_force))
        # 静水压强的逐子步状态：真实重力单位向量、射线半格步长、水头表（半格计数）。
        self.press = from_float(params.hydrostatic)
        self.press_on = self.press != 0
        self.press_active = False
        self.press_kg = 0
        self.press_ux = 0
        self.press_uy = ONE
        self.press_ray_x = 0
        self.press_ray_y = -(ONE >> 1)
        self.press_depth = [0] * (self.W * self.H)
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
        # 内聚力与静水压强必须先用全体块的旧状态算完，再统一施加。若边算边改，先处理到的块会改掉邻居用于阻尼的速度，结果依赖块的存储顺序。
        self._update_pressure(gx, gy)
        extra = (self._extra_accels()
                 if (self.cohesion_on or self.press_active) else None)
        for i, b in enumerate(self.blocks):
            if spin:
                rx = sub(b.px, pivx)
                ry = sub(b.py, pivy)
                # 离心：平面内是 [[Ωy²+Ωz², −ΩxΩy], [−ΩxΩy, Ωx²+Ωz²]]·r。对角项用 mul_sq 一次取整，避免 Ω 小时 Ω² 被量化抹掉
                cx = sub(add(mul_sq(wy, rx), mul_sq(wz, rx)), mul(cross, ry))
                cy = sub(add(mul_sq(wx, ry), mul_sq(wz, ry)), mul(cross, rx))
                # 科氏 +2Ωz(v_y, −v_x)；欧拉 +Ω̇z(r_y, −r_x)。Ωx、Ωy、Ω̇x、Ω̇y 的那部分垂直纸面，被支持力吃掉
                ax = add(add(gx, cx), add(mul(twz, b.vy), mul(wdz, ry)))
                ay = sub(add(gy, cy), add(mul(twz, b.vx), mul(wdz, rx)))
                if extra is not None:
                    ax = add(ax, extra[0][i])
                    ay = add(ay, extra[1][i])
                b.vx = clamp(mul(add(b.vx, ax), self.damp), -self.vmax, self.vmax)
                b.vy = clamp(mul(add(b.vy, ay), self.damp), -self.vmax, self.vmax)
            else:
                ax, ay = gx, gy
                if extra is not None:
                    ax = add(ax, extra[0][i])
                    ay = add(ay, extra[1][i])
                b.vx = clamp(mul(add(b.vx, ax), self.damp), -self.vmax, self.vmax)
                b.vy = clamp(mul(add(b.vy, ay), self.damp), -self.vmax, self.vmax)
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

    def _cohesion_wall(self, i: int):
        """单块的壁面项：越界虚拟水格的浸润力 + 接触线力，返回 (fx, fy)。

        内部块（1 ≤ cx ≤ W−2 且 1 ≤ cy ≤ H−2）没有越界邻居，接触线力也不在壁面格上作用，直接返回零。
        这一项与块对之间的相互作用无关，所以留在逐块的路径里；下面 _cohesion_pairs() 只管块与块之间的边。
        """
        b = self.blocks[i]
        W, H = self.W, self.H
        if 0 < b.cx < W - 1 and 0 < b.cy < H - 1:
            return 0, 0
        fx = 0
        fy = 0
        for (dx, dy, _ux, _uy, _diag), (wfx, wfy) in zip(_COH_OFFSETS, self.wet_force):
            nx = b.cx + dx
            ny = b.cy + dy
            if not (0 <= nx < W and 0 <= ny < H):
                fx = add(fx, wfx)
                fy = add(fy, wfy)
        lx, ly = self._contact_line(b)
        return add(fx, lx), add(fy, ly)

    def _cohesion_pairs(self, ax, ay):
        """块与块之间的内聚力与键轴阻尼，成对施加：每条相邻边只算一次，两个块同时更新。

        逐位等价于「每个块各自扫八邻域」，依据三条：
          1. 定点累加是整数加法，可交换、可结合，求和顺序不影响结果；
          2. mul() 的取整是向远离零的方向，严格奇，mul(−a, b) == −mul(a, b)；
          3. 阻尼项在反方向上恰好是同一个数：u' = −u，而 vrel' = (v_j − v_i)·u' = vrel，所以 d' = d，
             两个方向的推力 d·u' = −(d·u) 等大反向。
        力项不用额外论证：coh_force 里每一对相反方向的向量本来就是精确的相反数。
        """
        own = self.owner
        W, H = self.W, self.H
        blocks = self.blocks
        cdamp = self.cdamp
        for i, b in enumerate(blocks):
            cx = b.cx
            cy = b.cy
            vx = b.vx
            vy = b.vy
            for k in _COH_PAIR_IDX:
                dx, dy, ux, uy, _diag = _COH_OFFSETS[k]
                nx = cx + dx
                ny = cy + dy
                if not (0 <= nx < W and 0 <= ny < H):
                    continue
                j = own[nx + ny * W]
                if j < 0:
                    continue
                o = blocks[j]
                cfx, cfy = self.coh_force[k]
                ax[i] = add(ax[i], cfx)
                ay[i] = add(ay[i], cfy)
                ax[j] = sub(ax[j], cfx)
                ay[j] = sub(ay[j], cfy)
                if cdamp != 0:
                    vrel = add(mul(sub(vx, o.vx), ux), mul(sub(vy, o.vy), uy))
                    d = mul(cdamp, vrel)
                    jx = mul(d, ux)          # 只算一次，两个方向的推力就是这个向量及其反号
                    jy = mul(d, uy)
                    ax[i] = sub(ax[i], jx)
                    ay[i] = sub(ay[i], jy)
                    ax[j] = add(ax[j], jx)
                    ay[j] = add(ay[j], jy)

    def _is_water(self, x: int, y: int) -> bool:
        return 0 <= x < self.W and 0 <= y < self.H and self.owner[x + y * self.W] >= 0

    # ---------- 静水压强 ----------

    def _update_pressure(self, gx: int, gy: int) -> None:
        """每子步把重力方向归一化，再给每个水格沿 -ĝ 做一条半格步长的射线，得到真实水头。

        射线方向是连续的，不再量化到八邻域；水头用半格计数（整数），所以定点热路径里只有加法和查表。重力为零时整项关闭，零重力水珠的行为不变。
        水头表每子步重建一次：64 格 × 最多 24 步，和原来每格三次整数格行走同量级。
        """
        if not self.press_on:
            self.press_active = False
            return
        gmag = hypot(gx, gy)
        if gmag == 0:
            self.press_active = False
            return
        self.press_ux = div(gx, gmag)
        self.press_uy = div(gy, gmag)
        self.press_ray_x = halve(neg(self.press_ux))
        self.press_ray_y = halve(neg(self.press_uy))
        # 水头现在是格，Q16.16，直接乘增益：kg = 增益 · g∥
        self.press_kg = mul(self.press, gmag)
        self.press_active = self.press_kg != 0
        if self.press_active:
            self._build_depth()

    def _build_depth(self) -> None:
        """沿真实重力反方向给每个水格算水头，单位是格，Q16.16；空格保持 0。

        半格步进只负责找到「最后一个还有水的采样点」，真正的液面位置用射线穿过最后那格边界的位置做一次线性插值。这样水头随重力方向连续变化，不会在射线跨格时跳半格——这一步是手上转动时不出现台阶的关键。
        """
        depth = self.press_depth
        for k in range(len(depth)):
            depth[k] = 0
        half = ONE >> 1
        rx, ry = self.press_ray_x, self.press_ray_y
        for b in self.blocks:
            px = add(from_float(b.cx), half)
            py = add(from_float(b.cy), half)
            n = 0
            for _ in range(24):
                qx = add(px, rx)
                qy = add(py, ry)
                nx = cell(qx)
                ny = cell(qy)
                if not (0 <= nx < self.W and 0 <= ny < self.H):
                    break
                if self.owner[nx + ny * self.W] < 0:
                    break
                px, py = qx, qy
                n += 1
            # (px,py) 是最后一个仍有水的采样点；下一个采样点已经出格。求这一步走到格边的比例 f ∈ [0,1]。
            ox, oy = cell(px), cell(py)
            qx, qy = add(px, rx), add(py, ry)
            cx, cy = cell(qx), cell(qy)
            best_f = None
            if cx != ox and rx != 0:
                bx = from_float(ox + 1 if cx > ox else ox)
                f = div(sub(bx, px), rx)
                if f >= 0:
                    best_f = f
            if cy != oy and ry != 0:
                by = from_float(oy + 1 if cy > oy else oy)
                f = div(sub(by, py), ry)
                if f >= 0 and (best_f is None or f < best_f):
                    best_f = f
            if best_f is None:
                best_f = 0
            nq = add(from_float(n), best_f)          # 半格计数，含小数
            depth[b.cx + b.cy * self.W] = halve(sub(nq, ONE)) if nq > ONE else 0

    def _pressure(self, i: int):
        """单块的静水压强加速度：对水头表做中心差分，再扣掉沿重力分量。

        `p = ρg·h` 的梯度里，沿重力那一半正好抵消重力，已经由接触与法向支撑处理；这里只施加垂直分量，避免重复抵消。
        两侧都是自由面、水头也相等时是真正的对称简并：按「朝板心一侧倒」做确定性破缺，规则关于板心镜像等变，不引入随机数。
        """
        b = self.blocks[i]
        W, H = self.W, self.H
        depth = self.press_depth
        own = depth[b.cx + b.cy * W]

        def neighbor(x, y):
            if not (0 <= x < W and 0 <= y < H):
                return own          # 墙：法向反作用平衡掉这一侧
            return depth[x + y * W] if self.owner[x + y * W] >= 0 else 0

        dl = neighbor(b.cx - 1, b.cy)
        dr = neighbor(b.cx + 1, b.cy)
        du = neighbor(b.cx, b.cy - 1)
        dd = neighbor(b.cx, b.cy + 1)
        gcx = sub(dr, dl)
        gcy = sub(dd, du)
        gpar = add(mul(gcx, self.press_ux), mul(gcy, self.press_uy))
        tx = sub(gcx, mul(gpar, self.press_ux))
        ty = sub(gcy, mul(gpar, self.press_uy))
        ax = neg(mul(self.press_kg, tx))
        ay = neg(mul(self.press_kg, ty))
        if tx == 0 and ty == 0 and own > 0:
            # 找与真实侧向最接近的一对邻居；两边都空才是自由站立的窄柱。墙侧会给出非零梯度，走不到这里。
            sx, sy = neg(self.press_uy), self.press_ux
            best_p = best_m = None
            for ox, oy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                dot = add(mul(from_float(ox), sx), mul(from_float(oy), sy))
                if best_p is None or dot > best_p[0]:
                    best_p = (dot, ox, oy)
                if best_m is None or dot < best_m[0]:
                    best_m = (dot, ox, oy)

            def empty(x, y):
                return 0 <= x < W and 0 <= y < H and self.owner[x + y * W] < 0

            if empty(b.cx + best_p[1], b.cy + best_p[2]) and empty(b.cx + best_m[1], b.cy + best_m[2]):
                dcx = sub(from_float((W - 1) * 0.5), from_float(b.cx))
                dcy = sub(from_float((H - 1) * 0.5), from_float(b.cy))
                toward = add(mul(dcx, sx), mul(dcy, sy))
                f = mul(self.press_kg, own if toward > 0 else neg(own))
                ax = add(ax, mul(f, sx))
                ay = add(ay, mul(f, sy))
        return ax, ay

    def _extra_accels(self):
        """全场的额外加速度：内聚（壁面项 + 成对项）与静水压强，返回两个列表。

        必须先用全体块的旧状态算完再统一施加——先算的块不能污染后算的块用到的速度。
        改成返回全场数组是因为内聚要走成对路径：每条边只能算一次，就不能再按块独立返回。
        压强仍是逐块量，叠加顺序与原来一致（内聚在前、压强在后）；整数加法可交换，结果逐位不变。
        """
        n = len(self.blocks)
        ax = [0] * n
        ay = [0] * n
        if self.cohesion_on:
            for i in range(n):
                wx, wy = self._cohesion_wall(i)
                if wx or wy:
                    ax[i] = wx
                    ay[i] = wy
            self._cohesion_pairs(ax, ay)
        if self.press_active:
            for i in range(n):
                px, py = self._pressure(i)
                ax[i] = add(ax[i], px)
                ay[i] = add(ay[i], py)
        return ax, ay

    def _contact_line(self, b: Block):
        """接触线力：沿壁面指向背离液体的一侧。

        只在「壁面格 + 沿壁方向一侧空、一侧满」时作用。空的那侧是接触线外侧，力把它往外拉；两侧都空（孤立柱）或都满（平直接触线）时没有定义，不施加。

        这一项是必要的：虚拟壁面键是法向的，单独作用只会把水压向墙，接触线一步也不会爬。连续介质里让液面上升的是压力场与界面曲率的集体作用，格点模型没有那层传递，只能把 Young 方程的线力显式写出来。wetting_gain 就是给这个缺失的集体效应留的旋钮。
        """
        if self.line_force == 0:
            return 0, 0
        f = self.line_force
        fx = 0
        fy = 0
        # 沿壁方向的邻居必须都在界内：越界的那一侧是容器端点，不是液-气界面，把它算成“空”会在池底凭空加一个向下的线力。
        if (b.cx == 0 or b.cx == self.W - 1) and 0 < b.cy < self.H - 1:
            up = self._is_water(b.cx, b.cy - 1)
            down = self._is_water(b.cx, b.cy + 1)
            if not up and down:
                fy = sub(fy, f)
            elif up and not down:
                fy = add(fy, f)
        if (b.cy == 0 or b.cy == self.H - 1) and 0 < b.cx < self.W - 1:
            left = self._is_water(b.cx - 1, b.cy)
            right = self._is_water(b.cx + 1, b.cy)
            if not left and right:
                fx = sub(fx, f)
            elif left and not right:
                fx = add(fx, f)
        return fx, fy

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
