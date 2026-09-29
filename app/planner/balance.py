"""圆顶检修静平衡配重规划：离散槽位枚举 + 偏差情景穷举验收。

问题
----
检修后已装部件相对旋转轴存在残余**质量矩**（质量 × 旋转轴半径的二维向量，
方位角**北为 0°、向东递增**）。在至多两个候选槽位上加装离散配重，使残余
质量矩尽可能小，并在配重的质量/方位偏差下仍不超过给定限值。

残余质量矩 = 原始质量矩 + 各配重质量矩的向量和：配重装在槽位给定方位处，
其质量矩方向与槽位半径向量同向，因此要抵消原始质量矩，槽位应位于原始
质量矩的**对侧**（方位相差约 180°）。

约定
----
- 每个槽位的配重质量取 ``k · mass_step``（``k`` 为非负整数），非零档位
  最多 40 个且不得超过该槽 ``mass_limit``（即 ``40 · mass_step`` 不得超过
  ``mass_limit``）；
- 至多使用**两个**槽位（允许只用一个，也允许一个配重都不加）；
- 每个非零配重独立承受制造/安装偏差：
  质量取标称及 ``± mass_tolerance_ratio · mass``（即 1±ratio 三档），
  方位取标称及 ``± azimuth_tolerance``（三档），组合成 9 个验收情景；
  全部配重（最多两个）的情景再做笛卡尔积，**穷举**全部组合。

择优（逐级比较，差值不超过 ``TIE_TOL = 1e-9`` 视为并列）：

1. 最坏残余质量矩模长（全部偏差情景的最大值）最小；
2. 标称残余质量矩模长最小；
3. 总配重质量最小；
4. 按槽位 ID 排序的**全槽位**质量向量（未使用槽位记 0）字典序最小。

偏差组合的最坏值并列时，取按槽位 ID 顺序、先质量偏差 ``(-, 0, +)``
后方位偏差 ``(-, 0, +)`` 的首个组合。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

FULL_TURN = 360.0

# 择优并列容差：两个标量差值不超过该值即视为并列
TIE_TOL = 1e-9

MAX_SLOTS = 8
MAX_STEPS_PER_SLOT = 40
MAX_USED_SLOTS = 2


@dataclass(frozen=True)
class InstalledPart:
    """已装部件：质量、旋转轴半径与方位（度，北 0° 向东递增）。"""

    mass: float
    radius: float
    azimuth: float


@dataclass(frozen=True)
class BalanceSlot:
    """候选配重槽位。"""

    slot_id: str
    radius: float
    azimuth: float
    mass_limit: float


@dataclass(frozen=True)
class SlotLevel:
    """某槽位的一个非零质量档位。"""

    slot_index: int
    slot_id: str
    level: int
    mass: float
    radius: float
    azimuth: float
    # 标称质量矩分量（北、东）
    mn: float
    me: float


@dataclass(frozen=True)
class WeightItem:
    """入选方案中的一个非零配重。"""

    slot_id: str
    level: int
    mass: float
    radius: float
    azimuth: float


@dataclass
class BalanceResult:
    """枚举择优结果。"""

    weights: list[WeightItem]
    total_mass: float
    nominal_rx: float
    nominal_ry: float
    nominal_magnitude: float
    nominal_azimuth: float
    worst_rx: float
    worst_ry: float
    worst_magnitude: float
    worst_azimuth: float
    worst_deviation: list[dict]
    original_rx: float
    original_ry: float
    original_magnitude: float
    original_azimuth: float
    plans_evaluated: int
    scenarios_evaluated: int
    feasible: bool
    residual_moment_limit: float
    slot_ids: list[str]


def _azimuth_of(x: float, y: float) -> float:
    """二维向量（北 x、东 y）的方位角（度，[0, 360)）；零向量记 0。"""
    if x * x + y * y <= TIE_TOL * TIE_TOL:
        return 0.0
    return math.degrees(math.atan2(y, x)) % FULL_TURN


def plan_dome_balance(
    parts: list[InstalledPart],
    slots: list[BalanceSlot],
    mass_step: float,
    mass_tolerance_ratio: float,
    azimuth_tolerance: float,
    residual_moment_limit: float,
) -> BalanceResult:
    """精确枚举全部配重方案并穷举偏差情景，按文档规则择优。

    参数合法性由 Pydantic 请求模型负责；本函数假定全部参数有限、槽位 ID
    唯一。槽位在内部统一按 ID 排序，因此结果与输入顺序无关。
    """
    if not (1 <= len(slots) <= MAX_SLOTS):
        raise ValueError("候选槽位数量必须在 1..%d 之间" % MAX_SLOTS)

    # 槽位按 ID 排序：决定质量向量字典序、配重清单与最坏偏差组合的排列
    slots = sorted(slots, key=lambda s: s.slot_id)

    # 原始（未补偿）质量矩：北/东分量
    ox = oy = 0.0
    for p in parts:
        r = p.mass * p.radius
        a = math.radians(p.azimuth)
        ox += r * math.cos(a)
        oy += r * math.sin(a)
    original_mag = math.hypot(ox, oy)

    # 偏差档位（质量乘子, 方位偏移）；ratio/容差为 0 时 9 情景退化重合，
    # 仍逐一计算（首个取最大值的情景即字典序首个，归属确定）。
    # 顺序固定：先质量偏差 (-, 0, +)，再方位偏差 (-, 0, +)（嵌套）。
    mass_signs = (-1.0, 0.0, 1.0)
    az_signs = (-1.0, 0.0, 1.0)
    corner_specs: list[tuple[float, float, str, str]] = []
    for ms in mass_signs:
        for azs in az_signs:
            corner_specs.append((
                1.0 + ms * mass_tolerance_ratio,
                azs * azimuth_tolerance,
                "nominal" if ms == 0.0 else ("high" if ms > 0 else "low"),
                "nominal" if azs == 0.0 else ("plus" if azs > 0 else "minus"),
            ))

    # 各槽位的非零档位，并为每个档位预算 9 个偏差情景的质量矩分量
    # （角点坐标不随方案改变，避免在内层循环反复做三角函数）。
    levels_by_slot: list[list[SlotLevel]] = []
    corners_by_level: dict[int, list[tuple[float, float]]] = {}
    for si, s in enumerate(slots):
        max_level = int(math.floor(s.mass_limit / mass_step + TIE_TOL))
        if max_level < 1:
            # mass_limit < mass_step：该槽无法加装任何非零配重
            levels: list[SlotLevel] = []
        else:
            if max_level > MAX_STEPS_PER_SLOT:
                raise ValueError(
                    f"槽位 {s.slot_id} 的非零质量档位数 {max_level} 超过上限 "
                    f"{MAX_STEPS_PER_SLOT}（要求 40*mass_step <= mass_limit）"
                )
            a = math.radians(s.azimuth)
            ca, sa = math.cos(a), math.sin(a)
            levels = []
            for k in range(1, max_level + 1):
                m = k * mass_step
                rr = m * s.radius
                lv = SlotLevel(
                    slot_index=si,
                    slot_id=s.slot_id,
                    level=k,
                    mass=m,
                    radius=s.radius,
                    azimuth=s.azimuth,
                    mn=rr * ca,
                    me=rr * sa,
                )
                corners = []
                for factor, daz, _mtag, _atag in corner_specs:
                    aa = math.radians(s.azimuth + daz)
                    # factor 作用于质量（等价于作用于整个质量矩）
                    corners.append((rr * factor * math.cos(aa),
                                    rr * factor * math.sin(aa)))
                corners_by_level[id(lv)] = corners
                levels.append(lv)
        levels_by_slot.append(levels)

    all_levels: list[SlotLevel] = [lv for ls in levels_by_slot for lv in ls]

    def worst_single(w: SlotLevel):
        """单配重方案的 9 情景穷举。

        返回 ``(sq, rx, ry, corner_index)``；严格更大才替换，并列保留
        穷举顺序中的首个（字典序最小偏差组合）。
        """
        corners = corners_by_level[id(w)]
        best = None
        for idx, (cx, cy) in enumerate(corners):
            rx = ox + cx
            ry = oy + cy
            sq = rx * rx + ry * ry
            if best is None or sq > best[0]:
                best = (sq, rx, ry, idx)
        return best

    def worst_pair(w1: SlotLevel, w2: SlotLevel):
        """双配重方案的 9×9=81 情景穷举，返回角点索引对。"""
        c1 = corners_by_level[id(w1)]
        c2 = corners_by_level[id(w2)]
        best = None
        for i1, (c1x, c1y) in enumerate(c1):
            bx = ox + c1x
            by = oy + c1y
            for i2, (c2x, c2y) in enumerate(c2):
                rx = bx + c2x
                ry = by + c2y
                sq = rx * rx + ry * ry
                if best is None or sq > best[0]:
                    best = (sq, rx, ry, i1, i2)
        return best

    # 方案枚举：空方案 + 单槽 + 双槽（至多两个槽位、每槽一个配重）。
    n_single = len(all_levels)
    pair_levels: list[tuple[SlotLevel, SlotLevel]] = []
    for i in range(len(all_levels)):
        for j in range(i + 1, len(all_levels)):
            if all_levels[i].slot_index != all_levels[j].slot_index:
                pair_levels.append((all_levels[i], all_levels[j]))
    plans_count = 1 + n_single + len(pair_levels)
    # 情景总数：空 1 + 单槽 9/档 + 双槽 81/对（偏差为 0 时情景退化重合，
    # 仍逐一穷举并计入）。
    scenarios_count = 1 + 9 * n_single + 81 * len(pair_levels)

    def consider(best_key, best_payload, weights: tuple[SlotLevel, ...],
                 wsq: float, wx: float, wy: float, nmag: float,
                 total_mass: float):
        vec = [0.0] * n_slots
        for w in weights:
            vec[w.slot_index] = w.mass
        wmag = math.sqrt(wsq)
        key = (wmag, nmag, total_mass, tuple(vec))
        improved = best_key is None or _key_less(key, best_key)
        if improved:
            return key, (weights, nmag, wx, wy, wmag, total_mass), True
        return best_key, best_payload, False

    best_key = None
    best_payload = None
    n_slots = len(slots)
    best_worst_indices: tuple = ()

    # 空方案（唯一的零配重方案，最坏情景索引为空）
    best_key, best_payload, _ = consider(
        best_key, best_payload, (), ox * ox + oy * oy, ox, oy,
        math.hypot(ox, oy), 0.0,
    )

    # 单槽方案
    for lv in all_levels:
        wsq, wx, wy, ci = worst_single(lv)
        nx = ox + lv.mn
        ny = oy + lv.me
        best_key, best_payload, improved = consider(
            best_key, best_payload, (lv,), wsq, wx, wy,
            math.hypot(nx, ny), lv.mass,
        )
        if improved:
            best_worst_indices = (ci,)

    # 双槽方案
    for lv1, lv2 in pair_levels:
        wsq, wx, wy, ci1, ci2 = worst_pair(lv1, lv2)
        nx = ox + lv1.mn + lv2.mn
        ny = oy + lv1.me + lv2.me
        best_key, best_payload, improved = consider(
            best_key, best_payload, (lv1, lv2), wsq, wx, wy,
            math.hypot(nx, ny), lv1.mass + lv2.mass,
        )
        if improved:
            best_worst_indices = (ci1, ci2)

    assert best_key is not None and best_payload is not None
    weights, nmag, wx, wy, wmag, total_mass = best_payload
    nx = ox + sum(w.mn for w in weights)
    ny = oy + sum(w.me for w in weights)

    # 由最坏情景角点索引取每个配重的偏差标签（角点顺序固定）
    worst_deviation = [
        {
            "slot_id": w.slot_id,
            "mass_deviation": corner_specs[ci][2],
            "azimuth_deviation": corner_specs[ci][3],
        }
        for w, ci in zip(weights, best_worst_indices)
    ]

    return BalanceResult(
        weights=[
            WeightItem(
                slot_id=w.slot_id,
                level=w.level,
                mass=w.mass,
                radius=w.radius,
                azimuth=w.azimuth,
            )
            for w in weights
        ],
        total_mass=total_mass,
        nominal_rx=nx,
        nominal_ry=ny,
        nominal_magnitude=nmag,
        nominal_azimuth=_azimuth_of(nx, ny),
        worst_rx=wx,
        worst_ry=wy,
        worst_magnitude=wmag,
        worst_azimuth=_azimuth_of(wx, wy),
        worst_deviation=worst_deviation,
        original_rx=ox,
        original_ry=oy,
        original_magnitude=original_mag,
        original_azimuth=_azimuth_of(ox, oy),
        plans_evaluated=plans_count,
        scenarios_evaluated=scenarios_count,
        feasible=wmag <= residual_moment_limit + TIE_TOL,
        residual_moment_limit=residual_moment_limit,
        slot_ids=[s.slot_id for s in slots],
    )


def _scalar_less(a: float, b: float) -> bool:
    """择优标量比较：差值不超过 TIE_TOL 视为并列。"""
    if a < b - TIE_TOL:
        return True
    return False


def _key_less(a: tuple, b: tuple) -> bool:
    """四级择优键比较：worst、nominal、total_mass 带容差，质量向量字典序精确。"""
    for idx in range(3):
        if _scalar_less(a[idx], b[idx]):
            return True
        if _scalar_less(b[idx], a[idx]):
            return False
    # 质量向量逐分量字典序（浮点精确比较；档位质量同为 mass_step 整数倍，
    # 同分量相等在二进制下严格相等）
    va, vb = a[3], b[3]
    for x, y in zip(va, vb):
        if x < y:
            return True
        if x > y:
            return False
    return False
