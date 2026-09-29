"""圆顶检修静平衡配重规划：算法穷举正确性与 HTTP API 回归测试。

覆盖：
- 基本单槽精确配平（方位约定：北 0°、向东递增，配重与原始质量矩向量相加）；
- 跨 0° 方位（350°、170°）的配平与方位角归一化；
- 零配重入选（原始已平衡 / 任何配重都只会更差）；
- 并列时按槽位 ID 的质量向量字典序决胜；
- 偏差情景改变最优方案（标称最优 vs 最坏最优不同）；
- 参数校验（槽位数、档位上限、ID 唯一、偏差比例范围等）。
"""

from __future__ import annotations

import itertools
import math
import random as random_mod

import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.planner.balance import (
    BalanceSlot,
    InstalledPart,
    plan_dome_balance,
)

client = TestClient(app)


# ---------------------------------------------------------------------------
# 独立暴力参考实现（与生产代码独立，直接做 9/81 情景笛卡尔积穷举）
# ---------------------------------------------------------------------------


def _brute_force_reference(parts, slots, mass_step, ratio, az_tol):
    """返回全部方案的 (worst_mag, nominal_mag, total_mass, per_slot_mass_dict)。"""
    ox = oy = 0.0
    for p in parts:
        r = p.mass * p.radius
        a = math.radians(p.azimuth)
        ox += r * math.cos(a)
        oy += r * math.sin(a)

    levels = []
    for si, s in enumerate(slots):
        n = int(math.floor(s.mass_limit / mass_step + 1e-9))
        for k in range(1, n + 1):
            levels.append((si, s.slot_id, k, k * mass_step, s.radius, s.azimuth))

    plans: list[tuple] = [()]
    plans += [(lv,) for lv in levels]
    for i in range(len(levels)):
        for j in range(i + 1, len(levels)):
            if levels[i][0] != levels[j][0]:
                plans.append((levels[i], levels[j]))

    rows = []
    for pl in plans:
        nx = ox + sum(w[3] * w[4] * math.cos(math.radians(w[5])) for w in pl)
        ny = oy + sum(w[3] * w[4] * math.sin(math.radians(w[5])) for w in pl)

        corners_per_weight = []
        for w in pl:
            rr = w[3] * w[4]
            cs = []
            for ms in (-1, 0, 1):
                for azs in (-1, 0, 1):
                    f = 1.0 + ms * ratio
                    aa = math.radians(w[5] + azs * az_tol)
                    cs.append((rr * f * math.cos(aa), rr * f * math.sin(aa)))
            corners_per_weight.append(cs)

        worst = 0.0
        for combo in itertools.product(*corners_per_weight):
            rx = ox + sum(c[0] for c in combo)
            ry = oy + sum(c[1] for c in combo)
            worst = max(worst, math.hypot(rx, ry))
        if not pl:
            worst = math.hypot(ox, oy)

        mass_by_slot = {s.slot_id: 0.0 for s in slots}
        for w in pl:
            mass_by_slot[w[1]] = w[3]
        rows.append((
            worst,
            math.hypot(nx, ny),
            sum(w[3] for w in pl),
            tuple(mass_by_slot[s.slot_id] for s in slots),
            [(w[1], w[2]) for w in pl],
        ))
    return rows, ox, oy, len(plans)


def _parts(specs):
    return [InstalledPart(m, r, az) for m, r, az in specs]


def _slots(specs):
    return [BalanceSlot(sid, r, az, lim) for sid, r, az, lim in specs]


# ---------------------------------------------------------------------------
# 算法层：基本正确性
# ---------------------------------------------------------------------------


def test_single_slot_exact_balance():
    # 原始质量矩向东 10（部件 az=90°）；对侧槽位 az=270°，level 10 精确抵消
    res = plan_dome_balance(
        _parts([(10.0, 1.0, 90.0)]),
        _slots([("S1", 1.0, 270.0, 40.0)]),
        mass_step=1.0,
        mass_tolerance_ratio=0.0,
        azimuth_tolerance=0.0,
        residual_moment_limit=0.0,
    )
    assert [(w.slot_id, w.level, w.mass) for w in res.weights] == [
        ("S1", 10, 10.0)
    ]
    assert res.original_magnitude == pytest.approx(10.0)
    assert res.original_azimuth == pytest.approx(90.0)
    assert res.nominal_magnitude == pytest.approx(0.0, abs=1e-9)
    assert res.worst_magnitude == pytest.approx(0.0, abs=1e-9)
    assert res.feasible is True
    assert res.plans_evaluated == 1 + 40  # 空方案 + 40 个单槽档位
    assert res.scenarios_evaluated == 1 + 9 * 40


def test_north_zero_east_increasing_convention():
    # 北/东/南/西四个方向的原始质量矩分量与方位
    for az, (north, east) in [
        (0.0, (10.0, 0.0)),
        (90.0, (0.0, 10.0)),
        (180.0, (-10.0, 0.0)),
        (270.0, (0.0, -10.0)),
    ]:
        res = plan_dome_balance(
            _parts([(10.0, 1.0, az)]),
            _slots([("S", 1.0, (az + 180.0) % 360.0, 40.0)]),
            1.0, 0.0, 0.0, 0.0,
        )
        assert res.original_rx == pytest.approx(north, abs=1e-9)
        assert res.original_ry == pytest.approx(east, abs=1e-9)


def test_unwrapped_input_azimuth_equivalent():
    # -90° 与 270° 是同一方位；630° 与 270° 同
    for slot_az in (270.0, -90.0, 630.0):
        res = plan_dome_balance(
            _parts([(10.0, 1.0, 90.0)]),
            _slots([("S1", 1.0, slot_az, 40.0)]),
            1.0, 0.0, 0.0, 0.0,
        )
        assert [w.level for w in res.weights] == [10]
        assert res.nominal_magnitude == pytest.approx(0.0, abs=1e-9)


def test_crosses_zero_azimuth_350():
    # 原始质量矩方位 350°（跨 0° 边界）；对侧 170° 槽位 level 10 精确抵消
    res = plan_dome_balance(
        _parts([(10.0, 1.0, 350.0)]),
        _slots([("S1", 1.0, 170.0, 40.0)]),
        1.0, 0.0, 0.0, 0.0,
    )
    assert [w.level for w in res.weights] == [10]
    assert res.original_azimuth == pytest.approx(350.0, abs=1e-9)
    assert res.nominal_magnitude == pytest.approx(0.0, abs=1e-9)
    assert 0.0 <= res.nominal_azimuth < 360.0

    # 带偏差时最坏方位可能跨越 0°，输出方位始终归一化到 [0,360)
    res_tol = plan_dome_balance(
        _parts([(10.0, 1.0, 350.0)]),
        _slots([("S1", 1.0, 170.0, 40.0), ("S2", 1.0, 190.0, 40.0)]),
        1.0, 0.03, 3.0, 0.0,
    )
    assert 0.0 <= res_tol.worst_azimuth < 360.0
    assert 0.0 <= res_tol.nominal_azimuth < 360.0


def test_multiple_parts_vector_sum():
    # 两个部件：az 0° 质量矩 6 + az 90° 质量矩 8 → 合矩 10 @ atan2(8,6)≈53.13°
    res = plan_dome_balance(
        _parts([(6.0, 1.0, 0.0), (8.0, 1.0, 90.0)]),
        _slots([("S", 1.0, 233.13010235415598, 40.0)]),
        1.0, 0.0, 0.0, 0.0,
    )
    assert res.original_magnitude == pytest.approx(10.0)
    assert res.original_azimuth == pytest.approx(53.13010235415598, abs=1e-9)


# ---------------------------------------------------------------------------
# 零配重
# ---------------------------------------------------------------------------


def test_zero_counterweight_when_already_balanced():
    # 原始部件自相平衡：任何配重只会增大残余 → 空方案入选
    res = plan_dome_balance(
        _parts([(5.0, 1.0, 0.0), (5.0, 1.0, 180.0)]),
        _slots([("S1", 1.0, 90.0, 40.0), ("S2", 1.0, 270.0, 40.0)]),
        1.0, 0.05, 2.0, 1.0,
    )
    assert res.weights == []
    assert res.total_mass == 0.0
    assert res.worst_deviation == []
    assert res.original_magnitude == pytest.approx(0.0, abs=1e-9)
    assert res.nominal_magnitude == pytest.approx(0.0, abs=1e-9)
    assert res.feasible is True
    # 空方案仍计入枚举（1 个方案、1 个情景）
    assert res.plans_evaluated == 1 + 80 + 40 * 40
    assert res.scenarios_evaluated == 1 + 9 * 80 + 81 * 1600


def test_zero_counterweight_when_all_weights_hurt():
    # 原始矩向东 10，候选槽只能向北/向南加矩：任何非零配重都使残余更大
    res = plan_dome_balance(
        _parts([(10.0, 1.0, 90.0)]),
        _slots([("N", 1.0, 0.0, 10.0), ("S", 1.0, 180.0, 10.0)]),
        1.0, 0.0, 0.0, 100.0,
    )
    assert res.weights == []
    assert res.worst_magnitude == pytest.approx(10.0)


# ---------------------------------------------------------------------------
# 并列
# ---------------------------------------------------------------------------


def test_exact_tie_broken_by_mass_vector():
    # 两个槽位 X/Y 几何完全相同：X10 与 Y10 在最坏/标称/总质量上全部并列，
    # 按槽位 ID 排序的质量向量字典序决胜：(0,10) < (10,0) → Y10。
    res = plan_dome_balance(
        _parts([(10.0, 1.0, 0.0)]),
        _slots([("X", 1.0, 180.0, 40.0), ("Y", 1.0, 180.0, 40.0)]),
        1.0, 0.05, 2.0, 0.0,
    )
    assert [(w.slot_id, w.level) for w in res.weights] == [("Y", 10)]

    # 调换输入顺序（先 Y 后 X）不影响结果：内部按槽位 ID 排序
    res_rev = plan_dome_balance(
        _parts([(10.0, 1.0, 0.0)]),
        _slots([("Y", 1.0, 180.0, 40.0), ("X", 1.0, 180.0, 40.0)]),
        1.0, 0.05, 2.0, 0.0,
    )
    assert [(w.slot_id, w.level) for w in res_rev.weights] == [("Y", 10)]


# ---------------------------------------------------------------------------
# 偏差改变最优方案
# ---------------------------------------------------------------------------


def test_deviation_scenarios_change_optimum():
    # A 槽半径大（每档质量矩 10，1 档即精确抵消），标称最优是 A1（残余 0），
    # 但其偏差敏感度高；B/C 两槽半径 1、位于 ±20°，标称最优为 B5+C6，
    # 而最坏残余最小的方案是 B5+C5（牺牲标称换取最坏鲁棒性）。
    parts = _parts([(10.0, 1.0, 0.0)])
    slots = _slots([
        ("A", 10.0, 170.0, 40.0),
        ("B", 1.0, 160.0, 40.0),
        ("C", 1.0, 200.0, 40.0),
    ])

    # 无偏差时：标称最优为 B5+C6（残余 0.480，总质量 11）；A 槽虽只需
    # 1 档但方位偏 10°，标称残余 1.743，反而更差。
    res_nom = plan_dome_balance(parts, slots, 1.0, 0.0, 0.0, 0.0)
    assert [(w.slot_id, w.level) for w in res_nom.weights] == [("B", 5), ("C", 6)]
    assert res_nom.nominal_magnitude == pytest.approx(0.479885418, abs=1e-7)

    # 有偏差时：最坏残余排序改变最优方案为 B5+C5
    res = plan_dome_balance(parts, slots, 1.0, 0.15, 15.0, 0.0)
    assert [(w.slot_id, w.level) for w in res.weights] == [("B", 5), ("C", 5)]
    assert res.nominal_magnitude == pytest.approx(0.603073792, abs=1e-7)
    assert res.worst_magnitude == pytest.approx(3.112296214, abs=1e-7)

    # 标称最优方案 A1 的最坏残余（独立核验）比入选方案大
    rows, _, _, _ = _brute_force_reference(parts, slots, 1.0, 0.15, 15.0)
    a1 = [r for r in rows if r[4] == [("A", 1)]][0]
    b5c6 = [r for r in rows if r[4] == [("B", 5), ("C", 6)]][0]
    b5c5 = [r for r in rows if r[4] == [("B", 5), ("C", 5)]][0]
    assert b5c5[0] == pytest.approx(3.112296214, abs=1e-7)
    assert a1[0] > b5c5[0]
    assert b5c6[1] < b5c5[1]  # 标称 B5+C6 更小
    assert b5c6[0] > b5c5[0]  # 但最坏 B5+C6 更大 → 偏差确实改变最优

    # 验收限值边界：3.5 达标、3.0 不达标（仍返回最优方案 + feasible=false）
    res_ok = plan_dome_balance(parts, slots, 1.0, 0.15, 15.0, 3.5)
    assert res_ok.feasible is True
    res_bad = plan_dome_balance(parts, slots, 1.0, 0.15, 15.0, 3.0)
    assert res_bad.feasible is False
    assert [(w.slot_id, w.level) for w in res_bad.weights] == [("B", 5), ("C", 5)]

    # 最坏偏差组合（按槽位 ID 顺序）
    assert [(d["slot_id"], d["mass_deviation"], d["azimuth_deviation"])
            for d in res.worst_deviation] == [
        ("B", "low", "plus"),
        ("C", "high", "plus"),
    ]


def test_worst_deviation_is_true_maximum_over_corners():
    parts = _parts([(12.0, 2.0, 47.3)])
    slots = _slots([
        ("S1", 1.5, 200.0, 28.0),
        ("S2", 0.8, 350.0, 28.0),
        ("S3", 1.1, 120.0, 28.0),
    ])
    res = plan_dome_balance(parts, slots, 0.7, 0.08, 4.5, 0.0)

    # 与暴力参考实现的全局最优逐键比对
    rows, ox, oy, n_plans = _brute_force_reference(
        parts, slots, 0.7, 0.08, 4.5
    )
    assert res.plans_evaluated == n_plans
    best = min(rows, key=lambda r: (r[0], r[1], r[2], r[3]))
    assert res.worst_magnitude == pytest.approx(best[0], abs=1e-7)
    assert res.nominal_magnitude == pytest.approx(best[1], abs=1e-7)
    assert res.total_mass == pytest.approx(best[2], abs=1e-9)
    chosen = [(w.slot_id, w.level) for w in res.weights]
    assert chosen == best[4]


def test_worst_deviation_combination_reproduces_residual():
    # 用返回的最坏偏差组合直接重算残余，应与 worst_residual 一致
    parts = _parts([(10.0, 1.0, 0.0)])
    slots = _slots([
        ("A", 10.0, 170.0, 40.0),
        ("B", 1.0, 160.0, 40.0),
        ("C", 1.0, 200.0, 40.0),
    ])
    res = plan_dome_balance(parts, slots, 1.0, 0.15, 15.0, 0.0)
    mass_sign = {"low": -1.0, "nominal": 0.0, "high": 1.0}
    az_sign = {"minus": -1.0, "nominal": 0.0, "plus": 1.0}

    rx = res.original_rx
    ry = res.original_ry
    for w, dev in zip(res.weights, res.worst_deviation):
        f = 1.0 + mass_sign[dev["mass_deviation"]] * 0.15
        a = math.radians(w.azimuth + az_sign[dev["azimuth_deviation"]] * 15.0)
        m = w.mass * f
        rx += m * w.radius * math.cos(a)
        ry += m * w.radius * math.sin(a)
    assert math.hypot(rx, ry) == pytest.approx(res.worst_magnitude, abs=1e-9)


def test_zero_tolerance_nine_scenarios_coincide():
    # ratio=0、az_tol=0 时 9 个情景完全重合；最坏=标称，
    # 最坏偏差组合归属穷举顺序首个（low/minus）
    res = plan_dome_balance(
        _parts([(10.0, 1.0, 90.0)]),
        _slots([("S1", 1.0, 270.0, 40.0)]),
        1.0, 0.0, 0.0, 0.0,
    )
    assert res.worst_magnitude == pytest.approx(res.nominal_magnitude, abs=1e-12)
    assert res.worst_deviation[0]["mass_deviation"] == "low"
    assert res.worst_deviation[0]["azimuth_deviation"] == "minus"


def test_at_most_two_slots_used():
    # 4 个相同槽位：枚举中每方案至多两个槽
    slots = _slots([
        ("S1", 1.0, 180.0, 40.0),
        ("S2", 1.0, 180.0, 40.0),
        ("S3", 1.0, 180.0, 40.0),
        ("S4", 1.0, 180.0, 40.0),
    ])
    res = plan_dome_balance(
        _parts([(10.0, 1.0, 0.0)]), slots, 1.0, 0.05, 2.0, 0.0,
    )
    assert len(res.weights) <= 2
    # 方案数 = 1 + 160 + C(160,2) - 4*C(40,2)（去掉同槽两两组合）
    expected = 1 + 160 + (160 * 159 // 2 - 4 * (40 * 39 // 2))
    assert res.plans_evaluated == expected


def test_mass_limit_not_multiple_of_step():
    # mass_limit 不是 mass_step 整数倍：floor 截断，不超档
    res = plan_dome_balance(
        _parts([(10.0, 1.0, 90.0)]),
        _slots([("S1", 1.0, 270.0, 10.4)]),
        1.0, 0.0, 0.0, 0.0,
    )
    assert [w.level for w in res.weights] == [10]
    assert res.plans_evaluated == 1 + 10


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4])
def test_random_problems_match_brute_force(seed):
    # 随机小规模对拍：生产枚举结果必须与独立暴力参考的四级择优完全一致
    rnd = random_mod.Random(seed)
    n_slots = rnd.randint(1, 4)
    step = rnd.choice([0.5, 1.0, 2.0])
    slots = []
    for i in range(n_slots):
        lim = step * rnd.randint(1, 8)  # 最多 8 档，控制暴力规模
        slots.append(BalanceSlot(
            f"S{i}",
            rnd.uniform(0.5, 3.0),
            rnd.uniform(-360.0, 360.0),
            lim,
        ))
    parts = [
        InstalledPart(
            rnd.uniform(1.0, 15.0),
            rnd.uniform(0.5, 3.0),
            rnd.uniform(-360.0, 720.0),
        )
        for _ in range(rnd.randint(1, 4))
    ]
    ratio = rnd.choice([0.0, 0.05, 0.1])
    az_tol = rnd.choice([0.0, 2.0, 5.0])

    res = plan_dome_balance(parts, slots, step, ratio, az_tol, 0.0)
    rows, _ox, _oy, n_plans = _brute_force_reference(
        parts, slots, step, ratio, az_tol
    )
    assert res.plans_evaluated == n_plans
    best = min(rows, key=lambda r: (r[0], r[1], r[2], r[3]))
    assert res.worst_magnitude == pytest.approx(best[0], abs=1e-8)
    assert res.nominal_magnitude == pytest.approx(best[1], abs=1e-8)
    assert res.total_mass == pytest.approx(best[2], abs=1e-9)
    assert [(w.slot_id, w.level) for w in res.weights] == best[4]


# ---------------------------------------------------------------------------
# HTTP API
# ---------------------------------------------------------------------------


def _payload(**overrides):
    body = {
        "installed_parts": [
            {"mass": 10.0, "radius": 1.0, "azimuth": 90.0},
        ],
        "slots": [
            {"id": "S1", "radius": 1.0, "azimuth": 270.0, "mass_limit": 40.0},
        ],
        "mass_step": 1.0,
        "mass_tolerance_ratio": 0.05,
        "azimuth_tolerance": 2.0,
        "residual_moment_limit": 1.0,
    }
    body.update(overrides)
    return body


def test_api_basic_balance():
    r = client.post("/api/v1/plan-dome-balance", json=_payload())
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["feasible"] is True
    assert body["original_moment"] == {
        "north": 0.0, "east": 10.0, "magnitude": 10.0, "azimuth": 90.0,
    }
    assert len(body["weights"]) == 1
    assert body["weights"][0]["slot_id"] == "S1"
    assert body["weights"][0]["level"] == 10
    assert body["weights"][0]["mass"] == 10.0
    assert body["nominal_residual"]["magnitude"] == pytest.approx(0.0, abs=1e-9)
    assert body["worst_residual"]["magnitude"] == pytest.approx(0.614757128, abs=1e-7)
    assert body["worst_deviation"] == [
        {"slot_id": "S1", "mass_deviation": "high", "azimuth_deviation": "plus"}
    ]
    assert body["plans_evaluated"] == 41
    assert body["scenarios_evaluated"] == 361


def test_api_infeasible_returns_200_with_flag():
    r = client.post(
        "/api/v1/plan-dome-balance",
        json=_payload(residual_moment_limit=0.0),
    )
    assert r.status_code == 200
    body = r.json()
    assert body["feasible"] is False
    assert body["worst_residual"]["magnitude"] > 0.5
    assert len(body["weights"]) == 1


def test_api_zero_counterweight_response_shape():
    payload = _payload(
        installed_parts=[
            {"mass": 5.0, "radius": 1.0, "azimuth": 0.0},
            {"mass": 5.0, "radius": 1.0, "azimuth": 180.0},
        ],
        slots=[
            {"id": "N", "radius": 1.0, "azimuth": 90.0, "mass_limit": 40.0},
            {"id": "S", "radius": 1.0, "azimuth": 270.0, "mass_limit": 40.0},
        ],
        residual_moment_limit=1.0,
    )
    r = client.post("/api/v1/plan-dome-balance", json=payload)
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["weights"] == []
    assert body["worst_deviation"] == []
    assert body["total_counterweight_mass"] == 0.0
    assert body["feasible"] is True
    assert body["worst_residual"]["magnitude"] == pytest.approx(0.0, abs=1e-9)


def test_api_weights_sorted_by_slot_id():
    payload = _payload(
        installed_parts=[{"mass": 10.0, "radius": 1.0, "azimuth": 0.0}],
        slots=[
            {"id": "C", "radius": 1.0, "azimuth": 200.0, "mass_limit": 40.0},
            {"id": "A", "radius": 10.0, "azimuth": 170.0, "mass_limit": 40.0},
            {"id": "B", "radius": 1.0, "azimuth": 160.0, "mass_limit": 40.0},
        ],
        mass_tolerance_ratio=0.15,
        azimuth_tolerance=15.0,
        residual_moment_limit=4.0,
    )
    r = client.post("/api/v1/plan-dome-balance", json=payload)
    body = r.json()
    assert [w["slot_id"] for w in body["weights"]] == ["B", "C"]
    assert [d["slot_id"] for d in body["worst_deviation"]] == ["B", "C"]


def test_api_validation_too_many_slots():
    payload = _payload(slots=[
        {"id": f"S{i}", "radius": 1.0, "azimuth": 180.0, "mass_limit": 40.0}
        for i in range(9)
    ])
    r = client.post("/api/v1/plan-dome-balance", json=payload)
    assert r.status_code == 422


def test_api_validation_more_than_40_steps():
    # 40*mass_step > mass_limit
    payload = _payload(slots=[
        {"id": "S1", "radius": 1.0, "azimuth": 270.0, "mass_limit": 41.0},
    ])
    r = client.post("/api/v1/plan-dome-balance", json=payload)
    assert r.status_code == 422
    assert "S1" in r.text


def test_api_validation_step_larger_than_limit():
    payload = _payload(slots=[
        {"id": "S1", "radius": 1.0, "azimuth": 270.0, "mass_limit": 0.5},
    ])
    r = client.post("/api/v1/plan-dome-balance", json=payload)
    assert r.status_code == 422


def test_api_validation_duplicate_slot_ids():
    payload = _payload(slots=[
        {"id": "S1", "radius": 1.0, "azimuth": 270.0, "mass_limit": 40.0},
        {"id": "S1", "radius": 1.0, "azimuth": 90.0, "mass_limit": 40.0},
    ])
    r = client.post("/api/v1/plan-dome-balance", json=payload)
    assert r.status_code == 422
    assert "唯一" in r.text


def test_api_validation_bad_ratios_and_limits():
    for field, value in [
        ("mass_tolerance_ratio", -0.1),
        ("mass_tolerance_ratio", 1.1),
        ("azimuth_tolerance", -1.0),
        ("residual_moment_limit", -1.0),
        ("mass_step", 0.0),
    ]:
        r = client.post(
            "/api/v1/plan-dome-balance", json=_payload(**{field: value})
        )
        assert r.status_code == 422, (field, value)


def test_api_validation_nonpositive_part_and_slot():
    payload = _payload(installed_parts=[
        {"mass": 0.0, "radius": 1.0, "azimuth": 90.0},
    ])
    assert client.post("/api/v1/plan-dome-balance", json=payload).status_code == 422

    payload = _payload(slots=[
        {"id": "S1", "radius": -1.0, "azimuth": 270.0, "mass_limit": 40.0},
    ])
    assert client.post("/api/v1/plan-dome-balance", json=payload).status_code == 422


def test_api_validation_empty_parts_and_slot_id():
    payload = _payload(installed_parts=[])
    assert client.post("/api/v1/plan-dome-balance", json=payload).status_code == 422

    payload = _payload(slots=[
        {"id": "", "radius": 1.0, "azimuth": 270.0, "mass_limit": 40.0},
    ])
    assert client.post("/api/v1/plan-dome-balance", json=payload).status_code == 422


def test_api_nan_rejected():
    # NaN 不是合法 JSON 数值：直接发送裸 NaN 字面量，服务端必须拒绝
    raw = (
        '{"installed_parts":[{"mass":10.0,"radius":1.0,"azimuth":90.0}],'
        '"slots":[{"id":"S1","radius":1.0,"azimuth":270.0,"mass_limit":40.0}],'
        '"mass_step":NaN,"mass_tolerance_ratio":0.05,"azimuth_tolerance":2.0,'
        '"residual_moment_limit":1.0}'
    )
    r = client.post(
        "/api/v1/plan-dome-balance",
        content=raw,
        headers={"Content-Type": "application/json"},
    )
    assert r.status_code == 422
