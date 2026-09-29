# 天文台圆顶狭缝跟踪 API（Dome Slit Tracker）

使用 Python + FastAPI 实现：给定一条时间序列的**目标方位角**、圆顶**当前未展开
角度**、**最大转速**、**狭缝角宽**与**线缆角度区间**，计算圆顶在每个时刻应处
的未展开角度，使得：

1. **狭缝可见**：目标（允许跨 0°/360° 边界做整数圈平移）始终位于狭缝窗口内；
2. **转速约束**：相邻时刻转角不超过 `max_speed · Δt`；
3. **线缆边界**：圆顶角度始终位于 `[cable_low, cable_high]`；
4. **最优**：最小化累计转角 `Σ|d_t − d_{t−1}|`；
5. **平局规则**：累计转角相同时，返回**字典序最小**的角度序列
   （先比第 1 个时刻，再比第 2 个……）。

有解时返回完整动作时间线；无解时返回**最早失效时刻**与该时刻**未满足的约束**
（`slit_visibility` / `speed_limit` / `cable_limit`）及诊断细节。

## 一键启动（Docker Compose）

```bash
docker compose up --build
```

服务监听在宿主机 **8000** 端口（容器内同样为 8000）。

- 健康检查：<http://localhost:8000/health>
- Swagger 交互文档：<http://localhost:8000/docs>
- OpenAPI Schema：<http://localhost:8000/openapi.json>

停止：`docker compose down`。

## 接口

### GET `/health`

```bash
curl http://localhost:8000/health
# {"status":"ok","service":"dome-slit-tracker"}
```

### POST `/api/v1/track`

**请求字段**（角度单位：度；时间单位：秒）：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `target_azimuths` | number[] | 各时刻目标方位角（长度 1..5000，与 `times` 等长） |
| `times` | number[] | 各目标的绝对时刻（秒，严格递增，首项 > 0） |
| `initial_angle` | number | 圆顶当前未展开角度（可取任意实数，须在线缆区间内） |
| `max_speed` | number | 最大转速（度/秒，> 0） |
| `slit_width` | number | 狭缝角宽（度，(0, 360]；以圆顶角为中心的对称窗口） |
| `cable_low` / `cable_high` | number | 线缆允许的未展开角度区间（度） |

**示例请求**：

```bash
curl -X POST http://localhost:8000/api/v1/track \
  -H 'Content-Type: application/json' \
  -d @examples/track_request.json
```

其中 `examples/track_request.json`：

```json
{
  "target_azimuths": [0.0, 350.0, 340.0],
  "times": [1.0, 2.0, 3.0],
  "initial_angle": 0.0,
  "max_speed": 20.0,
  "slit_width": 10.0,
  "cable_low": -180.0,
  "cable_high": 180.0
}
```

**有解响应**（`feasible: true`，返回动作时间线）：

```json
{
  "feasible": true,
  "total_rotation": 15.0,
  "actions": [
    {
      "index": 1,
      "time": 1.0,
      "dome_angle": -5.0,
      "target_azimuth": 0.0,
      "lift": 0,
      "move_delta": -5.0,
      "move_speed": -5.0,
      "cable_low": -180.0,
      "cable_high": 180.0
    },
    {
      "index": 2,
      "time": 2.0,
      "dome_angle": -15.0,
      "target_azimuth": 350.0,
      "lift": -1,
      "move_delta": -10.0,
      "move_speed": -10.0,
      "cable_low": -180.0,
      "cable_high": 180.0
    },
    {
      "index": 3,
      "time": 3.0,
      "dome_angle": -15.0,
      "target_azimuth": 340.0,
      "lift": -1,
      "move_delta": 0.0,
      "move_speed": 0.0,
      "cable_low": -180.0,
      "cable_high": 180.0
    }
  ],
  "failure": null
}
```

说明：

- `dome_angle` 为该时刻选定的圆顶**未展开角度**；
- `lift` 为目标所在的整数圈，目标提升后方位为
  `target_azimuth + 360 · lift`（例如 350° 取 `lift = -1` 即 −10°，圆顶可向
  负方向小角度移动而非正转 350°）；
- `move_delta` / `move_speed` 为相对上一时刻的带符号转角 / 转速。

**无解响应**（`feasible: false`）：

```bash
curl -X POST http://localhost:8000/api/v1/track \
  -H 'Content-Type: application/json' \
  -d @examples/track_request_infeasible.json
```

```json
{
  "feasible": false,
  "total_rotation": 0.0,
  "actions": [],
  "failure": {
    "index": 2,
    "time": 2.0,
    "target_azimuth": 300.0,
    "unmet_constraints": ["speed_limit"],
    "details": {
      "candidate_windows": [{ "lift": -1, "low": -65.0, "high": -55.0 }],
      "reason": "从可达角度集合到最近狭缝窗口的最小间隙超过本步最大转角",
      "time_step_seconds": 1.0,
      "max_rotation_this_step": 10.0,
      "required_extra_rotation": 50.0,
      "required_min_speed_deg_per_s": 60.0,
      "reachable_set": [[-5.0, 20.0]],
      "nearest_window": { "lift": -1, "low": -65.0, "high": -55.0 }
    }
  }
}
```

`unmet_constraints` 取值：

- `speed_limit`：本步从可达角度集合到最近狭缝窗口的间隙超过最大转角，
  `details` 给出所需额外转角与最低转速；
- `cable_limit`：初始角度越界，或目标狭缝窗口在任意整数圈下都不与线缆区间
  相交（后者同时标注 `slit_visibility`）。

### POST `/api/v1/calibrate-sync`

用两组**各自按时间严格递增**的样本估计圆顶编码器相对参考方位的**响应时延**
与**零位偏差**：

- 参考样本（`reference_times` / `reference_angles`）；
- 圆顶编码器样本（`encoder_times` / `encoder_angles`，可与参考异步采样）。

模型：**正时延**表示编码器在时刻 `t` 的读数对应参考序列的 `t - lag` 时刻
（编码器滞后）；零位偏差定义为 `bias = reference - encoder`，即
`encoder + bias ≈ reference`。

**请求字段**（角度单位：度；时间单位：秒）：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `reference_times` | number[] | 参考样本时刻（严格递增，长度 ≥ 2） |
| `reference_angles` | number[] | 参考方位角（与 `reference_times` 等长） |
| `encoder_times` | number[] | 编码器样本时刻（严格递增，长度 ≥ 2） |
| `encoder_angles` | number[] | 编码器方位角（与 `encoder_times` 等长） |
| `lag_min` / `lag_max` | number | 时延搜索区间（秒） |
| `lag_step` | number | 时延枚举步长（秒，> 0；候选数上限 100000） |
| `outlier_threshold` | number | 离群点阈值（度，> 0） |
| `min_inliers` | integer | 有效候选所需的最少内点数（≥ 1） |

处理流程：

1. 两组角度分别以首样本为锚点，相邻角度按**最短弧**展开；最短弧恰为
   **180° 的跳变方向有歧义，直接拒绝**（HTTP 422）；
2. 对每个候选 `lag = lag_min + k·lag_step`，把编码器时刻平移为
   `t - lag`，在**重叠样本**上对参考序列线性插值，
   残差 = 插值参考角 − 展开编码器角；
3. 以残差**中位数**初估偏差，残差偏离初估超过 `outlier_threshold` 的样本
   判为离群点；再以**内点均值**重估偏差，并在内点集上计算 RMSE；
4. 内点数不足 `min_inliers` 的候选无效；
5. 选择 **RMSE 最小**的候选，并列时取 `|lag|` 较小者，再并列取 `lag`
   较小者。

**示例请求**：

```bash
curl -X POST http://localhost:8000/api/v1/calibrate-sync \
  -H 'Content-Type: application/json' \
  -d @examples/calibrate_request.json
```

**有有效候选时的响应**（偏差归一化到 `[-180, 180)`）：

```json
{
  "calibrated": true,
  "bias_deg": 29.807763,
  "lag_seconds": 0.5,
  "rmse_deg": 0.21044,
  "inlier_indices": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14],
  "outlier_indices": [],
  "reason": null,
  "details": {
    "evaluated_candidates": 41,
    "max_overlap_samples": 15,
    "max_inliers": 15,
    "min_inliers": 10,
    "outlier_threshold": 2.0
  }
}
```

`inlier_indices` / `outlier_indices` 为样本在**编码器数组中的 0 基原始
索引**（平移后落在参考时间区间之外的非重叠样本不出现在任一列表中）。

**无有效候选时**返回 HTTP 200、`calibrated: false`，并在 `reason` 说明
原因（搜索区间为空 / 所有候选均无重叠样本 / 内点数不足），`details` 给出
枚举候选数、最大重叠样本数与最大内点数：

```json
{
  "calibrated": false,
  "bias_deg": 0.0,
  "lag_seconds": 0.0,
  "rmse_deg": 0.0,
  "inlier_indices": [],
  "outlier_indices": [],
  "reason": "内点不足：41 个候选的内点数均小于 min_inliers……",
  "details": { "evaluated_candidates": 41, "max_overlap_samples": 9, "...": "..." }
}
```

### POST `/api/v1/audit-safety-interlock`

按**分段常值**重放安全联锁事件流，推导天气风险状态时间线，并审计每段
连续风险区间上的**关闭义务**履行情况（关顶命令与快门 `closed` 确认是否
在时限内完成）。

**事件类型**（`events` 数组按 `kind` 判别，时刻必须**非递减**，同刻事件
严格按输入顺序处理）：

| `kind` | 附加字段 | 说明 |
| --- | --- | --- |
| `wind` | `value` | 风速更新（≥ 0），两次更新之间分段常值保持 |
| `rain` | `value` | 雨量更新（≥ 0），同上 |
| `close_command` | — | 关顶命令（人工或自动发出） |
| `shutter` | `state` | 快门状态 `open → closing → closed → opening → open`；允许停留原态或沿循环前进一步，**全程无快门事件（缺初值）或跳态均拒绝（HTTP 422）** |

**请求字段**（时间单位：秒）：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `events` | object[] | 事件流（长度 1..5000，`time` 非递减） |
| `wind_trip` / `rain_trip` | number | 风速 / 雨量触发阈值（≥ 0） |
| `wind_reset` / `rain_reset` | number | 风速 / 雨量复位阈值（≥ 0 且 ≤ 各自触发阈值，构成迟滞） |
| `trip_duration` | number | 越限需连续保持的时长（≥ 0；0 表示当刻触发） |
| `reset_duration` | number | 复位条件需连续保持的时长（≥ 0） |
| `stale_after` | number | 距最近一次天气更新**严格超过**该时长即判中断（> 0） |
| `command_deadline` | number | 进入风险后发出关顶命令的时限（≥ 0） |
| `close_deadline` | number | 进入风险后确认快门 `closed` 的时限（≥ 0） |

天气状态机：

- 初始（收到首个天气更新前）为 `unknown`；
- 风速 ≥ `wind_trip` **或** 雨量 ≥ `rain_trip`，连续保持 `trip_duration`
  → `unsafe`；
- 风速 < `wind_reset` **且** 雨量 < `rain_reset`（两通道都已上报）连续保持
  `reset_duration` → `safe`；
- 距最近一次天气更新严格超过 `stale_after` → `unknown`，来新更新立即回到
  `safe` 并重新计时。

进入 `unsafe` / `unknown` 即产生关闭义务：须在 `command_deadline` 内发出
关顶命令、在 `close_deadline` 内确认 `closed`。义务时限在进入时刻钉死，
之后天气恢复或重复命令都不会取消或延后；`unsafe` 与 `unknown` 之间若无
`safe` 间隔，视为同一段连续风险（共享最早进入时刻的义务）。义务归属同时按
**时刻与原始事件索引**判定：同刻事件中只有**入口之后**的命令与 `closed`
可履行本次义务（`trip_duration=0` 时入口即触发天气事件本身，其前的同刻命令
不算履行），仅在入口**之前**已生效的 `closed` 才置
`already_closed_at_entry=true`；计时器在同刻事件组之前成熟（边界含相等）
时，该时刻全部事件都在入口之后，中断因要求“严格超过”回拨时则位于该时刻
全部事件之后。末个事件之后仍挂起的计时按发生时刻**投影**到时间线
（`projected: true`），开口义务的超期量截算到 `horizon`（末个事件时刻）。

**示例请求**：

```bash
curl -X POST http://localhost:8000/api/v1/audit-safety-interlock \
  -H 'Content-Type: application/json' \
  -d @examples/audit_interlock_request.json
```

**响应**（HTTP 200）：`status_timeline` 为天气状态时间线（半开区间段，
`cause_event_indices` 为触发事件的 0 基原始索引）；`risk_intervals` 为
极大连续非安全区间及其关闭义务审计（`command_due` / `close_due` 为义务
时限时刻，`command_overdue` / `close_overdue` 为超期量秒数，未履行时截算
到区间末 / `horizon`）；`close_commands` 与 `shutter_timeline` 附带原始
事件索引（`event_index`，同刻多次快门转换保留零长段以反映输入顺序）：

```json
{
  "status_timeline": [
    { "start": 0.0, "end": 1.0, "state": "unknown", "cause_event_indices": [], "projected": false },
    { "start": 1.0, "end": 8.0, "state": "safe", "cause_event_indices": [1], "projected": false },
    { "start": 8.0, "end": 25.0, "state": "unsafe", "cause_event_indices": [3, 2], "projected": false },
    { "start": 25.0, "end": 50.0, "state": "safe", "cause_event_indices": [7, 2], "projected": false },
    { "start": 50.0, "end": null, "state": "unknown", "cause_event_indices": [7, 2], "projected": true }
  ],
  "risk_intervals": [
    {
      "start": 8.0, "end": 25.0, "states": ["unsafe"],
      "entry_event_indices": [3, 2], "projected": false,
      "command_due": 68.0, "close_due": 128.0,
      "command_time": 10.0, "command_event_index": 4,
      "closed_time": 14.0, "closed_event_index": 6,
      "already_closed_at_entry": false,
      "command_overdue": 0.0, "close_overdue": 0.0
    },
    {
      "start": 50.0, "end": null, "states": ["unknown"],
      "entry_event_indices": [7, 2], "projected": true,
      "command_due": 110.0, "close_due": 170.0,
      "command_time": null, "command_event_index": null,
      "closed_time": null, "closed_event_index": null,
      "already_closed_at_entry": false,
      "command_overdue": 0.0, "close_overdue": 0.0
    }
  ],
  "close_commands": [{ "time": 10.0, "event_index": 4 }],
  "shutter_timeline": [
    { "start": 0.0, "end": 12.0, "state": "open", "event_index": 0 },
    { "start": 12.0, "end": 14.0, "state": "closing", "event_index": 5 },
    { "start": 14.0, "end": 30.0, "state": "closed", "event_index": 6 },
    { "start": 30.0, "end": 32.0, "state": "opening", "event_index": 8 },
    { "start": 32.0, "end": null, "state": "open", "event_index": 9 }
  ],
  "horizon": 32.0
}
```

（上例中初始 `unknown` 段 `[0, 1)` 同样产生一段风险区间及其义务，为
简洁从响应示例中省略。）

**HTTP 422**：事件流为空、时刻递减、阈值/时长非法（如复位阈值大于触发
阈值、`stale_after <= 0`）、未知事件类型、快门缺初值（全程无 `shutter`
事件）或跳态（未沿 `open → closing → closed → opening` 循环逐步转换）。

### POST `/api/v1/project-dome-aperture`

把望远镜**视轴前向射线**投影到**偏心球形穹顶**的内表面：给定穹顶半径、
视轴起点的 ENU 坐标与若干样本（标称方位角、高度角及二者的**对称误差**），
对每个样本的**标称视轴**与**误差矩形四角**
`(az ± δaz, el ± δel)` 共 5 条射线，分别求其与球面的**最近正根**，并汇总
五个交点的**开口包络**（最短覆盖圆弧 + 高度上下界）。

坐标系与约定：

- ENU 局部直角坐标：`east` 向东、`north` 向北、`up` 指向天顶，单位与
  `dome_radius` 相同；球心固定在穹顶地平面原点 `(0, 0, 0)`；
- 方位角**北为 0°、向东递增**（东 90°、南 180°、西 270°），可取任意实数；
  高度角自地平面起算，标称及四角高度都必须在 `[-90°, 90°]`；
- 视轴起点必须**严格位于球内**（`|origin| < dome_radius`，球面上也拒绝）；
- 穹顶地平面为 `up = 0`；五个交点中任一 `up < 0`（低于地平面）该样本非法。

**请求字段**（角度单位：度）：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `dome_radius` | number | 球形穹顶半径（> 0） |
| `origin` | object | 视轴起点 ENU 坐标 `{east, north, up}`，须严格位于球内 |
| `samples` | object[] | 样本（长度 1..5000） |
| `samples[].index` | integer | 样本在数组中的 **0 基**下标（必须与位置一致） |
| `samples[].azimuth` | number | 标称视轴方位角（度，北 0° 向东递增） |
| `samples[].elevation` | number | 标称视轴高度角（度，`[-90, 90]`） |
| `samples[].azimuth_error` | number | 方位角对称误差半宽 `δaz`（度，≥ 0） |
| `samples[].elevation_error` | number | 高度角对称误差半宽 `δel`（度，≥ 0） |

**示例请求**：

```bash
curl -X POST http://localhost:8000/api/v1/project-dome-aperture \
  -H 'Content-Type: application/json' \
  -d @examples/project_dome_aperture_request.json
```

每个样本返回：

- `nominal`：标称视轴的前向球面交点，含
  - `distance`：自起点沿前向射线到交点的距离（二次方程的最近正根）；
  - `point`：交点 ENU 坐标；
  - `azimuth` / `elevation`：交点相对**球心**的球面方位角（`[0, 360)`）
    与高度角（`[-90, 90]`）——偏心时与视线指向角并不相同；
- `corners`：误差矩形四角交点，固定顺序
  `(-δaz,-δel)`、`(+δaz,-δel)`、`(-δaz,+δel)`、`(+δaz,+δel)`，
  每项回显 `daz` / `dele` 并给出与 `nominal` 同构的交点；
- `envelope`：标称 + 四角共五个交点的开口包络：
  - `azimuth_center` / `azimuth_width`：**最短覆盖圆弧**的中心（归一化到
    `[0, 360)`）与角宽；圆弧由最大间隙法（圆周去掉最大的点间间隙）得到，
    最大间隙**并列时取 center 较小者**；
  - `azimuth_crosses_zero`：覆盖圆弧是否跨越 0°/360° 边界——跨 0° 时
    一律以 `center/width` 表示（例如点集在 350°..10° 时为
    `center=0, width=20, azimuth_crosses_zero=true`）；
  - `elevation_low` / `elevation_high`：五个交点球面高度的下/上界。

响应片段（偏心起点 `(1.5, -0.8, 2)`、R=10；有删减）：

```json
{
  "dome_radius": 10.0,
  "origin": {"east": 1.5, "north": -0.8, "up": 2.0},
  "results": [
    {
      "index": 0,
      "nominal": {
        "distance": 9.17889224,
        "point": {"east": 0.279003363, "north": 6.12461603, "up": 7.900078202},
        "azimuth": 2.608273065,
        "elevation": 52.186242314
      },
      "corners": [
        { "daz": -2.0, "dele": -1.0, "intersection": { "distance": 9.249836755, "...": "..." } }
      ],
      "envelope": {
        "azimuth_center": 2.608580923,
        "azimuth_width": 5.117263848,
        "azimuth_crosses_zero": false,
        "elevation_low": 51.055279465,
        "elevation_high": 53.317827278
      }
    }
  ]
}
```

**HTTP 422**（以下情况不产生 `results`，响应体指明原因）：

- 视轴起点不在球内（含恰在球面）：`reason = "origin_outside_sphere"`，
  并回显 `origin`、`origin_radius`、`dome_radius`；
- 标称或四角高度角越出 `[-90°, 90°]`：
  `elevation_out_of_range_indices` 给出全部相关样本的 **0 基索引**；
- 五个交点中任一低于穹顶地平面（`up < 0`）：
  `below_ground_indices` 给出全部相关样本的 **0 基索引**；
- 结构/参数非法（`dome_radius <= 0`、`samples` 为空或超长、误差为负、
  `index` 与数组下标不一致、NaN/无穷大等）同样返回 422。

数值说明：射线-球面求根采用稳定形式（`c = (|o|−R)(|o|+R)` 因式计算，
`B = o·d ≥ 0` 时正根取 `-c/(B+√(B²−c))`），在起点贴近球面的**近切线**
情形下避免相消误差。

### POST `/api/v1/plan-dome-balance`

为圆顶**检修后静平衡**规划离散槽位配重：给定已装部件的质量、旋转轴半径
与方位（质量矩 = 质量 × 半径，方位**北为 0°、向东递增**），在至多两个
候选槽位上加装离散配重，使残余质量矩最小，并穷举每个配重的质量/方位
偏差组合做验收。

约定：

- 残余质量矩 = 原始质量矩 + 各配重质量矩的**向量和**。配重装在槽位给定
  方位处、质量矩方向与槽位半径向量同向，因此要抵消原始质量矩，槽位应
  位于其**对侧**（方位相差约 180°）；
- 每个槽位的配重质量取 `mass_step` 的非负整数倍，非零档位最多 **40** 个，
  且最大非零档位质量 `40·mass_step` 不得超过该槽 `mass_limit`
  （同时要求 `mass_step ≤ mass_limit`，即每槽至少 1 个非零档位）；
- 候选槽位 1..8 个，每个方案**最多使用两个槽位**（每槽一个配重，允许
  只用一个或一个都不加）；
- 每个非零配重的质量独立取标称及 `± mass_tolerance_ratio · mass`
  （三档），方位独立取标称及 `± azimuth_tolerance`（三档），构成 9 个
  情景；两个配重再做笛卡尔积（最多 9×9=81 个情景），**全部穷举**，
  取残余模长最大者为“最坏情景”。偏差为 0 时 9 个情景退化重合，仍逐一
  计入穷举计数。

**请求字段**：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `installed_parts` | object[] | 已装部件（长度 1..5000） |
| `installed_parts[].mass` | number | 部件质量（> 0） |
| `installed_parts[].radius` | number | 部件相对旋转轴的半径（> 0） |
| `installed_parts[].azimuth` | number | 质量矩方位角（度，北 0° 向东递增，可取任意实数） |
| `slots` | object[] | 候选槽位（1..8 个，`id` 非空且唯一） |
| `slots[].id` | string | 槽位 ID |
| `slots[].radius` | number | 槽位相对旋转轴的半径（> 0） |
| `slots[].azimuth` | number | 槽位半径向量方位角（度，北 0° 向东递增，可取任意实数） |
| `slots[].mass_limit` | number | 该槽最大配重质量（> 0；要求 `mass_step ≤ mass_limit ≤ 40·mass_step`） |
| `mass_step` | number | 统一质量档位步长（> 0） |
| `mass_tolerance_ratio` | number | 质量对称相对偏差（[0, 1]）：质量取标称及 ±该比例 |
| `azimuth_tolerance` | number | 方位对称偏差半宽（度，≥ 0） |
| `residual_moment_limit` | number | 残余质量矩模长验收限值（≥ 0，质量 × 长度单位） |

择优规则（依次比较；前三项标量差值不超过 `1e-9` 视为并列）：

1. **最坏残余质量矩模长**（全部偏差情景最大值）最小；
2. **标称残余质量矩模长**最小；
3. **总配重质量**最小；
4. 按槽位 ID 排序的**全槽位**质量向量（未使用槽位记 0）字典序最小。

最坏偏差组合并列时，取按槽位 ID 顺序、先质量偏差 `(low, nominal, high)`
后方位偏差 `(minus, nominal, plus)` 的首个组合。注意：**标称最优的方案
不一定最坏最优**——大质量矩配重的偏差敏感度更高，接口按最坏残余择优。

**示例请求**：

```bash
curl -X POST http://localhost:8000/api/v1/plan-dome-balance \
  -H 'Content-Type: application/json' \
  -d @examples/plan_dome_balance_request.json
```

**响应**（HTTP 200；即使最坏残余超限也返回 200，以 `feasible` 标识是否
达标，并照常返回全部枚举中的最优方案）：

```json
{
  "feasible": true,
  "residual_moment_limit": 1.2,
  "original_moment": {
    "north": 2.954423259,
    "east": 17.479055467,
    "magnitude": 17.726984989,
    "azimuth": 80.406175127
  },
  "weights": [
    {"slot_id": "A", "level": 29, "mass": 14.5, "radius": 1.2, "azimuth": 260.0},
    {"slot_id": "C", "level": 1, "mass": 0.5, "radius": 1.0, "azimuth": 300.0}
  ],
  "total_counterweight_mass": 15.0,
  "nominal_residual": {
    "north": 0.182944968,
    "east": -0.089612137,
    "magnitude": 0.203713515,
    "azimuth": 333.902949473
  },
  "worst_residual": {
    "north": 0.733071365,
    "east": -0.705273631,
    "magnitude": 1.01725342,
    "azimuth": 316.107171512
  },
  "worst_deviation": [
    {"slot_id": "A", "mass_deviation": "high", "azimuth_deviation": "plus"},
    {"slot_id": "C", "mass_deviation": "high", "azimuth_deviation": "plus"}
  ],
  "plans_evaluated": 4921,
  "scenarios_evaluated": 389881
}
```

字段说明：

- `original_moment` / `nominal_residual` / `worst_residual` 均为
  `{north, east, magnitude, azimuth}` 二维向量（方位角归一化到
  `[0, 360)`，零向量方位记 0）；
- `weights` 与 `worst_deviation` 都按槽位 ID 升序排列；零配重方案
  （原始已平衡或任何配重都只会更差）时二者均为空列表、
  `total_counterweight_mass = 0`；
- `worst_deviation` 的 `mass_deviation` 取 `low`/`nominal`/`high`
  （−比例 / 0 / +比例），`azimuth_deviation` 取
  `minus`/`nominal`/`plus`（−容差 / 0 / +容差）；
- `plans_evaluated` 为枚举方案总数（`1 + Σ档位数 + 跨槽档位对`），
  `scenarios_evaluated` 为穷举的偏差情景总数（空方案 1、单槽 9/档、
  双槽 81/对）。8 槽 ×40 档的最大规模约枚举 4.5 万个方案、360 万个情景。

**HTTP 422**：槽位为空或超过 8 个、槽位 ID 为空或重复、`mass_step`
非正、某槽非零档位数超过 40（`40·mass_step > mass_limit`）或不足 1
（`mass_limit < mass_step`）、质量/半径非正、`mass_tolerance_ratio`
不在 `[0, 1]`、`azimuth_tolerance` 或 `residual_moment_limit` 为负、
NaN/无穷大等。

## 算法说明

角度使用**未展开（unwrapped）**表示，因此跨 0°/360° 的跟踪不需要特判：每个
时刻的可行域是若干闭区间（目标在各整数圈 `lift` 下的狭缝窗口
`[az + 360·lift − w/2, az + 360·lift + w/2]` 与线缆区间求交）。

求解器（`app/planner/`）是一个**精确的分段线性动态规划**，不做角度网格离散：

- 价值函数（从当前位置到序列末尾的最小剩余转角）对**单个连通前驱分量**
  始终连续且每段斜率属于 `{-1, 0, 1}`，用断点 + 斜率精确表示；当前一时刻
  存在多个互不连通的 lift 分量时，完整价值函数是各分量结果**逐点取 min**
  的函数（在分量恰好进入可达半径的位置可能向下跳变），因此每个时刻以
  若干 `(区间, PL)` 覆盖的形式表示，跳变点处保持分裂、绝不桥接；
- 时间转移 `g(y) = min_x |y−x| + f(x)`（受可达半径 `R = max_speed·Δt` 限制）
  通过结构固定的 4 类仿射候选片取**下包络**得到（段内截断片、左/右可达边界
  片、端点锥），见 `piecewise.py`；
- 自末时刻向前递推后，再从初始角正向重建，每一步在所有覆盖上取使
  `|y−start| + W(y)` 最小的**最左点**，从而唯一确定累计转角最小、且字典序最小
  的序列；
- 另有一次纯区间前向传播用于在无解时定位**最早失效时刻**与未满足约束。

5000 个时刻的序列在普通机器上约 2~3 秒内求解完成。

## 本地开发（不使用 Docker）

```bash
pip install -r requirements.txt
uvicorn app.main:app --reload --port 8000
```

运行测试（含对网格暴力 DP 的随机对拍，验证最优性与字典序）：

```bash
pip install pytest httpx
pytest -q
python3 scripts/fuzz_check.py
```

## 项目结构

```
app/
  main.py              # FastAPI 入口：/health、/api/v1/track、/api/v1/calibrate-sync、/api/v1/audit-safety-interlock、/api/v1/project-dome-aperture、/api/v1/plan-dome-balance
  models.py            # Pydantic 请求/响应模型与参数校验
  planner/
    geometry.py        # 方位归一化、狭缝窗口（lift）、线缆裁剪、区间合并
    piecewise.py       # 分段线性函数、距离转移算子、下包络
    solver.py          # 前向诊断 + 反向值函数 DP + 字典序路径重建
    calibration.py     # 最短弧展开 + 时延枚举/离群剔除/RMSE 同步标定
    interlock.py       # 安全联锁审计：分段常值重放、天气状态机、关闭义务
    aperture.py        # 视轴射线-偏心球面稳定求交、球面方位/高度、最短覆盖圆弧开口包络
    balance.py         # 静平衡配重：槽位档位精确枚举 + 质量/方位偏差情景穷举、四级择优
examples/              # 示例请求（跟踪有解 / 无解、同步标定、联锁审计、穹顶投影、静平衡配重）
scripts/fuzz_check.py  # 与网格暴力 DP 的随机对拍
tests/                 # 几何、求解器、标定算法、联锁审计、穹顶投影、静平衡配重、HTTP API 测试
Dockerfile
docker-compose.yml
```
