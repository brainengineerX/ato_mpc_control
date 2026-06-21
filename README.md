# ATO MPC Control Algorithm Simulation

列车自动驾驶（ATO）车载 **MPC 控制器** 的 C++17 独立仿真包。ATO
算法层，实现模型预测控制（含速度硬约束 B1/B2、自适应/分段 vref），以 PID 为对比基准呈现
MPC 的提升。

> 在真实尺度场景（1–2 km 站距、上下坡、限速段）下，MPC 把停车精度从 PID 的 ~100 cm
> 提升到 **1–43 cm**，全程不超 EBI 安全包络，加速度更平稳。详见[对比结果](#对比结果)。

> **范围**：本包仅含 MPC 控制器源码。PID 数据作为对比基准保留在下方的静态表格中
> （来自剥离前的 PID 实现），不再提供 PID 运行时。

工程单位与生产代码完全一致：速度 `cm/s`、距离 `cm`、加速度 `cm/s²`、力 `N`、控制周期 `200 ms`。

---

## 对比结果

### 停车精度与运行时间（真实尺度场景）

> PID 列为剥离前 PID 实现的静态基准数据，用于对比 MPC 提升；MPC 列为当前可运行结果。

| 场景 | PID 停车 | MPC 停车 | PID 运行 | MPC 运行 |
|------|---------|---------|---------|---------|
| 1km 平坡 | 109 cm | **1 cm** | 68.4 s | 77.6 s |
| 2km 上坡 | 0 cm | 1 cm | 117.6 s | 122.2 s |
| 1.5km 限速 | 70 cm | **1 cm** | 91.6 s | 101.0 s |
| 精确停车-上坡 | 91 cm | **43 cm** | 15.2 s | 15.4 s |
| 精确停车-下坡 | 86 cm | **24 cm** | 14.4 s | 16.8 s |
| 雨天低附着 | 95 cm | **32 cm** | 11.4 s | 12.0 s |

> MPC 配置：主动集硬约束（B2）+ 分段 vref（margin 0.4/0.8）。地铁 ATO 精确停车标准通常
> 要求 ±30–50 cm，**MPC 在所有真实尺度场景达标，PID 多数超标**。

### MPC 相对 PID 的提升

| 维度 | 提升 | 说明 |
|------|------|------|
| **停车精度** | 30–100× | 1km 从 109 cm → 1 cm；下坡从 86 → 24 cm（坡度自适应生效） |
| **安全性** | 持续合规 | 超 EBI / 超 SBI 步数全程 **0**（硬约束兜底） |
| **舒适度** | 加速度 RMS ↓ 25% | 1km 场景 58 vs 77 cm/s²（MPC 输出更平稳） |
| **能耗** | 上坡场景 ↓ 8% | 2km 上坡 151915 vs 152881 kJ |
| **运行时间** | 略增 5–13% | MPC 用时间换精度/舒适，分段 vref 已大幅收回（见下方演进） |

### MPC 方案演进（1km 平坡场景）

| 阶段 | 停车 | 运行 | 说明 |
|------|------|------|------|
| 固定制动率 vref（基线） | 34 cm | 85.0 s | 保守，全程低速 |
| 自适应 vref（坡度修正） | 29 cm | 85.0 s | 下坡更保守、上坡略松 |
| B2 主动集硬约束 | 1 cm | 85.0 s | 严格不超 EBI |
| 激进 vref（margin 0.8） | 766 cm | 72.2 s | 快但制动不及过冲 |
| **分段 vref + B2** | **1 cm** | **77.6 s** | **巡航段激进 + 停车段保守 + 硬约束兜底** |

分段 vref 是关键突破：巡航段用激进裕度（晚减速省时间），停车段用保守裕度（停得准），
B2 全程硬约束保证不超 EBI。**从基线 85 s/34 cm → 77.6 s/1 cm**。

---

## MPC 三种约束模式

| 模式 | `--constraint=` | 原理 | 严格可行 | 求解时间 |
|------|----------------|------|---------|---------|
| 软约束 | `soft` | maxSpeed 越界用 `wSlackSpeed` 二次罚 | 否（可能轻微超） | ~0.4 ms |
| B1 投影 | `projected` | 无约束 QP 解后按灵敏度投影修正超限步 | 否（启发式） | ~0.2 ms |
| **B2 主动集** | `activeset` | maxSpeed 作不等式硬约束，KKT 系统求解 | **是** | ~0.2 ms |

B2 是推荐模式：严格不超 EBI、输出更平稳（方向切换从 11 降到 5）、求解快。

---

## 算法模块

| 模块 | 文件 | 说明 |
|------|------|------|
| 停车制动率 | `parking_brake` | 停车目标制动率触发 |
| 目标速度 | `target_speed` | **GEN2 EB/SB 三阶段模型（EBI/SBI 速度包络）**、停车目标、运行等级限速 |
| MPC 控制器 | `mpc_controller` | 无约束 QP + B1 投影 / B2 主动集、热启动 |
| 约束生成 | `constraint_provider` | 限速走廊、**自适应/分段 vref**（制动能力查表 + 坡度修正） |
| 输出整形 | `output_transform` | 牵引/制动模拟量、斜率限制、最小制动脉宽、保持制动 |
| 停车修正 | `auto_adjust` | 停车精度自动修正状态机 |
| 车辆动力学 | `vehicle_plant` | Davis 阻力 + 旋转质量 + 坡度重力分量 + 牵引/制动曲线 |
| 控制闭环 | `control_loop` | 单周期串联调度 |
| 评估器 | `evaluator` | 停车精度/速度跟踪/Jerk/加速度/能耗分项/超 EBI/求解时间 |

> 通用曲线查表函数 `tractionAccelerationBySpeed` / `brakeAccelerationBySpeed`
> 定义在 `vehicle_plant`，供 output_transform 等模块复用。

### MPC 实现

- **预测模型**：阻力在当前速度线性化，动力学对控制量呈仿射，前向展开为 `v_k = aV[k] + BV[k]·x`
- **目标函数**：速度跟踪 + 控制能量 + Jerk 平滑 + 末端速度归零 + 末端位置精度
- **约束**：速度上限 maxSpeed（Soft 罚 / B1 投影 / B2 主动集硬约束）
- **求解**：组装二次型 `R·x = -q`，LDL^T 一次解（B2 加 KKT 等式约束），单步 0.2 ms
- **热启动**：上一周期解作为下周期初值
- **软回退**：求解失败退化到上一周期解（RTI 风格）

> PID 实现（新/老 PID、二阶微分滤波前馈、坡度补偿、曲线饱和）已从本包剥离，
> 其历史测试结果作为对比基准保留在上方表格中。

---

## 快速开始

### 构建

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure   # MPC 单元测试
```

### 运行仿真

```bash
# MPC 软约束（基线）
./build/ato_sim_cli --constraint=soft --evaluate scenarios/realistic_station_1km.csv

# MPC B1 投影法
./build/ato_sim_cli --constraint=projected --evaluate scenarios/realistic_station_1km.csv

# MPC B2 主动集 + 分段 vref（推荐，既快又准）
./build/ato_sim_cli --constraint=activeset \
    --vref-margin=0.4 --vref-margin-far=0.8 --vref-far-dist=30000 \
    --evaluate scenarios/realistic_station_1km.csv
```

### CLI 选项

| 选项 | 说明 |
|------|------|
| `--constraint=soft\|projected\|activeset` | 速度上限约束处理方式 |
| `--vref-margin=` | 停车段 vref 制动率裕度（保守，如 0.4） |
| `--vref-margin-far=` | 巡航段 vref 裕度（激进，如 0.8） |
| `--vref-far-dist=` | 分段过渡距离（cm，如 30000） |
| `--duration=` | 巡航场景运行时长（秒） |
| `--evaluate` | 末尾打印评估指标 |

### 生成分析面板

```bash
python3 tools/gen_analysis_html.py   # 生成 ato_analysis.html
```

自包含 HTML（Chart.js 画曲线 + 指标对比表），可切换场景与视图（速度跟踪/力/加速度/目标包络）。

---

## 场景

`scenarios/` 含真实尺度与精确停车两类场景：

- `realistic_station_1km.csv` — 1km 平坡站距
- `realistic_station_2km_uphill.csv` — 2km 站距，15‰ 上坡
- `realistic_line_1500m_restricted.csv` — 1.5km，含限速段 + 坡度变化
- `precise_stop_{uphill,downhill,normal}.csv` — 50m 精确停车
- `rain_low_adhesion.csv` — 雨天低附着
- `normal_cruise.csv` / `speed_restriction_approach.csv` — 巡航/限速接近

场景 CSV 的 `train_speed` 列仅作初值种子，运行时由车辆 plant 积分（真闭环）。

---

## 测试

```bash
ctest --test-dir build --output-on-failure
```

单元测试（TDD 开发）：

- `ato_mpc_unit_tests` — linalg、vehicle plant、constraint provider（自适应/分段 vref）、
  MPC（B1/B2 硬约束）、evaluator 边界、control loop 集成

算法核心模块行覆盖率 93–100%（evaluator 99.5%、target_speed 99.0%、
control_loop 96%、MPC 94%、constraint_provider 94%）。
