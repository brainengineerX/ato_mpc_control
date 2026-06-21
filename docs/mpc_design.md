# MPC 替换 PID 控制算法 — 技术设计文档

> 项目：`sim_control_algorithm`（ATO 控制算法仿真剥离层）
> 目标：用非线性 MPC 替换原 PID 速度跟踪器，联合优化限速、坡度、停车点、运行等级等多约束
> 验证方式：带车辆纵向动力学 Plant-in-the-Loop 仿真
> 文档版本：v0.1（初稿，待评审）

---

## 0. 背景与动机

现有 `pid_controller` + `output_transform` 的设计是经典的**串级外环 + 斜率限制**结构，存在以下结构性问题：

1. **约束处理是反应式而非预见式**：`cutTraction` / `fallingArea` / `brakeHold` 都是单周期信号，PID 在降速过程中靠反馈"打"回来，**没有预测能力**。
2. **停车段是开环公式**：停车目标速度 = `sqrt(2 * a * s)`，没有考虑停车点前 N 米的**速度曲线整形**，停车精度受 PID 跟踪误差影响。
3. **限速、坡度、停车点、运行等级各自单独处理**：`target_speed` 计算时把它们"取最小"，但下游 PID 跟踪这个最小值时**已经无法回头**重新优化。
4. **舒适度（Jerk）靠斜率限制近似**：原 `output_transform` 用 step 限制模拟量斜率，不是真正的"加加速度约束"，无法在约束紧张时主动放弃精度换取舒适度。

MPC 的核心价值：**在一个优化问题里把"约束 + 目标"统一建模**，求解时天然给出满足所有约束的最优轨迹。PID 给的是"反应"，MPC 给的是"预见"。

---

## 1. 整体架构

### 1.1 新旧对比

```
[旧]  target_speed → pid_controller → output_transform → vehicle
       目标速度算一次      PID 跟踪          斜率整形
       (取最小约束)        (反应式)         (近似舒适度)


[新]  scene/constraints → mpc_controller → command_transform → vehicle
       场景+约束建模         MPC 优化         模拟量整形
       (一次给完 N 步)     (预见式)         (保留但简化)
```

### 1.2 模块拆分

| 模块 | 角色 | 替代/保留 |
|------|------|-----------|
| `parking_brake` | 停车目标制动率公式 | **保留**（停车点附近 MPC 用作边界约束） |
| `target_speed`  | 各类约束取最小值 | **重构成约束提供器** `constraint_provider`（不再算 target，而是把约束按 N 步展开传给 MPC） |
| `pid_controller` | PID 速度跟踪 | **替换为 `mpc_controller`** |
| `output_transform` | 模拟量整形 + 斜率 + 保持制动 | **保留并简化**（MPC 输出已带 Jerk 约束，整形只做模拟量映射和保持制动） |
| `auto_adjust` | 停车精度学习 | **保留**（基于停车结果反向学习，下个停车点用） |
| `vehicle_plant` | 车辆纵向动力学 | **新增**（Plant-in-the-Loop 验证用） |

### 1.3 控制流（200ms 周期）

```
[场景层] scene_provider
   - 给出未来 N 步的限速、坡度、停车点距离、运行等级
        ↓
[约束层] constraint_provider
   - 把场景转成 MPC 用的线性约束
   - EBI 走廊：v(k) ≤ v_ebi(k)
   - SBI 走廊：v(k) ≤ v_sbi(k)
   - 运行等级：v(k) ≤ v_runlevel
   - 停车点：s(k) 终点约束
        ↓
[MPC 层] mpc_controller
   - 状态: [s, v, a_est]ᵀ
   - 输入: F_cmd (牵引/制动力)
   - 预测: H 步 (默认 30 步 = 6 秒)
   - 求解: NLP
   - 输出: 第 1 步的最优 F_cmd
        ↓
[执行层] command_transform
   - 牵引/制动百分比换算
   - 模拟量斜率限制
   - 保持制动
        ↓
[车辆层] vehicle_plant (仿真用)
   - x_{k+1} = f(x_k, u_k)
   - 反馈 s, v 给下一周期
```

---

## 2. MPC 问题建模

### 2.1 状态空间模型

列车纵向动力学（**连续时间**）：

$$
m \cdot (1 + \eta) \cdot \dot{v} = F_t - F_b - F_r(v) - m \cdot g \cdot \sin\theta
$$

其中：
- $m$ = 车重（含 AW0/AW2 加载）
- $\eta$ = 旋转质量系数（典型 0.08~0.12）
- $F_t$ = 牵引力（N）
- $F_b$ = 制动力（N）
- $F_r(v) = a_0 + a_1 v + a_2 v^2$ = Davis 阻力（N）
- $\theta$ = 坡度（rad）
- $g$ = 9.81 m/s²

为方便嵌入 MPC，**离散化**（前向欧拉，周期 T=0.2s）：

$$
s_{k+1} = s_k + v_k \cdot T
$$

$$
v_{k+1} = v_k + \frac{T}{m(1+\eta)} \cdot \left( u_k - F_r(v_k) - m g \sin\theta_k \right)
$$

其中控制量 $u_k$ = 牵引力（正值）或制动力（负值），通过符号区分。

### 2.2 决策变量

$$
\mathbf{u} = [u_0, u_1, \ldots, u_{H-1}]^T \in \mathbb{R}^{H}
$$

预测时域 H = 30 步（6 秒）。**只优化控制量，状态由模型递推**（single-shooting），降低 NLP 变量规模。

### 2.3 目标函数

$$
\min_{\mathbf{u}} \sum_{k=0}^{H-1} \left[
  w_v (v_k - v_{\text{ref},k})^2
  + w_s (s_{k} - s_{\text{ref},k})^2
  + w_u u_k^2
  + w_j (u_k - u_{k-1})^2
  + w_{\text{slack}} \epsilon
\right] + w_T \cdot (s_H - s_{\text{stop}})^2
$$

各项含义：

| 项 | 含义 | 典型权重 |
|----|------|----------|
| $w_v (v_k - v_{\text{ref},k})^2$ | 速度跟踪 | $w_v = 1.0$ |
| $w_s (s_k - s_{\text{ref},k})^2$ | 位置跟踪（停车点附近用） | $w_s = 0.01$ |
| $w_u u_k^2$ | 牵引/制动能量 | $w_u = 1e-4$ |
| $w_j (u_k - u_{k-1})^2$ | Jerk 平滑（加加速度） | $w_j = 5e-3$ |
| $w_{\text{slack}} \epsilon$ | 软约束松弛（防止 infeasible） | $w_{\text{slack}} = 1e4$ |
| $w_T (s_H - s_{\text{stop}})^2$ | 终端位置约束（停车点准确） | $w_T = 100$ |

$v_{\text{ref},k}$ 由约束层给出：巡航段 = EBI 走廊下界 + 5 km/h 裕量，停车段 = $\sqrt{2 a_{\text{brake}} (s_{\text{stop}} - s_k)}$。

### 2.4 约束

#### 硬约束

| 约束 | 表达式 | 来源 |
|------|--------|------|
| 速度上限 | $v_k \leq v_{\text{ebi},k}$ | ATP 紧急制动干预速度 |
| 速度软上限 | $v_k \leq v_{\text{sbi},k} + \epsilon$ | 常用制动干预速度 |
| 运行等级 | $v_k \leq v_{\text{runlevel}}$ | ATO 模式上限 |
| 牵引上限 | $u_k \leq F_{t,\max}(v_k)$ | 牵引曲线 |
| 制动下限 | $u_k \geq -F_{b,\max}(v_k)$ | 制动曲线 |
| 停车点 | $s_H = s_{\text{stop}}$（终端约束） | 精确停车 |

#### 软约束

| 约束 | 表达式 | 含义 |
|------|--------|------|
| 加加速度 | $\|u_k - u_{k-1}\| \leq J_{\max} \cdot T$ | 舒适度，通过权重 $w_j$ 软化 |
| 停车精度 | $\|s_H - s_{\text{stop}}\| \leq \delta$ | 防止硬终端约束 infeasible |

### 2.5 非线性来源与处理

NLP 的非线性来源：
1. **Davis 阻力 $F_r(v)$** 含 $v^2$ 项 → 状态方程非线性
2. **牵引/制动曲线** $F_{t,\max}(v)$ 是分段线性/常数 → 上界约束非线性

**方案**：用 CasADi 做符号微分 + IPOPT（SQP）求解。每个控制周期 1 次 NLP，初值用上一周期解的热启动。

---

## 3. 软件设计

### 3.1 目录结构（新增）

```
sim_control_algorithm/
├── include/ato_sim/
│   ├── mpc_controller.hpp          # 新：MPC 控制器接口
│   ├── constraint_provider.hpp     # 新：约束提供器（替代 target_speed 的部分功能）
│   ├── vehicle_plant.hpp           # 新：车辆纵向动力学接口
│   └── ...
├── src/
│   ├── mpc_controller.cpp          # 新：CasADi NLP 构建 + 求解
│   ├── constraint_provider.cpp     # 新
│   ├── vehicle_plant.cpp           # 新
│   └── ...
├── tools/
│   ├── ato_sim_cli.cpp             # 改：支持选择 PID/MPC 控制器
│   └── mpc_tune_cli.cpp            # 新：MPC 权重和约束调参工具
├── tests/
│   ├── test_mpc_controller.cpp     # 新
│   ├── test_vehicle_plant.cpp      # 新
│   ├── test_constraint_provider.cpp# 新
│   └── test_pid_vs_mpc_compare.cpp # 新：同一场景 PID vs MPC 跑分
├── scenarios/                      # 新：典型场景库
│   ├── normal_cruise.csv
│   ├── speed_restriction_approach.csv
│   ├── precise_stop_normal.csv
│   ├── precise_stop_uphill.csv
│   ├── precise_stop_downhill.csv
│   ├── rain_low_adhesion.csv
│   └── full_line_simulation.csv
└── docs/
    └── mpc_design.md               # 本文档
```

### 3.2 核心接口

```cpp
// mpc_controller.hpp
namespace ato {

struct MpcConfig {
    // 预测时域
    int horizon = 30;             // 步数
    double sampleTime = 0.2;      // s

    // 权重
    double wSpeed = 1.0;
    double wPosition = 0.01;
    double wControl = 1e-4;
    double wJerk = 5e-3;
    double wTerminal = 100.0;

    // 车辆参数
    double mass = 200000.0;        // kg
    double rotaryFactor = 0.10;    // 旋转质量系数
    double gravity = 9.81;

    // Davis 阻力参数
    double davisA0 = 2000.0;       // N
    double davisA1 = 30.0;         // N/(m/s)
    double davisA2 = 3.0;          // N/(m/s)^2

    // 牵引/制动曲线
    Curve tractionCurve;
    Curve brakeCurve;

    // 舒适度
    double maxJerk = 0.8;          // m/s^3

    // 求解器选项
    int maxIterations = 50;
    double acceptableTol = 1e-4;
};

struct MpcPrediction {
    std::vector<double> position;  // H+1 步
    std::vector<double> speed;     // H+1 步
    std::vector<double> force;     // H 步
    double cost;
    int iterations;
    double solveTimeMs;
};

class MpcController {
public:
    explicit MpcController(MpcConfig config);

    // 单步控制：传入当前状态和未来 H 步约束，求解第 1 步的最优控制
    MpcPrediction step(const VehicleState& current,
                       const ConstraintHorizon& constraints);

    void reset();
};
}  // namespace ato
```

```cpp
// vehicle_plant.hpp
namespace ato {

struct VehicleState {
    double position = 0.0;      // m
    double speed = 0.0;         // m/s
    double grade = 0.0;         // 上坡为正，rad
};

// 注意：MPC 内部用 m/s²、m/s，仿真层转换 cm/s
class VehiclePlant {
public:
    explicit VehiclePlant(VehicleConfig config);

    VehicleState step(const VehicleState& current,
                      double force,           // 牵引+ 或 制动-（N）
                      double grade,
                      double dt = 0.2);

    void reset();
};

}  // namespace ato
```

### 3.3 与现有模块的集成

**不破坏现有 PID 链路**。新增一个 `ControllerType` 枚举：

```cpp
// control_loop.hpp
enum class ControllerType {
    Pid,        // 原 PID（默认，保留对拍）
    Mpc,        // 新 MPC
};

struct ControlConfig {
    ControllerType type = ControllerType::Pid;
    ControlConfigPid pid;
    ControlConfigMpc mpc;
    // ... 其他配置
};
```

`ControlLoop::step` 内部根据 `type` 走两条路径之一。这样：
- 现有 CLI 默认仍跑 PID
- 新增 CLI flag `--controller=mpc` 切换
- 现有单元测试不破坏

### 3.4 单位约定

**MPC 内部用 SI 单位**（m, m/s, m/s², N），和现有仿真层（cm/s, cm, cm/s²）不同。

转换放在 `MpcController` 的入口/出口：

```cpp
// 输入：现有仿真层数据（cm/s, cm）
ControlInput → MpcController 内部换算成 (m/s, m) → NLP 求解
MpcController 输出 → 换算回 (cm/s²) → 给下游 OutputCommand
```

单位转换点显式标注，避免出现上次 PID 那种"两种单位混用"的隐患。

---

## 4. 求解器集成

### 4.1 选型：CasADi + IPOPT

| 维度 | CasADi + IPOPT | ACADO Toolkit | 自写 SQP |
|------|----------------|---------------|----------|
| 非线性 MPC 表达力 | ★★★★★ | ★★★★ | ★★ |
| 集成速度 | ★★★★★ | ★★★ | ★ |
| 实时性（200ms 周期） | ★★★ (单次 5-30ms) | ★★★★★ | ★★★★★ |
| 文档/社区 | ★★★★★ | ★★★ | ★ |
| 仿真剥离层适配 | ★★★★★ | ★★ | ★★★ |

**选 CasADi + IPOPT** 的理由：
- 仿真剥离层**不要求嵌入式实时**，200ms 周期对 NLP 求解器很宽裕
- CasADi 符号微分大幅降低建模出错概率
- 后期若要上车，可平滑迁移到 ACADO（结构兼容）

### 4.2 RTI 加速

用 CasADi 的 `nlpsol` 配合 `nl_init_x` / `nl_init_lam` 实现 RTI（Real-Time Iteration）：

```cpp
// 第一次迭代：完整 SQP
nlpsol("nlp", "ipopt", opts)();
// 后续周期：把上一周期的解作为初值，只做一次 SQP 迭代
nlpsol("rti", "sqpmethod", rti_opts)(x0=prev_x, lam_x0=prev_lam, lam_g0=prev_lam_g);
```

实测在标准笔记本上 RTI 单次 5-10ms，200ms 周期完全够用。

### 4.3 依赖管理

- CasADi 通过 CMake `find_package` 或 `FetchContent` 引入
- 备选：用 Docker 镜像封装，避免本地编译问题
- CI 环境：Ubuntu 22.04 + CMake 3.22 + CasADi 3.6

---

## 5. 验证方案

### 5.1 三层验证

#### 5.1.1 单元测试

| 模块 | 测试内容 |
|------|----------|
| `mpc_controller` | NLP 可解性、终端位置精度、约束满足、求解时间 |
| `vehicle_plant` | 稳态牵引/制动、最大速度、坡道停车 |
| `constraint_provider` | 多限速走廊正确性、停车点边界条件 |

#### 5.1.2 端到端测试（Plant-in-the-Loop）

```
┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐
│   Controller    │ u_k  │  Vehicle Plant  │ x_k  │   Scenario      │
│  (PID or MPC)   │─────→│  (Davis 阻力)   │─────→│  (限速/坡度)    │
│                 │←─────│                 │      │                 │
└─────────────────┘ x_k  └─────────────────┘      └─────────────────┘
```

每个场景跑两遍（PID baseline + MPC candidate），输出对齐的时间序列。

#### 5.1.3 场景库

`scenarios/` 目录下放典型场景：

| 场景 | 重点验证 |
|------|----------|
| `normal_cruise.csv` | 速度跟踪 RMSE |
| `speed_restriction_approach.csv` | 限速响应是否提前减速 |
| `precise_stop_normal.csv` | 停车误差 |
| `precise_stop_uphill.csv` | 上坡停车误差 |
| `precise_stop_downhill.csv` | 下坡停车误差（最容易超调） |
| `rain_low_adhesion.csv` | 低黏着 Jerk 表现 |
| `full_line_simulation.csv` | 综合（30+ 限速段，5+ 停车点） |

### 5.2 评价指标

| 指标 | 计算方式 | 目标 |
|------|----------|------|
| 停车精度 | 停车后 \|s - s_stop\| | MPC < 20cm（PID baseline 通常 30-50cm） |
| 速度跟踪 RMSE | $\sqrt{\frac{1}{N}\sum(v_k - v_{\text{ref},k})^2}$ | MPC < 0.3 m/s |
| Jerk RMS | $\sqrt{\frac{1}{N}\sum\left(\frac{a_{k+1}-a_k}{T}\right)^2}$ | MPC < 0.5 m/s³ |
| 牵引/制动切换次数 | 计数 | MPC < PID 50% |
| 准点率 | 终点时间误差 | 在 ±5s 内 |
| 能耗 | $\sum |u_k| \cdot v_k \cdot T$ | MPC ≤ PID |
| 单周期求解时间 | ms | < 50ms（200ms 周期内） |

### 5.3 报告输出

`tests/reports/` 下生成：
- `pid_vs_mpc_<scenario>.csv`：双控制器对齐时间序列
- `pid_vs_mpc_summary.md`：指标对比表
- `pid_vs_mpc_plots.html`（可选）：可视化曲线

---

## 6. 风险与缓解

| 风险 | 影响 | 缓解 |
|------|------|------|
| CasADi 集成/编译复杂 | 阻塞开发 | 优先用 Docker 镜像；提供 vcpkg/conan 二进制 fallback |
| NLP infeasible | 停车精度崩溃 | 软约束 + 终端松弛；fallback 退回 PID |
| 单周期求解超时 | 控制失效 | RTI 加速 + 超时回退到上周期解 |
| 权重调参是个无底洞 | 时间失控 | 先用 4 个标准场景调通，再做敏感性分析 |
| MPC 输出和原 PID 输出差异大 | 难以和原工程对拍 | 保留 PID 链路；MPC 输出仅作为候选 |

---

## 7. 实施计划

### 阶段 1：基础设施（并行 3 轨道）
- **A1 求解器集成**：CasADi 接入，示例 NLP 跑通
- **A2 车辆模型**：Davis 阻力 + 旋转质量 + 坡度
- **A3 指标体系**：评价指标 Python 脚本（pandas + matplotlib）

### 阶段 2：核心实现（并行 2 轨道）
- **B1 MPC 控制器**：约束提供器 + MPC 实现 + 单元测试
- **B2 PID baseline**：固化现有 PID 输出，作为对比基线

### 阶段 3：场景与验证
- **C1 场景库**：7 个典型场景
- **C2 对比验证**：PID vs MPC 跑分 + 报告

### 阶段 4：交付
- 集成到现有 `ato_sim_cli`
- 文档（设计 + 用户使用）
- 评审与迭代

---

## 8. 开放问题（待用户决策）

1. **预测时域 H**：建议 30 步（6 秒）。需要确认是否够用。
2. **停车精度目标**：20cm 是否合理？原 PID 是多少？
3. **Jerk 限制**：原工程没明确 Jerk 限值，建议从 0.8 m/s³ 开始。
4. **车型参数**：用通用地铁参数还是从项目 MSCP 配置拟合？
5. **MPC 失败回退策略**：直接 fallback 到 PID，还是保持上周期解不动？

---

## 9. 评审 CheckList

- [ ] 整体架构是否合理（特别是约束提供器拆出来）
- [ ] MPC 模型（Davis + 旋转质量）是否覆盖你想要的车辆特性
- [ ] 权重初值是否合理（停车精度权重会不会太大）
- [ ] 场景库是否覆盖你关心的实际场景
- [ ] 求解器选型（CasADi + IPOPT）是否接受
- [ ] 验证指标（特别是 Jerk、能耗）是否需要补充
- [ ] 单位转换点（cm/s ↔ m/s）是否同意放在 MPC 入口

---

> 文档完。审阅后我会启动 mavis-team 拆 5 个并行轨道推进。
