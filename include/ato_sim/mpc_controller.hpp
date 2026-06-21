#pragma once

#include "ato_sim/constraint_provider.hpp"
#include "ato_sim/sqpmethod.hpp"
#include "ato_sim/types.hpp"
#include "ato_sim/vehicle_plant.hpp"

#include <vector>

namespace ato {

// 速度上限约束的处理方式。
enum class MpcConstraintMode {
    // 软约束：maxSpeed 越界用 wSlackSpeed 二次罚（原行为）。
    Soft,
    // B1 投影法：先解无约束 QP，再对超 maxSpeed 的步做投影修正（启发式，可能轻微超）。
    Projected,
    // B2 主动集：maxSpeed 作为不等式硬约束，KKT 系统求解（严格可行）。
    ActiveSet,
};

// MPC 控制器配置。
struct MpcConfig {
    // 预测时域（步数）
    int horizon = 30;
    // 控制周期（s），与项目对齐 0.2s
    double sampleTime = 0.2;

    // 目标函数权重
    double wSpeed = 1.0;        // 速度跟踪
    double wPosition = 0.0;     // 位置跟踪（停车段时用）
    double wControl = 1e-4;     // 牵引/制动能量
    double wJerk = 5e-3;        // Jerk 平滑（相邻控制量之差）
    double wTerminal = 100.0;   // 末端位置精度
    double wStopSpeed = 50.0;   // 停车速度=0 的权重（替代硬约束）

    // 软约束违反惩罚
    double wSlackSpeed = 1e3;   // 越界速度

    // 速度上限约束处理方式（Soft/Projected/ActiveSet）。
    MpcConstraintMode constraintMode = MpcConstraintMode::Soft;
    // B1/B2 最大投影/主动集迭代次数。
    int constraintMaxIter = 20;
    // 停车 vref 自适应制动率裕度（透传给 ConstraintProvider）。
    // 0.4=保守(不超速但慢)，0.8=激进(快但需硬约束兜底)。
    double adaptiveBrakeMargin = -1.0;  // <0 表示用 ConstraintProvider 默认
    // 分段 vref：远处裕度(激进)与过渡距离，透传给 ConstraintProvider。
    double adaptiveBrakeMarginFar = -1.0;  // <0 用默认(禁用分段)
    double adaptiveBrakeFarDist = -1.0;    // <0 用默认

    // 求解器
    SqpmethodConfig solver;
    bool enableSolver = true;   // false 时退化为上一次解（用于 RTI 风格）

    // 车辆参数
    VehicleConfig vehicle;
};

// MPC 单步预测结果
struct MpcPrediction {
    std::vector<double> speed;        // H+1 步速度（cm/s）
    std::vector<double> position;     // H+1 步位置（cm）
    std::vector<double> force;        // H 步牵引/制动力（N）
    double cost = 0.0;
    bool converged = false;
    int iterations = 0;
    double solveTimeMs = 0.0;
};

class MpcController {
public:
    explicit MpcController(MpcConfig config);

    // 单步控制：传入当前状态、约束、停车点距离，输出 H 步预测和第 1 步力。
    MpcPrediction step(double currentSpeedCmps,
                       double currentPositionCm,
                       double currentGrade,
                       const std::vector<ConstraintStep>& constraints,
                       bool hasStop,
                       double stopPositionCm,
                       int horizon);

    // 软回退：求解器失败时使用上一周期解。
    void reset();

    // 暴露一些信息给上层（用于日志/调试）
    const MpcConfig& config() const { return config_; }

private:
    MpcConfig config_;
    VehiclePlant plant_;
    // 上一周期解（热启动）
    std::vector<double> lastForce_;
};

}  // namespace ato
