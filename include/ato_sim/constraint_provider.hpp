#pragma once

#include "ato_sim/target_speed.hpp"
#include "ato_sim/types.hpp"

#include <vector>

namespace ato {

// 单步约束：MPC 在预测时域内每个时刻的"边界"信息。
struct ConstraintStep {
    double maxSpeed = 0.0;       // 该步允许的最大速度（cm/s）
    double minSpeed = 0.0;       // 允许的最小速度（cm/s），通常 0
    double targetSpeed = 0.0;    // 速度参考（cm/s），用于目标函数跟踪
    double positionRef = 0.0;    // 位置参考（cm），停车段用
    bool isStopStep = false;     // 是否到达停车点（仅末端可能为 true）
    bool softSpeedLimit = false; // 超过 maxSpeed 时是否软约束（true=软）
};

// 整段预测时域的约束序列。
struct ConstraintHorizon {
    std::vector<ConstraintStep> steps;
    // 停车点：若启用停车段，MPC 会在末端施加位置约束。
    bool hasStop = false;
    double stopPosition = 0.0;   // cm
    double stopBrakeAcc = 0.0;   // cm/s^2，用于停车目标速度
};

// 配置：决定走廊怎么算。
struct ConstraintConfig {
    // 运行等级（cm/s），若 <=0 表示不限制。
    double runLevelSpeed = 0.0;
    // 线路最大限速（cm/s）。
    double maxLineLimitSpeed = 2200.0;
    // EBI 裕量（cm/s），目标速度会比 EBI 低这个数。
    double minMarginFromEbi = 55.0;
    // 软约束裕量（cm/s），越过 SBI 不立刻判定 infeasible。
    double softMarginFromSbi = 30.0;
    // GEN2 EB/SB 模型参数。仿真剥离层不依赖 DQU，由调用方预先算好每步限速走廊。
    EbSbConfig ebSb;

    // 限速点列表（多限速），用于生成限速走廊。
    std::vector<SpeedRestriction> restrictions;

    // 自动修正停车距离偏移（cm）。
    double stopDistanceAdjust = 0.0;
    // MA 调整距离（cm）。
    double maAdjustmentDistance = 0.0;

    // 停车段自适应 vref 用的车辆制动能力曲线（cm/s, cm/s^2）与当前坡度(‰)。
    // 制动曲线为空时退化为固定保守制动率 0.35×EBI（兼容旧行为）。
    Curve brakeCurve;
    double grade = 0.0;
    // 自适应制动率相对实际制动能力的裕度系数。取低于 1 的值给 MPC 跟踪误差、
    // 阻力线性化误差、坡度扰动留裕度。默认 0.4 使等效制动率接近原固定保守值
    // 0.35×EBI（实际制动能力约 95-110，0.4×≈40-44），保证不制动不及过冲；
    // 同时下坡自动更保守、上坡略松。
    double adaptiveBrakeMargin = 0.4;
    // 自适应制动率下限（cm/s^2），防止极端下坡 a→0 使曲线退化。
    double adaptiveBrakeFloor = 0.0;  // 0 表示按 0.3×EBI 自动取

    // 分段 vref：远处(距停车点远)用更激进的制动率裕度，临近停车用保守值。
    // margin 在 distToStop 从 adaptiveBrakeFarDist(远) 线性过渡到 0(近)，
    // 从 adaptiveBrakeMarginFar 降到 adaptiveBrakeMargin。这样巡航段敢晚减速
    // (省时间)，停车段保守制动(停得准)。
    // adaptiveBrakeMarginFar <= adaptiveBrakeMargin 时禁用分段(全程用单一值)。
    double adaptiveBrakeMarginFar = 0.0;  // 0 表示禁用分段
    double adaptiveBrakeFarDist = 30000.0;  // cm，过渡距离(典型 300m)
};

// 把仿真层的限制 + 状态 转换成一串 H+1 步约束。
// 内部使用 SI 单位，最后转回工程单位。
class ConstraintProvider {
public:
    // 构造时确定预测时域。每调用一次 generate 产出 H+1 步约束。
    static std::vector<ConstraintStep> generate(
        const ConstraintConfig& cfg,
        double currentSpeedCmps,
        double currentPositionCm,
        double stopDistanceCm,
        int horizon);

private:
    static double gen2EbSbSpeedCmps(double distanceCm, double grade,
                                    double restrictedSpeedCmps,
                                    const EbSbConfig& cfg,
                                    bool useBrakeAcc);
};

}  // namespace ato
