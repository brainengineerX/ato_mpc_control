#include "ato_sim/parking_brake.hpp"

#include <algorithm>
#include <cmath>

namespace ato {

namespace {

// 根据停车点类型选择候选目标制动率。这里对应原函数中 TargetAcc 的
// 分支计算，先不判断是否进入精确停车，只算“如果要精停，应使用多大制动率”。
double candidateBrakeAcc(const ParkingBrakeConfig& cfg, const ParkingBrakeInput& in) {
    switch (in.stopPointKind) {
        case StopPointKind::AutoReversal:
            // 无人折返：A 参数仍乘上一周期目标速度，B 项换成 ARStopTgtAccB。
            return cfg.brakeAccParaA * in.lastTargetSpeed + cfg.arStopTargetAccB;
        case StopPointKind::Garage:
            // 库内停车：源码使用固定库内制动率。
            return cfg.garageBrakeAcc;
        case StopPointKind::Reversal:
            // 折返停车点：B 项使用折返停车常量。
            return cfg.brakeAccParaA * in.lastTargetSpeed + cfg.reversalBrakeAccB;
        case StopPointKind::Normal:
        default:
            // 普通停车：线性公式后再受 AtoOriginBrakeAcc 上限约束。
            return std::min(cfg.brakeAccParaA * in.lastTargetSpeed + cfg.brakeAccParaB,
                            cfg.originBrakeAcc);
    }
}

}  // namespace

ParkingBrakeResult calculateParkingBrake(const ParkingBrakeConfig& cfg,
                                         const ParkingBrakeInput& in) {
    ParkingBrakeResult out;

    // DTG 小于等于 0 时，原源码认为列车已经到达或越过目标点，直接进入
    // 停车制动保护，目标制动率取 ATO_EMERGENCY_ACC。
    if (in.distanceToTarget <= 0.0) {
        out.brakeAcc = kEmergencyAcc;
        out.candidateAcc = kEmergencyAcc;
        out.triggerSpeed = 0.0;
        out.preciseStopTriggered = true;
        return out;
    }

    double targetAcc = candidateBrakeAcc(cfg, in);

    // 雨雪模式下，如果已经使用雨雪 PID 参数，源码会把精确停车制动率限制到
    // 默认常用制动率以内，避免低黏着场景下目标制动率过大。
    // rainImmediateStop 表示业务侧要求立即停车，这里只抑制普通精停授权，
    // 不在算法层实现业务停车命令。
    if (cfg.rainMode && cfg.rainPidParamsActive) {
        targetAcc = std::min(targetAcc, cfg.defaultBrakeAcc);
    }

    // 虚拟编组后车：原模块会在巡航转停车距离内寻找 A1 制动率，再在低速区
    // 过渡到 A2。剥离版本把 A1 作为显式配置输入，用相同的距离窗口触发。
    if (in.virtualCouplingRear && cfg.virtualRearTurnStopDist > 0.0 &&
        in.distanceToTarget <= cfg.virtualRearTurnStopDist &&
        cfg.virtualRearTurnStopAcc > 0.0) {
        targetAcc = cfg.virtualRearTurnStopAcc;
    }

    out.candidateAcc = targetAcc;

    // 精确停车触发速度：BrakeTriggerSpd = sqrt(2 * TargetAcc * dwAtoDtg)。
    // 原源码使用“上一周期目标速度 > 触发速度”作为进入精停阶段的判断。
    out.triggerSpeed = std::sqrt(std::max(0.0, 2.0 * targetAcc * in.distanceToTarget));

    const bool validInputs = in.targetSpeedValid && in.trainSpeedValid && !in.trainStopped;
    out.preciseStopTriggered =
        validInputs && !cfg.rainImmediateStop && (in.lastTargetSpeed > out.triggerSpeed);
    out.brakeAcc = out.preciseStopTriggered ? targetAcc : cfg.defaultBrakeAcc;
    return out;
}

}  // namespace ato
