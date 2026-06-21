#pragma once

#include "ato_sim/types.hpp"

namespace ato {

// 停车目标制动率配置。该模块对应 AtoDriverModeControl.c 中
// AtoParkingAuthorizeToTrainStop() 的算法部分，只保留公式和门限，
// 不包含运营状态、主控模式、通信状态等业务判断。
struct ParkingBrakeConfig {
    // 普通停车目标制动率线性公式：
    // TargetAcc = AtoBrakeAccParaA * LastTargetSpd + AtoBrakeAccParaB。
    double brakeAccParaA = 0.0;
    double brakeAccParaB = 100.0;

    // 普通停车目标制动率上限，对应 AtoOriginBrakeAcc。
    double originBrakeAcc = 100.0;

    // 无人折返、库内、折返停车点使用的特殊参数。
    double arStopTargetAccB = 100.0;
    double garageBrakeAcc = 80.0;
    double reversalBrakeAccB = 100.0;

    // 雨雪和虚拟编组相关变量在原工程中来自多个模块；这里作为显式配置输入，
    // 让仿真可以独立复现制动率选择逻辑。
    double defaultBrakeAcc = 100.0;
    bool rainMode = false;
    bool rainPidParamsActive = false;
    bool rainImmediateStop = false;

    double virtualRearTurnStopDist = 0.0;
    double virtualRearCruiseToStopSbi = 0.0;
    double virtualRearTurnStopAcc = 0.0;
};

// 停车制动率计算输入。lastTargetSpeed 使用上一周期目标速度，这是原源码
// 判断是否进入精确停车触发条件的关键输入。
struct ParkingBrakeInput {
    double distanceToTarget = 0.0;
    double lastTargetSpeed = 0.0;
    double trainSpeed = 0.0;
    bool trainStopped = false;
    StopPointKind stopPointKind = StopPointKind::Normal;
    bool virtualCouplingRear = false;
    bool targetSpeedValid = true;
    bool trainSpeedValid = true;
};

// 停车制动率计算结果：
//   candidateAcc          按停车类型算出的候选制动率
//   triggerSpeed          sqrt(2 * candidateAcc * distance)
//   preciseStopTriggered  上一周期目标速度是否已超过触发速度
//   brakeAcc              供目标速度/PID 使用的最终停车制动率
struct ParkingBrakeResult {
    double brakeAcc = 0.0;
    double triggerSpeed = 0.0;
    double candidateAcc = 0.0;
    bool preciseStopTriggered = false;
};

ParkingBrakeResult calculateParkingBrake(const ParkingBrakeConfig& cfg,
                                         const ParkingBrakeInput& in);

}  // namespace ato
