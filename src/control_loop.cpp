#include "ato_sim/control_loop.hpp"

#include "ato_sim/vehicle_plant.hpp"

#include <algorithm>
#include <cmath>

namespace ato {

ControlLoop::ControlLoop(ControlConfig config)
    : config_(std::move(config)),
      output_(config_.output),
      autoAdjust_(config_.autoAdjust),
      mpc_(config_.mpc),
      plant_(new VehiclePlant(config_.mpc.vehicle)) {}

void ControlLoop::reset() {
    // 重置所有跨周期状态。目标速度算法本身是纯函数，但停车制动率需要上一周期
    // 目标速度，输出整形/停车修正/MPC 热启动也都有状态。
    output_.reset();
    mpc_.reset();
    lastTargetSpeed_ = 0.0;
    simSpeed_ = 0.0;
    simPosition_ = 0.0;
}

ControlOutput ControlLoop::step(const ControlInput& in) {
    ControlOutput out;

    // 在 plant 仿真开启时，用 plant 推进状态——MPC 必须用真实的当前状态。
    // 不开启时，完全用 in 字段。
    ControlInput inEffective = in;
    if (plantEnabled_ && plant_ != nullptr) {
        inEffective.trainSpeed = simSpeed_;
        lastGrade_ = in.grade;
    }

    // 1. 停车目标制动率。这里使用上一周期目标速度 lastTargetSpeed_。
    ParkingBrakeInput parkingInput;
    parkingInput.distanceToTarget = inEffective.distanceToStop;
    parkingInput.lastTargetSpeed = lastTargetSpeed_;
    parkingInput.trainSpeed = inEffective.trainSpeed;
    parkingInput.trainStopped = inEffective.trainStopped;
    out.parking = calculateParkingBrake(config_.parking, parkingInput);

    // 2. 读取该停车点上一次停车精度修正值，并作为目标速度停车距离修正。
    const int stopAdjust = autoAdjust_.adjustValueFor(inEffective.stopPointId);

    // 3. 目标速度计算。线路、MA、限速等原 DQU 查询数据都由 restrictions 输入。
    TargetSpeedInput targetInput;
    targetInput.trainSpeed = inEffective.trainSpeed;
    targetInput.ebiSpeed = inEffective.ebiSpeed;
    targetInput.runLevelSpeed = inEffective.runLevelSpeed;
    targetInput.stopDistance = inEffective.distanceToStop;
    targetInput.stopDistanceAdjust = static_cast<double>(stopAdjust);
    targetInput.parkingBrakeAcc = out.parking.brakeAcc;
    targetInput.restrictions = inEffective.restrictions;
    out.target = calculateTargetSpeed(config_.target, config_.ebSb, targetInput);
    out.targetSpeed = out.target.targetSpeed;

    // 4. MPC 路径：构造约束、求解、输出。
    ConstraintConfig ccfg;
    ccfg.runLevelSpeed = inEffective.runLevelSpeed;
    ccfg.maxLineLimitSpeed = config_.target.maxLineLimitSpeed;
    ccfg.minMarginFromEbi = config_.target.minMarginFromEbi;
    ccfg.ebSb = config_.ebSb;
    ccfg.restrictions = inEffective.restrictions;
    ccfg.stopDistanceAdjust = static_cast<double>(stopAdjust);
    ccfg.maAdjustmentDistance = config_.target.maAdjustmentDistance;
    // 自适应 vref：传入车辆制动曲线与当前坡度，使参考减速曲线按实际制动
    // 能力反推(方案 A)，远处敢跑、临近停车收紧。
    ccfg.brakeCurve = config_.mpc.vehicle.brakeCurve;
    ccfg.grade = inEffective.grade;
    if (config_.mpc.adaptiveBrakeMargin > 0.0) {
        ccfg.adaptiveBrakeMargin = config_.mpc.adaptiveBrakeMargin;
    }
    if (config_.mpc.adaptiveBrakeMarginFar > 0.0) {
        ccfg.adaptiveBrakeMarginFar = config_.mpc.adaptiveBrakeMarginFar;
    }
    if (config_.mpc.adaptiveBrakeFarDist > 0.0) {
        ccfg.adaptiveBrakeFarDist = config_.mpc.adaptiveBrakeFarDist;
    }

    const auto constraints = ConstraintProvider::generate(
        ccfg, inEffective.trainSpeed, inEffective.currentPositionCm,
        inEffective.distanceToStop, config_.mpc.horizon);

    const bool hasStop = inEffective.distanceToStop > 0.0;
    // 位置基准统一用调用方传入的 currentPositionCm（闭环仿真由 CLI 维护），
    // 不混用内部 simPosition_——后者仅在 enablePlant() 时更新，未启用时
    // 恒为 0，会导致 MPC 约束生成与求解器位置基准不一致。
    const double stopPos = inEffective.currentPositionCm + inEffective.distanceToStop;

    out.mpc = mpc_.step(inEffective.trainSpeed, inEffective.currentPositionCm,
                        inEffective.grade, constraints,
                        hasStop, stopPos, config_.mpc.horizon);

    // 5. 把 MPC 第 1 步的力（如果可用）转成加速度需求。
    if (!out.mpc.force.empty()) {
        out.appliedForce = out.mpc.force.front();
        const double effMass = config_.mpc.vehicle.mass
                             * (1.0 + config_.mpc.vehicle.rotaryFactor);
        const double aMps2 = (effMass > 0.0) ? out.appliedForce / effMass : 0.0;
        const double aCmps2 = aMps2 * 100.0;
        const double aSat = std::max(kLimitLow,
                                     std::min(kLimitUpper, aCmps2));
        OutputInput outputInput;
        outputInput.trainSpeed = inEffective.trainSpeed;
        outputInput.accelerationDemand = aSat;
        outputInput.atoCutTraction = out.target.cutTraction;
        out.command = output_.step(outputInput);
    } else {
        out.appliedForce = 0.0;
        OutputInput outputInput;
        outputInput.trainSpeed = inEffective.trainSpeed;
        outputInput.accelerationDemand = 0.0;
        out.command = output_.step(outputInput);
    }

    // 6. 停车精度自动修正。
    AutoAdjustInput autoInput;
    autoInput.stopPointId = inEffective.stopPointId;
    autoInput.drivePhase = inEffective.phase;
    autoInput.trainStopped = inEffective.trainStopped;
    autoInput.distanceToTarget = static_cast<int>(inEffective.distanceToStop);
    out.autoAdjust = autoAdjust_.step(autoInput);

    // 7. Plant 仿真：用 MPC 输出的力推进一步。
    if (plantEnabled_ && plant_ != nullptr) {
        VehicleState vs;
        vs.position = simPosition_ * 0.01;  // cm -> m
        vs.speed = simSpeed_ * 0.01;
        // grade 在工程层为千分度(‰)，VehiclePlant 期望弧度(rad)，在此换算。
        vs.grade = in.grade * 1e-3;
        const double force = out.appliedForce;
        const auto step = plant_->step(vs, force);
        simSpeed_ = step.state.speed * 100.0;
        simPosition_ = step.state.position * 100.0;
        out.simulatedSpeed = simSpeed_;
        out.simulatedPosition = simPosition_;
    }

    // 更新上一周期目标速度
    lastTargetSpeed_ = out.target.targetSpeed;
    return out;
}

}  // namespace ato
