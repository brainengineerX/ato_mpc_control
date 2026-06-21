#include "ato_sim/vehicle_plant.hpp"

#include <algorithm>
#include <cmath>

namespace ato {

namespace {

double tractionAccelAt(const Curve& curve, double speedCmps) {
    if (curve.empty()) {
        return kLimitUpper;
    }
    if (speedCmps <= curve.front().speed) {
        return curve.front().acceleration;
    }
    if (curve.size() >= 2 && speedCmps <= curve[1].speed) {
        // 首采样点速度为 0 时恒功率公式 v0*a0/v 退化为 0，低速段取恒定加速度。
        if (curve.front().speed > 0.0 && speedCmps > curve.front().speed) {
            return curve.front().speed * curve.front().acceleration / speedCmps;
        }
        return curve.front().acceleration;
    }
    for (std::size_t i = 1; i < curve.size(); ++i) {
        if (speedCmps <= curve[i].speed) {
            const CurvePoint& left = curve[i - 1];
            const CurvePoint& right = curve[i];
            const double span = right.speed - left.speed;
            if (span <= 0.0) return right.acceleration;
            const double ratio = (speedCmps - left.speed) / span;
            return left.acceleration + ratio * (right.acceleration - left.acceleration);
        }
    }
    return curve.back().acceleration * 0.8;
}

double brakeAccelAt(const Curve& curve, double speedCmps) {
    if (curve.empty()) {
        return -kLimitLow;
    }
    for (const CurvePoint& point : curve) {
        if (speedCmps <= point.speed) {
            return point.acceleration;
        }
    }
    return curve.back().acceleration * 0.8;
}

}  // namespace

// 公开曲线查表（委托给上方匿名实现）。从 pid_controller 迁入，供 output_transform
// 等模块复用——与 PID 无关，是通用的车辆能力曲线查询。
double tractionAccelerationBySpeed(const Curve& curve, double trainSpeed) {
    return tractionAccelAt(curve, trainSpeed);
}

double brakeAccelerationBySpeed(const Curve& curve, double trainSpeed) {
    return brakeAccelAt(curve, trainSpeed);
}

VehiclePlant::VehiclePlant(VehicleConfig config) : config_(std::move(config)) {}

double VehiclePlant::forceFromAccelCmps2(double aCmps2) const {
    return aCmps2 * 0.01 * config_.mass * (1.0 + config_.rotaryFactor);
}

double VehiclePlant::maxTractionForce(double speedMps) const {
    const double aCmps2 = tractionAccelAt(config_.tractionCurve, speedMps * 100.0);
    return aCmps2 * 0.01 * config_.mass * (1.0 + config_.rotaryFactor);
}

double VehiclePlant::maxBrakeForce(double speedMps) const {
    const double aCmps2 = brakeAccelAt(config_.brakeCurve, speedMps * 100.0);
    return aCmps2 * 0.01 * config_.mass * (1.0 + config_.rotaryFactor);
}

VehicleStepResult VehiclePlant::step(const VehicleState& current, double force) {
    // 阻力 = a0 + a1*v + a2*v^2（N，v 单位 m/s）
    const double v = current.speed;
    const double resistance = config_.davisA0
                           + config_.davisA1 * v
                           + config_.davisA2 * v * v;
    // 重力坡度分量（N）：上坡为正 grade，需要减去 mg sinθ
    const double gradeComponent = config_.mass * config_.gravity * std::sin(current.grade);
    // 总有效净力（不含阻力与坡度）
    const double net = force - resistance - gradeComponent;
    const double effectiveMass = config_.mass * (1.0 + config_.rotaryFactor);
    const double accel = (effectiveMass > 0.0) ? net / effectiveMass : 0.0;

    VehicleStepResult out;
    out.state.position = current.position + v * config_.sampleTime
                       + 0.5 * accel * config_.sampleTime * config_.sampleTime;
    out.state.speed = std::max(0.0, v + accel * config_.sampleTime);
    out.state.grade = current.grade;
    out.state.acceleration = accel;
    out.netForce = force;
    out.resistanceForce = resistance + gradeComponent;
    return out;
}

}  // namespace ato
