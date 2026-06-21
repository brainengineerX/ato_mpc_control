#include "ato_sim/output_transform.hpp"

#include "ato_sim/vehicle_plant.hpp"

#include <algorithm>
#include <cmath>

namespace ato {

OutputTransformer::OutputTransformer(OutputConfig config)
    : config_(std::move(config)), lastAnalogOutput_(config_.minAnalogOutput) {}

void OutputTransformer::reset() {
    // 重置为最小有效模拟量，而不是 0。这样首周期牵引/制动输出会从
    // minAnalogOutput 按步长爬升，贴近车辆接口的有效输出范围。
    lastAnalogOutput_ = config_.minAnalogOutput;
    lastDirection_ = ControlDirection::Coast;
    consecutiveBrakeCycles_ = 0;
}

double OutputTransformer::applyStepLimit(double demand, ControlDirection direction) {
    // 根据本周期需求值和上一周期实际输出值判断上升/下降，再选择对应步长。
    const bool rising = demand >= lastAnalogOutput_;
    double step = 0.0;

    if (direction == ControlDirection::Traction) {
        step = rising ? config_.tractionRiseStep : config_.tractionFallStep;
    } else if (direction == ControlDirection::Brake) {
        step = rising ? config_.brakeRiseStep : config_.brakeFallStep;
    } else {
        step = std::max(config_.tractionFallStep, config_.brakeFallStep);
    }

    double output = demand;
    // 普通斜率限制形式：
    //   demand 比 last 大很多 -> 只允许 last + upStep
    //   demand 比 last 小很多 -> 只允许 last - downStep
    //   差值在步长内       -> 直接输出 demand
    if (demand - lastAnalogOutput_ >= step) {
        output = lastAnalogOutput_ + step;
    } else if (lastAnalogOutput_ - demand >= step) {
        output = lastAnalogOutput_ - step;
    }

    lastAnalogOutput_ = output;
    lastDirection_ = direction;
    return output;
}

OutputCommand OutputTransformer::step(const OutputInput& in) {
    OutputCommand out;
    double demand = in.accelerationDemand;

    if ((in.atpCutTraction || in.atoCutTraction) && demand > 0.0) {
        // ATP 切牵引或 ATO 降速区切牵引生效时，牵引需求转为惰行。
        demand = 0.0;
    }

    if (demand > 0.0) {
        // 牵引：把 PID 加速度需求除以当前速度下最大牵引能力，得到牵引百分比。
        const double maxAcc = tractionAccelerationBySpeed(config_.tractionCurve, in.trainSpeed);
        out.direction = ControlDirection::Traction;
        out.tractionPercent = (maxAcc > 0.0) ? std::min(1.0, demand / maxAcc) : 0.0;
        out.analogOutput =
            config_.minAnalogOutput +
            (config_.maxAnalogOutput - config_.minAnalogOutput) * out.tractionPercent;
        out.bits.traction = true;
        consecutiveBrakeCycles_ = 0;
    } else if (demand < 0.0) {
        // 制动：负加速度需求取反后除以当前速度下最大制动能力，得到制动百分比。
        const double maxBrake = brakeAccelerationBySpeed(config_.brakeCurve, in.trainSpeed);
        out.direction = ControlDirection::Brake;
        out.brakePercent = (maxBrake > 0.0) ? std::min(1.0, -demand / maxBrake) : 0.0;
        out.analogOutput =
            config_.minAnalogOutput +
            (config_.maxAnalogOutput - config_.minAnalogOutput) * out.brakePercent;
        out.bits.brake = true;
        consecutiveBrakeCycles_++;
    } else {
        out.direction = ControlDirection::Coast;
        out.analogOutput = 0.0;

        // 最小制动脉宽保护：刚进入制动后，若 PID 下一周期立刻回到惰行/牵引，
        // 仍保持短制动输出，避免车辆侧识别到过窄制动脉冲。
        if (lastDirection_ == ControlDirection::Brake &&
            consecutiveBrakeCycles_ > 0 &&
            consecutiveBrakeCycles_ < kMinBrakeCycles) {
            out.direction = ControlDirection::Brake;
            out.bits.brake = true;
            out.analogOutput = kMinBrakeCycleOutput;
            consecutiveBrakeCycles_++;
        } else {
            consecutiveBrakeCycles_ = 0;
        }
    }

    if (config_.enableBrakeHold && in.trainSpeed <= config_.brakeHoldSpeed &&
        out.direction != ControlDirection::Traction) {
        // 低速且非牵引时施加保持制动。保持制动可以覆盖惰行或普通制动输出，
        // 但不覆盖有效牵引。
        out.direction = ControlDirection::Brake;
        out.bits.brake = true;
        out.bits.brakeHold = true;
        out.brakePercent = std::max(out.brakePercent, config_.brakeHoldPercent / 100.0);
        out.analogOutput =
            std::max(out.analogOutput,
                     config_.minAnalogOutput +
                         (config_.maxAnalogOutput - config_.minAnalogOutput) * out.brakePercent);
    }

    // 所有方向和保持制动处理完成后，最后统一施加每周期模拟量斜率限制。
    out.analogOutput = applyStepLimit(out.analogOutput, out.direction);
    return out;
}

}  // namespace ato
