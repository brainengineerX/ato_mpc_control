#pragma once

#include "ato_sim/types.hpp"

namespace ato {

// 输出整形配置。该模块对应 PIDCtrl.c 中 AtoOutputTrans()、
// AtoAnaStepOutput() 和 atoBrakeHoldApply() 的算法部分。
struct OutputConfig {
    // 车辆接口模拟量范围。原工程会根据模拟量/PWM 输出方式换算。
    double minAnalogOutput = 1000.0;
    double maxAnalogOutput = 9000.0;

    // 每周期牵引/制动上升和下降步长，用于限制冲击。
    double tractionRiseStep = 500.0;
    double tractionFallStep = 500.0;
    double brakeRiseStep = 500.0;
    double brakeFallStep = 500.0;

    // 保持制动门限速度和制动百分比。
    double brakeHoldSpeed = 5.0;
    double brakeHoldPercent = 20.0;
    bool enableBrakeHold = true;

    // 用于把加速度需求转换为牵引/制动百分比的车辆能力曲线。
    Curve tractionCurve;
    Curve brakeCurve;
};

// 输出整形输入。accelerationDemand 来自 PID，cutTraction 标志来自 ATP 或目标速度模块。
struct OutputInput {
    double trainSpeed = 0.0;
    double accelerationDemand = 0.0;
    bool atpCutTraction = false;
    bool atoCutTraction = false;
};

// 输出整形结果。analogOutput 已经施加斜率限制和保持制动处理。
struct OutputCommand {
    ControlDirection direction = ControlDirection::Coast;
    OutputBits bits;
    double analogOutput = 0.0;
    double tractionPercent = 0.0;
    double brakePercent = 0.0;
};

// 输出整形器需要记忆上一周期模拟量和方向，用来实现斜率限制和最小制动脉宽。
class OutputTransformer {
public:
    explicit OutputTransformer(OutputConfig config);

    OutputCommand step(const OutputInput& in);
    void reset();

private:
    // 将期望模拟量按当前工况的上升/下降步长限制到本周期可达值。
    double applyStepLimit(double demand, ControlDirection direction);

    OutputConfig config_;
    double lastAnalogOutput_ = 0.0;
    ControlDirection lastDirection_ = ControlDirection::Coast;
    int consecutiveBrakeCycles_ = 0;
};

}  // namespace ato
