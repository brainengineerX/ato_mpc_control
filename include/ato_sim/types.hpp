#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace ato {

// 本目录是从原 ATO 工程中剥离出来的算法仿真层。为了方便和原 C 源码
// 逐项对拍，这里不做单位换算，直接沿用原工程的工程单位：
//   speed        cm/s
//   distance     cm
//   acceleration cm/s^2
//   cycle time   s，默认控制周期 200 ms
//
// 也就是说，CSV 输入、算法结构体和输出结果都应使用这些单位。这样可以
// 避免仿真层和实车代码之间因为 km/h、m/s、m 等单位转换产生隐蔽偏差。
constexpr double kControlCycleSeconds = 0.2;

// 对应 CommonMacroDef.h 中 ATO_EMERGENCY_ACC。
constexpr double kEmergencyAcc = 150.0;

// PID 输出的加速度上下限，对应 PIDCtrlExtern.h 中 ATO_LIMIT_LOW/UPPER。
constexpr double kLimitLow = -109.0;
constexpr double kLimitUpper = 100.0;

// 原工程中用于表示无效距离/速度的哨兵值。仿真输入缺省时也使用这些值。
constexpr double kInvalidDistance = 99999999.0;
constexpr double kInvalidSpeed = 65535.0;

// 最小制动脉宽保护：刚进入制动后，至少保持若干周期的制动输出。
constexpr int kMinBrakeCycles = 3;
constexpr double kMinBrakeCycleOutput = 1000.0;

// ATO 驾驶阶段。当前剥离层只保留和控制算法直接相关的两个阶段：
// 巡航阶段用于速度跟踪，停车阶段用于精确停车前馈和停车修正。
enum class DrivePhase {
    Cruise,
    Stop,
};

// 控制输出方向。PID 内部输出的是加速度需求，输出整形模块再根据符号
// 转换成牵引、制动或惰行。
enum class ControlDirection {
    Coast,
    Traction,
    Brake,
};

// 停车点类型，用于选择停车目标制动率公式的 B 项或固定制动率。
enum class StopPointKind {
    Normal,
    AutoReversal,
    Garage,
    Reversal,
};

// 停车精度自动修正状态，对应 AutoAdjustModule.h 中 AJD_* 常量。
enum class AdjustState {
    Initial = 0,
    Unstable = 1,
    Stable = 2,
    Abnormal = 3,
};

// PID 三参数。这里不对参数做量纲变换，直接使用配置中的数值。
struct PidGains {
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
};

// 车辆牵引/制动能力曲线采样点：
//   speed        速度采样点，cm/s
//   acceleration 该速度点对应的最大牵引或制动加速度，cm/s^2
struct CurvePoint {
    double speed = 0.0;
    double acceleration = 0.0;
};

using Curve = std::vector<CurvePoint>;

// 限速/障碍物抽象。原工程通过 DQU 查询线路限速、MA 终点、道岔等数据；
// 剥离后不再访问 DQU，而是由仿真输入显式给出这些约束。
struct SpeedRestriction {
    // 车头到约束点的距离，cm。
    double distance = 0.0;

    // 约束点目标速度，cm/s。
    double restrictedSpeed = 0.0;

    // 该约束段使用的坡度。按原 PID 符号约定，正值表示上坡，会增加牵引需求。
    double grade = 0.0;
};

// 输出开关量的抽象表示。原工程中这些位最终被打包到 wAtoOutputValue。
struct OutputBits {
    bool traction = false;
    bool brake = false;
    bool brakeHold = false;
};

inline const char* toString(ControlDirection direction) {
    switch (direction) {
        case ControlDirection::Traction:
            return "traction";
        case ControlDirection::Brake:
            return "brake";
        case ControlDirection::Coast:
        default:
            return "coast";
    }
}

inline const char* toString(DrivePhase phase) {
    switch (phase) {
        case DrivePhase::Stop:
            return "stop";
        case DrivePhase::Cruise:
        default:
            return "cruise";
    }
}

}  // namespace ato
