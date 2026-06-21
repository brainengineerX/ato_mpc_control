#pragma once

#include "ato_sim/types.hpp"

#include <vector>

namespace ato {

// 车辆纵向动力学配置。Davis 阻力 + 旋转质量系数 + 牵引/制动曲线。
struct VehicleConfig {
    // 列车总质量（含 AW0/AW2 加载），kg。地铁 6 节编组 AW0 约 200t。
    double mass = 200000.0;

    // 旋转质量系数，无量纲，典型 0.08~0.12。
    double rotaryFactor = 0.10;

    // 重力加速度 m/s^2。
    double gravity = 9.81;

    // Davis 阻力参数：F_r = a0 + a1*v + a2*v*v（N，v 单位 m/s）。
    double davisA0 = 2000.0;
    double davisA1 = 30.0;
    double davisA2 = 3.0;

    // 牵引/制动能力曲线（cm/s, cm/s^2），与现有项目统一单位。
    Curve tractionCurve;
    Curve brakeCurve;

    // 控制周期。
    double sampleTime = 0.2;
};

// 车辆单步状态。MPC 内部用 SI 单位（m, m/s）。
struct VehicleState {
    double position = 0.0;   // m
    double speed = 0.0;      // m/s
    double grade = 0.0;      // 坡度，rad，上坡为正
    double acceleration = 0.0;  // 当前加速度，仅用于记录/观测
};

// 车辆单步结果（用于打点输出）。
struct VehicleStepResult {
    VehicleState state;
    double netForce = 0.0;       // 净力（N），牵引+ / 制动-
    double resistanceForce = 0.0;  // 阻力（N）
};

// 牵引曲线查表：低速恒加速，第一二点之间恒功率，中间线性插值，高速取末点 0.8。
// 通用曲线能力查询，供 output_transform 等模块复用。
double tractionAccelerationBySpeed(const Curve& curve, double trainSpeed);

// 制动曲线查表：按速度区间阶梯取值，高速取末点 0.8。
double brakeAccelerationBySpeed(const Curve& curve, double trainSpeed);

// 简单的纵向动力学仿真器。
// 内部使用 SI 单位。提供与仿真层（cm/s, cm）互转的便捷函数。
class VehiclePlant {
public:
    explicit VehiclePlant(VehicleConfig config);

    // 用指定净力推进一步。force: 牵引为正，制动为负。
    VehicleStepResult step(const VehicleState& current, double force);

    // 工具：把仿真层速度（cm/s）转 SI（m/s）。
    static double cmpsToMps(double vCmps) { return vCmps * 0.01; }
    // 工具：把仿真层距离（cm）转 SI（m）。
    static double cmToM(double sCm) { return sCm * 0.01; }
    // 工具：把仿真层加速度（cm/s^2）转 SI（m/s^2）。
    static double cmps2ToMps2(double aCmps2) { return aCmps2 * 0.01; }
    static double mpsToCmps(double vMps) { return vMps * 100.0; }
    static double mToCm(double sM) { return sM * 100.0; }
    static double mps2ToCmps2(double aMps2) { return aMps2 * 100.0; }

    // 工具：把工程加速度（cm/s^2）换算成 SI 力（N）。用于从 MPC 输出的加速度指令反推 force。
    // 仅用于参考，MPC 内部直接输出力。
    double forceFromAccelCmps2(double aCmps2) const;

    // 工具：当前速度下最大牵引力 / 制动力的绝对值。
    double maxTractionForce(double speedMps) const;
    double maxBrakeForce(double speedMps) const;

    const VehicleConfig& config() const { return config_; }

private:
    VehicleConfig config_;
};

}  // namespace ato
