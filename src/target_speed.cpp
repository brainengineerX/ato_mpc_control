#include "ato_sim/target_speed.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ato {

namespace {

// 求二次方程 a*x^2 + b*x + c = 0 的正根。GEN2 三阶段模型最终会化成
// 该形式，正根即当前约束下允许的 EB/SB 速度。
double positiveQuadraticRoot(double a, double b, double c) {
    const double discriminant = b * b - 4.0 * a * c;
    if (a == 0.0 || discriminant < 0.0) {
        return 0.0;
    }
    return (-b + std::sqrt(discriminant)) / (2.0 * a);
}

// 速度和距离在仿真层不允许出现负输出。源码里很多中间量使用无符号类型，
// 剥离后使用 double，因此在边界处显式钳到 0。
double clampNonNegative(double value) {
    return std::max(0.0, value);
}

}  // namespace

double calculateGen2RestrictedSpeed(const EbSbConfig& cfg,
                                    double distance,
                                    double grade,
                                    double restrictedSpeed) {
    const double dist = clampNonNegative(distance);
    const double gradeValue = (cfg.trainRotateRate == 0.0) ? grade : grade / cfg.trainRotateRate;

    // 对应 FontEbSbCalcBaseGEN2() 的三阶段运动学模型：
    //   1. 牵引切除延时 motorDelay 内，列车仍按牵引加速度 a1 运动；
    //   2. 制动建立延时 brakeDelay 内，按坡度等效加速度 a2 过渡；
    //   3. 制动建立完成后，以 a3 制动到目标限速 restrictedSpeed。
    //
    // 坡度符号沿用原工程：上坡为正，会降低牵引阶段等效加速度，
    // 同时增加制动阶段等效减速度。
    const double a1 = cfg.motorAcc - gradeValue;
    const double a2 = gradeValue;
    const double a3 = cfg.brakeAcc + gradeValue;

    if (a3 <= 0.0) {
        return restrictedSpeed;
    }

    const double temp = a1 * cfg.motorDelay - a2 * cfg.brakeDelay;
    const double alpha = cfg.motorDelay + cfg.brakeDelay + temp / a3;
    const double beta = 0.5 * a1 * cfg.motorDelay * cfg.motorDelay +
                        a1 * cfg.motorDelay * cfg.brakeDelay -
                        0.5 * a2 * cfg.brakeDelay * cfg.brakeDelay +
                        0.5 * temp * temp / a3;
    const double lValue =
        (restrictedSpeed * restrictedSpeed + 2.0 * a3 * dist) / (2.0 * a3);
    const double threeStageSpeed = positiveQuadraticRoot(0.5 / a3, alpha, beta - lValue);

    // 源码还会计算一个两阶段速度，并和三阶段速度取较大值。这样在距离较短、
    // 限速点很近时，牵引切除和制动建立延时不会被低估。
    double twoStageSpeed = 0.0;
    if (gradeValue >= 0.0) {
        twoStageSpeed = restrictedSpeed - a1 * cfg.motorDelay;
    } else {
        twoStageSpeed = restrictedSpeed + a2 * cfg.brakeDelay - a1 * cfg.motorDelay;
    }

    return std::max({restrictedSpeed, threeStageSpeed, twoStageSpeed});
}

EbSbResult calculateEbSbSpeed(const EbSbConfig& ebCfg,
                              const TargetSpeedConfig& targetCfg,
                              const std::vector<SpeedRestriction>& restrictions) {
    EbSbResult out;
    out.ebiSpeed = targetCfg.maxLineLimitSpeed;
    out.sbiSpeed = targetCfg.maxLineLimitSpeed;

    if (restrictions.empty()) {
        // 没有限速/障碍物输入时，等价于只受线路最大限速约束。
        return out;
    }

    for (const SpeedRestriction& restriction : restrictions) {
        if (restriction.distance < 0.0) {
            // 约束点已经在车身范围内或车头后方时，不再计算制动曲线，
            // 直接用该限制速度参与取小。
            out.ebiSpeed = std::min(out.ebiSpeed, restriction.restrictedSpeed);
            out.sbiSpeed = std::min(out.sbiSpeed, restriction.restrictedSpeed);
            continue;
        }

        // EBI 使用 ebCfg.brakeAcc，通常对应最不利紧急制动率。
        const double ebi = calculateGen2RestrictedSpeed(
            ebCfg, restriction.distance, restriction.grade, restriction.restrictedSpeed);
        out.ebiSpeed = std::min(out.ebiSpeed, ebi);

        // SBI 不是简单 EBI 减常数，而是先对限速点做距离/速度偏移，再用
        // 常用制动率重新计算曲线。
        SpeedRestriction sbiRestriction = restriction;
        sbiRestriction.distance -=
            restriction.restrictedSpeed * targetCfg.sbiTimeOffset + targetCfg.sbiDistanceOffset;
        sbiRestriction.restrictedSpeed =
            std::max(0.0, restriction.restrictedSpeed - targetCfg.sbiSpeedOffset);

        EbSbConfig sbiCfg = ebCfg;
        sbiCfg.brakeAcc = targetCfg.defaultBrakeAcc;
        const double sbi = calculateGen2RestrictedSpeed(
            sbiCfg,
            sbiRestriction.distance,
            sbiRestriction.grade,
            sbiRestriction.restrictedSpeed);
        out.sbiSpeed = std::min(out.sbiSpeed, sbi);
    }

    out.ebiSpeed = clampNonNegative(out.ebiSpeed);
    out.sbiSpeed = clampNonNegative(out.sbiSpeed);
    return out;
}

StopSpeedResult calculateStopTargetSpeed(const TargetSpeedConfig& cfg,
                                         const TargetSpeedInput& in) {
    StopSpeedResult out;
    out.displacement = in.stopDistance + cfg.maAdjustmentDistance + in.stopDistanceAdjust;

    if (out.displacement <= 0.0 || in.parkingBrakeAcc <= 0.0) {
        // 已到达/越过停车点，或制动率无效时，停车目标速度直接为 0。
        out.rawStopSpeed = 0.0;
        out.targetSpeed = 0.0;
        return out;
    }

    // 精确停车基本公式：V = sqrt(2 * a * s)。
    out.rawStopSpeed = std::sqrt(2.0 * in.parkingBrakeAcc * out.displacement);
    out.targetSpeed = out.rawStopSpeed;

    // 再按 ATP/EBI 裕量限幅，保证 ATO 目标速度低于 EBI。
    if (in.ebiSpeed <= cfg.minMarginFromEbi) {
        out.targetSpeed = 0.0;
    } else {
        out.targetSpeed = std::min(out.targetSpeed, in.ebiSpeed - cfg.minMarginFromEbi);
    }
    return out;
}

TargetSpeedResult calculateTargetSpeed(const TargetSpeedConfig& cfg,
                                       const EbSbConfig& ebCfg,
                                       const TargetSpeedInput& in) {
    TargetSpeedResult out;

    if (in.openLoopFallback) {
        // 开口/FRM 等降级场景不走完整曲线，直接使用 EBI - margin。
        out.targetSpeed =
            (in.ebiSpeed > cfg.minMarginFromEbi) ? in.ebiSpeed - cfg.minMarginFromEbi : 0.0;
        out.curMaxSbv = out.targetSpeed;
        out.stopTargetSpeed = out.targetSpeed;
        out.sbiSpeed = out.targetSpeed;
        return out;
    }

    const EbSbResult ebSb = calculateEbSbSpeed(ebCfg, cfg, in.restrictions);
    const double runLevel =
        (in.runLevelSpeed > 0.0 && in.runLevelSpeed <= cfg.maxLineLimitSpeed)
            ? in.runLevelSpeed
            : cfg.maxLineLimitSpeed;

    // CurMaxSbv 在原工程中是不考虑运行等级前的最高推荐速度，PID/ECO 会用到。
    out.sbiSpeed = ebSb.sbiSpeed;
    out.curMaxSbv = std::min(ebSb.sbiSpeed, cfg.maxLineLimitSpeed);

    // 停车目标速度使用更小的 EBI：外部输入 EBI 与本模块计算 EBI 取小。
    TargetSpeedInput stopInput = in;
    stopInput.ebiSpeed = (in.ebiSpeed > 0.0) ? std::min(in.ebiSpeed, ebSb.ebiSpeed) : ebSb.ebiSpeed;
    const StopSpeedResult stop = calculateStopTargetSpeed(cfg, stopInput);
    out.stopTargetSpeed = stop.targetSpeed;

    // 旧目标速度分支的核心思想：所有约束取小。这里保留巡航/SBI、运行等级、
    // 精确停车三类约束；虚拟 MA 等约束可通过 restrictions 扩展输入。
    out.targetSpeed = std::min({out.curMaxSbv, runLevel, stop.targetSpeed});

    // 完整源码的降速区识别有三周期计数和未来目标速度预测。剥离层先暴露
    // 一个单周期等价信号：目标速度低于当前速度时认为存在降速趋势。
    out.fallingArea = out.targetSpeed < in.trainSpeed;
    out.cutTraction = out.fallingArea && out.targetSpeed > 0.0 && in.trainSpeed > out.targetSpeed;
    return out;
}

}  // namespace ato
