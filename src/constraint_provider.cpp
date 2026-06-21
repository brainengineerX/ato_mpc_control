#include "ato_sim/constraint_provider.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ato {

namespace {

double positiveQuadraticRoot(double a, double b, double c) {
    const double disc = b * b - 4.0 * a * c;
    if (a == 0.0 || disc < 0.0) {
        return 0.0;
    }
    return (-b + std::sqrt(disc)) / (2.0 * a);
}

double clampNonNeg(double v) { return std::max(0.0, v); }

// 制动能力查表（与 vehicle_plant.cpp 的 brakeAccelAt 同语义）：
// 按速度区间阶梯取值，高于末点取末点 0.8 倍。返回 cm/s^2。
double brakeAccelBySpeed(const Curve& curve, double speedCmps) {
    if (curve.empty()) {
        return 0.0;
    }
    for (const CurvePoint& point : curve) {
        if (speedCmps <= point.speed) {
            return point.acceleration;
        }
    }
    return curve.back().acceleration * 0.8;
}

}  // namespace

double ConstraintProvider::gen2EbSbSpeedCmps(double distanceCm, double grade,
                                             double restrictedSpeedCmps,
                                             const EbSbConfig& cfg,
                                             bool useBrakeAcc) {
    const double dist = clampNonNeg(distanceCm);
    const double gradeValue = (cfg.trainRotateRate == 0.0)
                                  ? grade
                                  : grade / cfg.trainRotateRate;
    const double a1 = cfg.motorAcc - gradeValue;
    const double a2 = gradeValue;
    const double a3 = (useBrakeAcc ? cfg.brakeAcc : cfg.brakeAcc) + gradeValue;
    if (a3 <= 0.0) {
        return restrictedSpeedCmps;
    }
    const double temp = a1 * cfg.motorDelay - a2 * cfg.brakeDelay;
    const double alpha = cfg.motorDelay + cfg.brakeDelay + temp / a3;
    const double beta = 0.5 * a1 * cfg.motorDelay * cfg.motorDelay
                      + a1 * cfg.motorDelay * cfg.brakeDelay
                      - 0.5 * a2 * cfg.brakeDelay * cfg.brakeDelay
                      + 0.5 * temp * temp / a3;
    const double lValue =
        (restrictedSpeedCmps * restrictedSpeedCmps + 2.0 * a3 * dist) / (2.0 * a3);
    const double threeStage =
        positiveQuadraticRoot(0.5 / a3, alpha, beta - lValue);
    double twoStage = 0.0;
    if (gradeValue >= 0.0) {
        twoStage = restrictedSpeedCmps - a1 * cfg.motorDelay;
    } else {
        twoStage = restrictedSpeedCmps + a2 * cfg.brakeDelay - a1 * cfg.motorDelay;
    }
    return std::max({restrictedSpeedCmps, threeStage, twoStage});
}

std::vector<ConstraintStep> ConstraintProvider::generate(
    const ConstraintConfig& cfg,
    double currentSpeedCmps,
    double currentPositionCm,
    double stopDistanceCm,
    int horizon) {
    std::vector<ConstraintStep> steps(horizon + 1);

    // 1) 计算每步位置（cm）— 假设匀速预测，作为约束评估的位置。
    for (int k = 0; k <= horizon; ++k) {
        steps[k].positionRef = currentPositionCm + currentSpeedCmps * 0.2 * k;
    }

    // 2) 限速走廊：取 EBI / SBI 最小值。
    double ebiSpeed = cfg.maxLineLimitSpeed;
    double sbiSpeed = cfg.maxLineLimitSpeed;

    for (const SpeedRestriction& r : cfg.restrictions) {
        if (r.distance < 0.0) {
            ebiSpeed = std::min(ebiSpeed, r.restrictedSpeed);
            sbiSpeed = std::min(sbiSpeed, r.restrictedSpeed);
            continue;
        }
        // EBI 用 cfg.ebSb.brakeAcc（紧急制动率）。
        const double ebi = gen2EbSbSpeedCmps(
            r.distance, r.grade, r.restrictedSpeed, cfg.ebSb, true);
        ebiSpeed = std::min(ebiSpeed, ebi);

        // SBI 偏移：和 target_speed 同样的处理（距离/速度偏移后再算）。
        const double sbiDist = std::max(0.0, r.distance
            - r.restrictedSpeed * 0.5 - 500.0);  // 简化：常数偏移
        const double sbiSpd = std::max(0.0, r.restrictedSpeed - 27.0);
        const double sbi = gen2EbSbSpeedCmps(
            sbiDist, r.grade, sbiSpd, cfg.ebSb, true);
        sbiSpeed = std::min(sbiSpeed, sbi);
    }
    ebiSpeed = clampNonNeg(ebiSpeed);
    sbiSpeed = clampNonNeg(sbiSpeed);

    // 3) 停车段：若有停车点，从停车点向回反推每步自适应参考速度。
    bool hasStop = (stopDistanceCm > 0.0);
    double stopPos = 0.0;
    if (hasStop) {
        stopPos = currentPositionCm + stopDistanceCm
                + cfg.maAdjustmentDistance + cfg.stopDistanceAdjust;
    }

    // 4) 先填每步 maxSpeed（限速走廊上界），vref 反推要用到。
    double vEbi = ebiSpeed - cfg.minMarginFromEbi;
    if (vEbi < 0.0) vEbi = 0.0;
    const double runLevel = (cfg.runLevelSpeed > 0.0
                             && cfg.runLevelSpeed <= cfg.maxLineLimitSpeed)
                            ? cfg.runLevelSpeed
                            : cfg.maxLineLimitSpeed;
    for (int k = 0; k <= horizon; ++k) {
        double vMax = std::min(vEbi, runLevel);
        if (vMax < 0.0) vMax = 0.0;
        steps[k].maxSpeed = vMax;
        steps[k].minSpeed = 0.0;
        steps[k].softSpeedLimit = true;
        steps[k].targetSpeed = vMax;  // 默认取 maxSpeed，停车段下面覆盖
        steps[k].isStopStep = false;
    }

    // 5) 停车段自适应 vref：从停车点向回递推。
    //    vref[k] = sqrt(vref[k+1]^2 + 2 * a(vref[k+1]) * dist_k)
    //    a 用"下一步参考速度对应的实际制动能力"×裕度，并扣除坡度分量。
    //    无制动曲线时退化为固定保守值 0.35×EBI。
    std::vector<double> vRefAtK(static_cast<std::size_t>(horizon) + 1, 0.0);
    if (hasStop) {
        const bool useAdaptive = !cfg.brakeCurve.empty();
        // 坡度分量(‰，参与计算前除以旋转质量系数)：下坡为负，削弱制动。
        const double gradeEff = (cfg.ebSb.trainRotateRate > 0.0)
            ? cfg.grade / cfg.ebSb.trainRotateRate : cfg.grade;
        const double aFloor = (cfg.adaptiveBrakeFloor > 0.0)
            ? cfg.adaptiveBrakeFloor : 0.3 * cfg.ebSb.brakeAcc;
        const double aFixed = 0.35 * cfg.ebSb.brakeAcc;  // 退化基准

        // 每步用单步闭式 vref[k] = sqrt(2 * a * dist_k) 反推。a 取该步参考速度
        // 对应的实际制动能力×裕度并加坡度分量。采用单步闭式（不累加相邻步速度）
        // 以保持与原固定保守值一致的保守性，避免 MPC 跟踪滞后导致过冲。
        for (int k = 0; k <= horizon; ++k) {
            const double s = steps[static_cast<std::size_t>(k)].positionRef;
            const double distK = stopPos - s;
            if (distK <= 0.0) {
                vRefAtK[static_cast<std::size_t>(k)] = 0.0;
                continue;
            }
            // 用该步 maxSpeed 估计参考速度查制动能力（vref 不会超过 maxSpeed）。
            const double vEstimate = steps[static_cast<std::size_t>(k)].maxSpeed;
            double a;
            if (useAdaptive) {
                // 分段 margin：远处(distK>=FarDist)用激进 marginFar，临近停车
                // (distK->0)用保守 margin，中间线性过渡。marginFar<=margin 时
                // 禁用分段（全程 margin）。
                double marginK = cfg.adaptiveBrakeMargin;
                if (cfg.adaptiveBrakeMarginFar > cfg.adaptiveBrakeMargin
                    && cfg.adaptiveBrakeFarDist > 0.0) {
                    const double ratio = std::min(1.0, distK / cfg.adaptiveBrakeFarDist);
                    marginK = cfg.adaptiveBrakeMargin
                            + ratio * (cfg.adaptiveBrakeMarginFar - cfg.adaptiveBrakeMargin);
                }
                a = brakeAccelBySpeed(cfg.brakeCurve, vEstimate) * marginK + gradeEff;
                a = std::max(a, aFloor);
            } else {
                a = aFixed;
            }
            double vRef = std::sqrt(2.0 * a * distK);
            vRef = std::min(vRef, steps[static_cast<std::size_t>(k)].maxSpeed);
            vRefAtK[static_cast<std::size_t>(k)] = vRef;
        }
    }

    // 6) 填 targetSpeed：停车段用自适应 vref，非停车段保持 maxSpeed。
    for (int k = 0; k <= horizon; ++k) {
        if (hasStop) {
            const double s = steps[static_cast<std::size_t>(k)].positionRef;
            const double distToStop = stopPos - s;
            if (distToStop > 0.0) {
                steps[static_cast<std::size_t>(k)].targetSpeed =
                    vRefAtK[static_cast<std::size_t>(k)];
            } else {
                steps[static_cast<std::size_t>(k)].targetSpeed = 0.0;
                steps[static_cast<std::size_t>(k)].maxSpeed = 0.0;
                if (k == horizon) steps[static_cast<std::size_t>(k)].isStopStep = true;
            }
        }
        // 非 hasStop 时 targetSpeed 已在 step 4 设为 maxSpeed。
    }

    return steps;
}

}  // namespace ato
