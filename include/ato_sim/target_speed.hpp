#pragma once

#include "ato_sim/types.hpp"

#include <vector>

namespace ato {

// GEN2 EB/SB 曲线计算参数。对应 AimSpdCompute.c 中
// FontEbSbCalcBaseGEN2() 的主要输入。
struct EbSbConfig {
    // 最大牵引加速度，用于牵引切除延时阶段。
    double motorAcc = 80.0;

    // 制动加速度。计算 EBI 时使用紧急制动率，计算 SBI 时使用常用制动率。
    double brakeAcc = 100.0;

    // 牵引切除延时 t0、制动建立延时 t1。
    double motorDelay = 1.0;
    double brakeDelay = 2.0;

    // 列车旋转质量系数。坡度参与加速度计算前会除以该系数。
    double trainRotateRate = 1.0;
};

// 目标速度模块配置。对应 AimSpdCompute.c 中旧目标速度分支的主要算法参数。
struct TargetSpeedConfig {
    double maxLineLimitSpeed = 2200.0;
    double defaultBrakeAcc = 100.0;

    // EBI/SBI 最小差值，用于将 ATP/EBI 速度转换为 ATO 目标速度上限。
    double minMarginFromEbi = 55.0;

    // SBI 偏移参数：
    // 距离偏移 = RestrictedV * sbiTimeOffset + sbiDistanceOffset
    // 速度偏移 = sbiSpeedOffset
    double sbiTimeOffset = 1.0;
    double sbiDistanceOffset = 500.0;
    double sbiSpeedOffset = 27.0;

    // MA_ADJUSTMENT_DIST，当前原工程定义为 0，但保留字段方便对拍。
    double maAdjustmentDistance = 0.0;

    // 是否启用自动停车修正。当前修正值由 TargetSpeedInput.stopDistanceAdjust 传入。
    bool autoStopAdjustEnabled = false;
};

// 目标速度计算输入。所有外部查询结果都已经被压平成结构体字段：
// 例如线路限速/障碍物由 restrictions 给出，不在算法层访问 DQU。
struct TargetSpeedInput {
    double trainSpeed = 0.0;
    double ebiSpeed = 0.0;
    double runLevelSpeed = 0.0;
    double stopDistance = kInvalidDistance;
    double stopDistanceAdjust = 0.0;
    double parkingBrakeAcc = 100.0;
    bool openLoopFallback = false;
    std::vector<SpeedRestriction> restrictions;
};

// 精确停车目标速度子算法输出。
struct StopSpeedResult {
    // 修正后的停车距离：AtoDtg + MA_ADJUSTMENT_DIST + StopDistanceAdjust。
    double displacement = 0.0;

    // 未经过 EBI 裕量限幅的 sqrt(2 * a * s) 速度。
    double rawStopSpeed = 0.0;

    // 经过 EBI 裕量限幅后的停车目标速度。
    double targetSpeed = 0.0;
};

// 一组约束计算出的 EB/SB 速度。
struct EbSbResult {
    double ebiSpeed = 0.0;
    double sbiSpeed = 0.0;
};

// 目标速度模块总输出，对应原工程对外提供的 AtoTargetSpd、CurMaxSbv、
// AtoIsFall、AtoCutTrack 等核心算法量。
struct TargetSpeedResult {
    double targetSpeed = 0.0;
    double curMaxSbv = 0.0;
    double stopTargetSpeed = 0.0;
    double sbiSpeed = 0.0;
    bool fallingArea = false;
    bool cutTraction = false;
};

// GEN2 单个限速/障碍物的速度包络计算。
double calculateGen2RestrictedSpeed(const EbSbConfig& cfg,
                                    double distance,
                                    double grade,
                                    double restrictedSpeed);

// 多个限速/障碍物计算 EB/SB，并取最小速度作为当前约束。
EbSbResult calculateEbSbSpeed(const EbSbConfig& ebCfg,
                              const TargetSpeedConfig& targetCfg,
                              const std::vector<SpeedRestriction>& restrictions);

// 精确停车目标速度：V = sqrt(2 * AtoBrakeAcc * displacement)，再受 EBI 裕量限制。
StopSpeedResult calculateStopTargetSpeed(const TargetSpeedConfig& cfg,
                                         const TargetSpeedInput& in);

// 目标速度总入口：巡航/SBI/运行等级/停车目标速度综合取小。
TargetSpeedResult calculateTargetSpeed(const TargetSpeedConfig& cfg,
                                       const EbSbConfig& ebCfg,
                                       const TargetSpeedInput& in);

}  // namespace ato
