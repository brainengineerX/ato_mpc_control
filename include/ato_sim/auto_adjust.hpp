#pragma once

#include "ato_sim/types.hpp"

#include <array>
#include <map>

namespace ato {

// 停车精度自动修正配置。该模块对应 AutoAdjustModule.c 中的状态机，
// 输出的是“下一次停车目标距离修正值”，不直接参与牵引/制动输出。
struct AutoAdjustConfig {
    bool enabled = false;

    // 允许停车误差窗口。初始/未稳定状态会使用 0.75 倍，稳定/异常状态使用 1 倍。
    int maxError = 40;

    // 没有历史记录时使用的默认修正值。
    int defaultAdjustValue = 0;

    // 稳定后异常记录达到该次数后，才判断是否继续修正。
    int abnormalMaxRecord = 3;
};

// 单周期停车修正输入。
struct AutoAdjustInput {
    int stopPointId = 0;
    DrivePhase drivePhase = DrivePhase::Cruise;
    bool trainStopped = false;
    int distanceToTarget = 0;
};

// 单个停车点的历史记录。字段名沿用原工程，方便和保存文件/日志对照。
struct AutoAdjustRecord {
    AdjustState state = AdjustState::Initial;

    // 当前停车距离修正值，后续由目标速度模块加到 disPlacement。
    int adjustValue = 0;

    // oldAdjArray[0..1] 存最近调整值，oldAdjArray[2..3] 存最近误差。
    int oldAdjNum = 0;
    std::array<int, 4> oldAdjArray{};

    // 稳定后发生异常停车时，异常误差先暂存在 stopErrArray。
    int stopErrNum = 0;
    std::array<int, 4> stopErrArray{};
};

// 停车修正单周期输出。
struct AutoAdjustOutput {
    bool updated = false;
    int stopError = static_cast<int>(kInvalidDistance);
    int currentAdjustValue = 0;
    AdjustState state = AdjustState::Initial;
};

// 多停车点自动修正器。records_ 按 stopPointId 保存每个停车点独立状态。
class AutoAdjuster {
public:
    explicit AutoAdjuster(AutoAdjustConfig config);

    AutoAdjustOutput step(const AutoAdjustInput& in);
    int adjustValueFor(int stopPointId) const;
    AutoAdjustRecord recordFor(int stopPointId) const;

private:
    // 以下私有函数基本对应原 AutoAdjustModule.c 中的静态函数拆分。
    int calculateStopError(const AutoAdjustInput& in) const;
    bool trainStopCalculation(AutoAdjustRecord& record, int stopError);
    void trainToStable(AutoAdjustRecord& record);
    void calculateUnstable(AutoAdjustRecord& record, int stopError);
    void recordAbnormal(AutoAdjustRecord& record, int stopError);
    bool abnormalCheck(AutoAdjustRecord& record);
    void calculateAbnormal(AutoAdjustRecord& record);

    AutoAdjustConfig config_;
    std::map<int, AutoAdjustRecord> records_;
};

}  // namespace ato
