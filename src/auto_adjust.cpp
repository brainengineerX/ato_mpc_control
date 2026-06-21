#include "ato_sim/auto_adjust.hpp"

#include <algorithm>
#include <cmath>

namespace ato {

namespace {

// oldAdjArray 的下标含义沿用 AutoAdjustModule.c：
//   0: 最近一次调整值
//   1: 上一次调整值
//   2: 最近一次停车误差
//   3: 上一次停车误差
constexpr int kValue1Index = 0;
constexpr int kValue2Index = 1;
constexpr int kTargetErr1Index = 2;
constexpr int kTargetErr2Index = 3;

// 原工程把部分记录字段存为 INT8，这里保留同样的数值范围，避免仿真和
// 源码在大误差输入时出现不同截断行为。
int clampInt8Like(int value) {
    return std::max(-128, std::min(127, value));
}

}  // namespace

AutoAdjuster::AutoAdjuster(AutoAdjustConfig config) : config_(config) {}

int AutoAdjuster::calculateStopError(const AutoAdjustInput& in) const {
    // 只有停车阶段且列车零速时才认为停车误差有效；否则返回 ATO_MAX_DTG
    // 语义的无效距离。
    if (in.drivePhase == DrivePhase::Stop && in.trainStopped) {
        return in.distanceToTarget;
    }
    return static_cast<int>(kInvalidDistance);
}

void AutoAdjuster::trainToStable(AutoAdjustRecord& record) {
    if (record.state == AdjustState::Abnormal) {
        // 异常状态回到稳定状态时，源码会清空异常停车记录。
        record.stopErrNum = 0;
        record.stopErrArray.fill(0);
    }
    record.state = AdjustState::Stable;
}

void AutoAdjuster::calculateUnstable(AutoAdjustRecord& record, int stopError) {
    // 计算“不考虑本次自动调整时”的真实误差近似：
    //   wSum = 本次停车误差 + 当前调整值
    int sum = stopError + record.adjustValue;

    if (record.oldAdjNum == 4) {
        // 已有完整历史时，再叠加上一轮调整值和上一轮误差，然后除以 2。
        sum += record.oldAdjArray[kValue1Index] + record.oldAdjArray[kTargetErr1Index];

        // 历史记录滚动：最近值变上一次，当前值写到最近位置。
        record.oldAdjArray[kValue2Index] = record.oldAdjArray[kValue1Index];
        record.oldAdjArray[kValue1Index] = record.adjustValue;
        record.oldAdjArray[kTargetErr2Index] = record.oldAdjArray[kTargetErr1Index];
        record.oldAdjArray[kTargetErr1Index] = clampInt8Like(stopError);
        record.adjustValue = clampInt8Like(sum / 2);
    } else {
        // 没有完整历史时，清空旧数组，用默认修正值 + 本次误差作为新修正值。
        record.oldAdjArray.fill(0);
        record.oldAdjArray[kTargetErr1Index] = clampInt8Like(stopError);
        record.adjustValue = clampInt8Like(config_.defaultAdjustValue + stopError);
    }

    record.oldAdjNum = 4;
    record.state = AdjustState::Unstable;
}

void AutoAdjuster::recordAbnormal(AutoAdjustRecord& record, int stopError) {
    if (std::abs(stopError) <= config_.maxError &&
        record.stopErrNum < static_cast<int>(record.stopErrArray.size())) {
        // 稳定状态后再次发生超窗误差时，先只记录异常，不立即改调整值。
        record.stopErrArray[record.stopErrNum] = clampInt8Like(stopError);
        record.stopErrNum++;
        record.state = AdjustState::Abnormal;
    }
}

bool AutoAdjuster::abnormalCheck(AutoAdjustRecord& record) {
    int lackCount = 0;
    int overCount = 0;

    // 保持原实现：虽然异常误差存在 stopErrArray，但方向判断循环使用
    // oldAdjArray 的正负号。这里不修正逻辑，只做等价剥离。
    for (int i = 0; i < record.stopErrNum && i < static_cast<int>(record.oldAdjArray.size()); ++i) {
        if (record.oldAdjArray[i] >= 0) {
            lackCount++;
        } else {
            overCount++;
        }
    }

    if (lackCount == 0 || overCount == 0) {
        // 全欠标或全过标，认为是稳定同类异常，可以继续修正。
        return true;
    }

    // 既有欠标又有过标，认为误差不稳定，清空异常记录并保持当前修正值。
    record.stopErrNum = 0;
    record.stopErrArray.fill(0);
    return false;
}

void AutoAdjuster::calculateAbnormal(AutoAdjustRecord& record) {
    if (record.stopErrNum <= 2) {
        return;
    }

    // 保持原实现：异常累积后的追加修正使用 oldAdjArray 求均值，而不是
    // stopErrArray。该处是源码行为，不在剥离层改变。
    int sum = 0;
    for (int i = 0; i < record.stopErrNum && i < static_cast<int>(record.oldAdjArray.size()); ++i) {
        sum += record.oldAdjArray[i];
    }

    // 将最近两次异常误差写入历史误差位，然后回到未稳定状态重新收敛。
    record.oldAdjArray[kValue1Index] = record.adjustValue;
    record.oldAdjArray[kValue2Index] = record.adjustValue;
    record.oldAdjArray[kTargetErr1Index] = record.stopErrArray[record.stopErrNum - 1];
    record.oldAdjArray[kTargetErr2Index] = record.stopErrArray[record.stopErrNum - 2];
    record.oldAdjNum = 4;
    record.adjustValue = clampInt8Like(record.adjustValue + sum / record.stopErrNum);
    record.state = AdjustState::Unstable;
    record.stopErrNum = 0;
    record.stopErrArray.fill(0);
}

bool AutoAdjuster::trainStopCalculation(AutoAdjustRecord& record, int stopError) {
    if (stopError == static_cast<int>(kInvalidDistance)) {
        return false;
    }

    // 严进宽出：
    //   初始/未稳定状态：使用 0.75 * maxError，更严格地确认稳定；
    //   稳定/异常状态：使用 1.0 * maxError，避免稳定后频繁抖动。
    const bool strictState =
        record.state == AdjustState::Initial || record.state == AdjustState::Unstable;
    const int targetWindow =
        strictState ? static_cast<int>(static_cast<double>(config_.maxError) * 0.75)
                    : config_.maxError;

    if (std::abs(stopError) < targetWindow) {
        if (record.state != AdjustState::Stable) {
            // 误差进入目标窗，初始/未稳定/异常状态都可转稳定。
            trainToStable(record);
            return true;
        }
        return false;
    }

    if (record.state == AdjustState::Initial || record.state == AdjustState::Unstable) {
        // 初始或未稳定：误差超窗后立即计算新的修正值。
        calculateUnstable(record, stopError);
        return true;
    }

    // 稳定后超窗：先记录异常，等待异常次数达到阈值后再判断是否追加修正。
    recordAbnormal(record, stopError);
    return true;
}

AutoAdjustOutput AutoAdjuster::step(const AutoAdjustInput& in) {
    AutoAdjustOutput out;
    if (!config_.enabled) {
        // 功能关闭时，不创建/更新停车点记录。
        return out;
    }

    // 每个停车点独立保存一份修正状态。
    AutoAdjustRecord& record = records_[in.stopPointId];
    out.stopError = calculateStopError(in);
    out.updated = trainStopCalculation(record, out.stopError);

    if (record.state == AdjustState::Abnormal &&
        record.stopErrNum >= config_.abnormalMaxRecord) {
        if (abnormalCheck(record)) {
            calculateAbnormal(record);
            out.updated = true;
        }
    }

    out.currentAdjustValue = record.adjustValue;
    out.state = record.state;
    return out;
}

int AutoAdjuster::adjustValueFor(int stopPointId) const {
    const auto iter = records_.find(stopPointId);
    return (iter == records_.end()) ? 0 : iter->second.adjustValue;
}

AutoAdjustRecord AutoAdjuster::recordFor(int stopPointId) const {
    const auto iter = records_.find(stopPointId);
    return (iter == records_.end()) ? AutoAdjustRecord{} : iter->second;
}

}  // namespace ato
