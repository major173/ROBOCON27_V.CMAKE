#pragma once

namespace BeltConfig {
// 输出轴每转约 490°，传送带移动 100 mm；向上时左反转、右正转。
inline constexpr float kMmPerRad = 11.693f;
inline constexpr float kPositionKp = 3.0f;       // 1/s
inline constexpr float kJogSpeedMmS = 100.0f;    // 位置参考变化速度
inline constexpr float kMaxSpeedMmS = 500.0f;   // 位置环输出速度上限

// 速度 PI 输入为输出轴 rad/s，输出为 C610 电流指令。
inline constexpr float kSpeedKp = 3000.0f;
inline constexpr float kSpeedKi = 100.0f;
inline constexpr float kSpeedKd = 0.0f;
inline constexpr float kMaxCurrentCommand = 3000.0f;
inline constexpr float kIntegralLimit = 500.0f;
}
