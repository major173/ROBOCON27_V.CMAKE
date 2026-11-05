#pragma once

#include <cstdint>
#include <limits>

// NaN 明确表示待实测/整定，允许合法的 0 与未配置值区分。
inline constexpr float kGripperUnconfigured =
    std::numeric_limits<float>::quiet_NaN();


#define GRIPPER_PULLEY_RADIUS_MM   kGripperUnconfigured
#define GRIPPER_DIRECTION             -1 
#define GRIPPER_MM_PER_RAD            33.1596f
#define GRIPPER_MIN_WIDTH_MM          200
#define GRIPPER_MAX_WIDTH_MM          412
#define GRIPPER_WIDTH_TOLERANCE_MM    10

// 所有量均为电机输出端量；这里不重复进行电机内部减速比换算。
struct GripperConfig {
  struct Mechanical {
    // +1：电机正转使开口增大；-1：电机正转使开口减小。(待测试)
    int direction{GRIPPER_DIRECTION};
    // 整个夹爪开口的变化量，单位 mm/rad。(输出轴旋转1rad变化的开口量)
    // 默认 K=2r；可将上方宏覆盖为实测 K，计算时只使用 K。
    float mm_per_rad{GRIPPER_MM_PER_RAD};
    // 搜索物体时允许到达的最小开口，单位 mm。
    float min_width_mm{GRIPPER_MIN_WIDTH_MM};
    // 外侧归零止挡处的实测开口，单位 mm。
    float max_width_mm{GRIPPER_MAX_WIDTH_MM};
    // 判断位置到达目标时使用的允许误差，单位 mm。
    float width_tolerance_mm{GRIPPER_WIDTH_TOLERANCE_MM};
  } mechanical;

  struct Homing {
    float speed_rad_s           {0.4f};        // 向外归零速度幅值，rad/s
    float kd                    {10.0f};       // N.m/(rad/s)
    float contact_torque_nm     {2.6f};        // 外止挡接触阈值，N.m
  } homing;

  struct Moving {
    float width_speed_mm_s      {60.0f};                // 整个开口的变化速度，mm/s
    float kp                    {7};                // N.m/rad
    float kd                    {6};                // N.m/(rad/s)，允许明确填 0
  } moving;

  struct Search {
    float speed_rad_s            {0.3f};        // 向内搜索速度幅值，rad/s
    float kd                     {10.0f};        // N.m/(rad/s)
    float contact_torque_nm      {2.0f};        // 夹物接触阈值，N.m
    // 下列接触确认参数由归零与夹取共用。
    float contact_speed_rad_s   {0.15f};
    uint32_t confirm_frames     {5};                          // 待配置：连续满足条件的新反馈帧数，必须大于 0
  } search;

  struct Holding {
    float torque_nm            {5.0f};         // 默认保持力矩幅值，N.m
    float ramp_nm_s            {2.0f};         // 力矩变化率，N.m/s
    float kd                   {0.0f};                // N.m/(rad/s)，允许明确填 0
  } holding;

  struct Fault {
    uint32_t communication_timeout_ms   {1000};          // 待配置：无有效反馈的时限，ms
    float driver_temperature_c          {60.0f};         // 异常温度阈值，摄氏度
    float winding_temperature_c         {70.0f};         // 异常温度阈值，摄氏度
    float torque_nm                     {30.0f};         // 异常力矩绝对值阈值，N.m
  } fault;
};
