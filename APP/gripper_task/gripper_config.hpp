#pragma once

#include <cstdint>

#define GRIPPER_DIRECTION          -1
#define GRIPPER_MM_PER_RAD         33.1596f
#define GRIPPER_MIN_WIDTH_MM       200.0f
#define GRIPPER_MAX_WIDTH_MM       412.0f
#define GRIPPER_WIDTH_TOLERANCE_MM  10.0f
#define GRIPPER_PRE_WIDTH_MM       350.0f

// 实测机构与控制参数；角度、速度、力矩均使用电机输出端量。
struct GripperConfig {
  struct Mechanical {
    // +1：电机正转使开口增大；-1：电机正转使开口减小。
    int direction{GRIPPER_DIRECTION};
    float mm_per_rad{GRIPPER_MM_PER_RAD};                 // 整个开口变化量，mm/rad
    float min_width_mm{GRIPPER_MIN_WIDTH_MM};              // 搜索最小开口，mm
    float max_width_mm{GRIPPER_MAX_WIDTH_MM};              // 外侧归零止挡处开口，mm
    float width_tolerance_mm{GRIPPER_WIDTH_TOLERANCE_MM};  // 到位允许误差，mm
  } mechanical;

  struct Homing {
    float speed_rad_s{0.4f};       // 向外归零速度幅值，rad/s
    float kd{10.0f};               // N.m/(rad/s)
    float contact_torque_nm{2.6f}; // 外止挡接触阈值，N.m
  } homing;

  struct Moving {
    float width_speed_mm_s{60.0f}; // 整个开口变化速度，mm/s
    float kp{7.0f};                // N.m/rad
    float kd{6.0f};                // N.m/(rad/s)
  } moving;

  struct Search {
    float speed_rad_s{0.3f};       // 向内搜索速度幅值，rad/s
    float kd{10.0f};               // N.m/(rad/s)
    float contact_torque_nm{2.0f}; // 夹物接触阈值，N.m
    // 归零与夹取共用：低速且力矩达阈值的连续新反馈帧。
    float contact_speed_rad_s{0.15f};
    uint32_t confirm_frames{5};
  } search;

  struct Holding {
    float torque_nm{5.0f}; // 默认保持力矩幅值，N.m
    float ramp_nm_s{2.0f}; // 力矩变化率，N.m/s
    float kd{0.0f};        // N.m/(rad/s)
  } holding;
};
