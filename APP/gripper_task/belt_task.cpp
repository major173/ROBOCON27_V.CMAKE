#include "gripper_task.h"
#include "belt_config.hpp"
#include "Motor.hpp"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include <algorithm>
#include <atomic>
#include "pid_controller.h"

osThreadId_t BeltTaskHandle;
extern C610Motor gripper_left_2006;
extern C610Motor gripper_right_2006;

namespace {
struct BeltAxis {
  C610Motor &motor;
  float direction;
  PID_t speed_pid{};
  float zero_deg{0.0f};
  float target_mm{0.0f};
};

BeltAxis belts[] = {{gripper_left_2006, -1.0f},
                   {gripper_right_2006, 1.0f}};
std::atomic<BeltDirection> requested_direction{BeltDirection::Hold};

void startBelts() {
  for (auto &axis : belts) {
    auto &pid = axis.speed_pid;
    PID_Init(&pid);
    pid.Kp = BeltConfig::kSpeedKp;
    pid.Ki = BeltConfig::kSpeedKi;
    pid.Kd = BeltConfig::kSpeedKd;
    pid.MaxOut = BeltConfig::kMaxCurrentCommand;
    pid.IntegralLimit = BeltConfig::kIntegralLimit;
    pid.DeadBand = -1.0f; // 零误差时保留承担负载的积分。
    pid.Improve = Integral_Limit;
    (void)DWT_GetDeltaT(&pid.DWT_CNT);
    axis.zero_deg = axis.motor.getCurrentSumPos();
    axis.target_mm = 0.0f;
  }
}

void updateBelt(BeltAxis &axis, float step_mm, bool capture_target) {
  const float actual_mm = axis.direction * BeltConfig::kMmPerRad *
      (axis.motor.getCurrentSumPos() - axis.zero_deg) * DEGREE_2_RAD;
  // 松手或反向时只锁存一次，之后位置环持续保持，速度积分不清零。
  if (capture_target) axis.target_mm = actual_mm;
  axis.target_mm += step_mm;
  const float speed_mm_s = std::clamp(
      BeltConfig::kPositionKp * (axis.target_mm - actual_mm),
      -BeltConfig::kMaxSpeedMmS, BeltConfig::kMaxSpeedMmS);
  const float speed_rad_s = axis.direction * speed_mm_s / BeltConfig::kMmPerRad;
  axis.motor.setMotorCmd(PID_Calculate(
      &axis.speed_pid, axis.motor.getCurrentSpeed(), speed_rad_s));
}
}

void gripperBeltMove(BeltDirection direction) {
  requested_direction.store(direction, std::memory_order_relaxed);
}

void beltTask(void *argument) {
  (void)argument;
  TickType_t wake_tick = xTaskGetTickCount();
  uint32_t last_update_ms = HAL_GetTick();
  BeltDirection previous_direction = BeltDirection::Hold;
  bool started = false;

  for (;;) {
    const uint32_t now_ms = HAL_GetTick();
    float dt = (now_ms - last_update_ms) * 0.001f;
    last_update_ms = now_ms;
    const auto direction = requested_direction.load(std::memory_order_relaxed);
    if (!started && direction != BeltDirection::Hold) {
      startBelts();
      started = true;
      dt = 0.0f;
    }
    if (started) {
      const bool capture_target = previous_direction != BeltDirection::Hold &&
                                  direction != previous_direction;
      const float step_mm = static_cast<int>(direction) * BeltConfig::kJogSpeedMmS * dt;
      for (auto &axis : belts) updateBelt(axis, step_mm, capture_target);
    } else {
      for (auto &axis : belts) axis.motor.setMotorCmd(0.0f);
    }
    previous_direction = direction;
    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(1));
  }
}
