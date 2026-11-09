#include "GripperController.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kMillisecondsPerSecond = 1000.0f;

float approach(float value, float target, float step) {
  return value < target ? std::min(value + step, target)
                        : std::max(value - step, target);
}
} // namespace

void GripperController::init(const GripperConfig &config) {
  config_ = config;
  status_ = GripperStatus{};
  request_ = GripperRequest{};
  clock_started_ = feedback_seen_ = grip_mode_ = false;
  last_update_ms_ = contact_count_ = 0;
  reference_width_mm_ = 0.0f;
  auto_home_pending_ = true;
}

float GripperController::angleToWidth(float angle_rad) const {
  const auto &m = config_.mechanical;
  return m.max_width_mm + m.direction * m.mm_per_rad *
         (angle_rad - status_.home_position_rad);
}

float GripperController::widthToAngle(float width_mm) const {
  const auto &m = config_.mechanical;
  return status_.home_position_rad +
         (width_mm - m.max_width_mm) / (m.direction * m.mm_per_rad);
}

void GripperController::transition(GripperState state, GripperResult result) {
  status_.state = state;
  status_.result = result;
  status_.closing_phase = GripperClosingPhase::Approach;
  contact_count_ = 0;
  if (state != GripperState::Holding) status_.ramped_hold_torque_nm = 0.0f;
  if (state == GripperState::Homing ||
      state == GripperState::Uninitialized) status_.homed = false;
  if (state == GripperState::Ready ||
      state == GripperState::Uninitialized) grip_mode_ = false;
}

bool GripperController::requestValid(const GripperRequest &r) const {
  if (r.action == GripperAction::None || r.action == GripperAction::Stop ||
      r.action == GripperAction::Home)
    return true;
  // 绝对宽度运动需要先通过归零建立坐标。
  if (!status_.homed) return false;
  if (r.action == GripperAction::Open || r.action == GripperAction::Release)
    return true;
  if (r.action != GripperAction::MoveToWidth && r.action != GripperAction::Grip)
    return false;
  const auto &m = config_.mechanical;
  if (!std::isfinite(r.width_mm) || r.width_mm < m.min_width_mm ||
      r.width_mm > m.max_width_mm)
    return false;
  // 力矩参数表示幅值，0 使用配置中的保持力矩。
  return r.action != GripperAction::Grip ||
         (std::isfinite(r.hold_torque_nm) && r.hold_torque_nm >= 0.0f);
}

bool GripperController::request(GripperAction action, float width_mm, float hold_torque_nm) {
  const GripperRequest next{action, width_mm, hold_torque_nm};
  if (!requestValid(next)) {
    status_.result = GripperResult::Rejected;
    return false;
  }
  if (action != GripperAction::None) {
    request_ = next;
    status_.result = GripperResult::None;
  }
  return true;
}

bool GripperController::acceptRequest(const GripperRequest &r,
                                     const IM6014::Feedback &feedback) {
  if (r.action == GripperAction::None) return false;
  if (r.action == GripperAction::Stop) {
    auto_home_pending_ = false;
    status_.contact_detected = false;
    transition(status_.homed ? GripperState::Ready : GripperState::Uninitialized,
               GripperResult::Stopped);
    return true;
  }
  if (r.action == GripperAction::Home) {
    status_.contact_detected = false;
    auto_home_pending_ = !feedback_seen_;
    transition(feedback_seen_ ? GripperState::Homing : GripperState::Uninitialized);
    return true;
  }

  status_.target_width_mm = (r.action == GripperAction::Open ||
                            r.action == GripperAction::Release)
                               ? config_.mechanical.max_width_mm : r.width_mm;
  status_.contact_detected = false;
  grip_mode_ = r.action == GripperAction::Grip;
  status_.target_hold_torque_nm = grip_mode_
      ? (r.hold_torque_nm == 0.0f ? config_.holding.torque_nm : r.hold_torque_nm) : 0.0f;
  reference_width_mm_ = angleToWidth(feedback.position);
  transition(status_.target_width_mm > reference_width_mm_
                 ? GripperState::Opening : GripperState::Closing);
  return true;
}

bool GripperController::contactConfirmed(bool condition, bool fresh) {
  if (!fresh) return false; // 旧帧不能增加计数。
  if (!condition) contact_count_ = 0;
  else if (contact_count_ < config_.search.confirm_frames) ++contact_count_;
  return contact_count_ >= config_.search.confirm_frames;
}

void GripperController::advance(const IM6014::Feedback &feedback, bool fresh, float dt_s) {
  const auto &m = config_.mechanical;
  const bool slow = std::fabs(feedback.speed) <= config_.search.contact_speed_rad_s;
  switch (status_.state) {
  case GripperState::Homing:
    if (contactConfirmed(slow && m.direction * feedback.torque >=
                         config_.homing.contact_torque_nm, fresh)) {
      status_.home_position_rad = feedback.position;
      status_.homed = true;
      status_.actual_width_mm = m.max_width_mm;
      status_.target_width_mm = m.max_width_mm;
      status_.contact_detected = true;
      transition(GripperState::Ready, GripperResult::Homed);
    }
    break;
  case GripperState::Opening:
  case GripperState::Closing: {
    if (status_.state == GripperState::Closing &&
        contactConfirmed(slow && -m.direction * feedback.torque >=
                         config_.search.contact_torque_nm, fresh)) {
      status_.contact_detected = true;
      if (grip_mode_) {
        transition(GripperState::Holding);
        status_.ramped_hold_torque_nm = -m.direction * feedback.torque;
      } else {
        transition(GripperState::Ready, GripperResult::ContactStopped);
      }
      break;
    }
    if (status_.closing_phase == GripperClosingPhase::Search) {
      if (fresh && status_.actual_width_mm <= m.min_width_mm + m.width_tolerance_mm)
        transition(GripperState::Ready, GripperResult::NoObject);
      break;
    }
    reference_width_mm_ = approach(reference_width_mm_, status_.target_width_mm,
                                   config_.moving.width_speed_mm_s * dt_s);
    if (fresh && std::fabs(status_.actual_width_mm - status_.target_width_mm) <=
                     m.width_tolerance_mm) {
      if (grip_mode_) {
        transition(GripperState::Closing);
        status_.closing_phase = GripperClosingPhase::Search;
        if (status_.actual_width_mm <= m.min_width_mm + m.width_tolerance_mm)
          transition(GripperState::Ready, GripperResult::NoObject);
      } else {
        transition(GripperState::Ready, GripperResult::Reached);
      }
    }
    break;
  }
  case GripperState::Holding:
    status_.ramped_hold_torque_nm = approach(status_.ramped_hold_torque_nm,
        status_.target_hold_torque_nm, config_.holding.ramp_nm_s * dt_s);
    if (status_.result == GripperResult::None &&
        status_.ramped_hold_torque_nm == status_.target_hold_torque_nm)
      status_.result = GripperResult::HoldTargetReached;
    break;
  default:
    break;
  }
}

void GripperController::makeCommand(IM6014::Command &command) const {
  const auto &m = config_.mechanical;
  switch (status_.state) {
  case GripperState::Homing:
    command.mode = IM6014::Mode::Foc;
    command.speed = m.direction * config_.homing.speed_rad_s;
    command.kd = config_.homing.kd;
    break;
  case GripperState::Opening:
  case GripperState::Closing:
    command.mode = IM6014::Mode::Foc;
    if (status_.closing_phase == GripperClosingPhase::Search) {
      command.speed = -m.direction * config_.search.speed_rad_s;
      command.kd = config_.search.kd;
    } else {
      command.position = widthToAngle(reference_width_mm_);
      if (reference_width_mm_ != status_.target_width_mm)
        command.speed = (status_.target_width_mm > reference_width_mm_ ? 1.0f : -1.0f) *
                        config_.moving.width_speed_mm_s / (m.direction * m.mm_per_rad);
      command.kp = config_.moving.kp;
      command.kd = config_.moving.kd;
    }
    break;
  case GripperState::Holding:
    command.mode = IM6014::Mode::Foc;
    command.torque = -m.direction * status_.ramped_hold_torque_nm;
    command.kd = config_.holding.kd;
    break;
  default:
    break;
  }
}

void GripperController::update(const IM6014::Feedback &feedback, bool fresh,
                               uint32_t now_ms,
                               IM6014::Command &command_out) {
  command_out = IM6014::Command{}; // 所有状态从完整的 Stop/零参数开始生成。
  if (!clock_started_) {
    clock_started_ = true;
    last_update_ms_ = now_ms;
  }
  const float dt_s = static_cast<uint32_t>(now_ms - last_update_ms_) / kMillisecondsPerSecond;
  last_update_ms_ = now_ms;
  feedback_seen_ = feedback_seen_ || fresh;
  if (fresh && status_.homed) status_.actual_width_mm = angleToWidth(feedback.position);

  const GripperRequest pending = request_;
  request_.action = GripperAction::None;
  bool action_started = acceptRequest(pending, feedback);
  if (!feedback_seen_) return;
  if (auto_home_pending_) {
    auto_home_pending_ = false;
    status_.contact_detected = false;
    transition(GripperState::Homing);
    action_started = true;
  }
  // 刚切换动作时的反馈属于前一个命令，不能拿它确认新动作的接触。
  advance(feedback, fresh && !action_started, action_started ? 0.0f : dt_s);
  makeCommand(command_out);
}
