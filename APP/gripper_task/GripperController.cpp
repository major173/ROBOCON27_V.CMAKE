#include "GripperController.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kMillisecondsPerSecond = 1000.0f;

// 仅检查机构模型能否使用，不把未配置的值替换为猜测值。
bool geometryValid(const GripperConfig::Mechanical& m) {
    const bool direction_ok =
        m.direction == 1 || m.direction == -1;
    const bool width_ok =
        m.min_width_mm >= 0.0f &&
        std::isfinite(m.max_width_mm) &&
        m.max_width_mm > m.min_width_mm;
    const bool conversion_ok =
        m.mm_per_rad > 0.0f;
    const bool tolerance_ok =
        m.width_tolerance_mm > 0.0f &&
        m.width_tolerance_mm < m.max_width_mm - m.min_width_mm;
    return direction_ok && width_ok &&
           conversion_ok && tolerance_ok;
}

bool configValid(const GripperConfig& c) {
    return geometryValid(c.mechanical) &&
           c.search.confirm_frames > 0 &&
           c.fault.communication_timeout_ms > 0 &&
           c.holding.ramp_nm_s > 0.0f;
}

float approach(float value, float target, float step) {
  return value < target ? 
  std::min(value + step, target)
  : std::max(value - step, target);
}
} // namespace

bool GripperController::init(const GripperConfig &config) {
  config_ = config;
  status_ = GripperStatus{};
  request_.action = GripperAction::None;
  request_.width_mm = 0.0f;
  request_.hold_torque_nm = 0.0f;
  clock_started_ = feedback_seen_ = grip_mode_ = false;
  started_ms_ = last_update_ms_ = last_feedback_sequence_ = contact_count_ = 0;
  reference_width_mm_ = 0.0f;
  status_.config_valid = configValid(config_);
  auto_home_pending_ = status_.config_valid;
  return status_.config_valid;
}

float GripperController::angleToWidth(float angle_rad) const {
  const auto &m = config_.mechanical;
  return  m.max_width_mm + m.direction * m.mm_per_rad *
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
  if (state == GripperState::Homing || state == GripperState::Fault ||
      state == GripperState::Uninitialized) status_.homed = false;
  if (state == GripperState::Ready || state == GripperState::Fault ||
      state == GripperState::Uninitialized) grip_mode_ = false;
}
//检查请求合理性
bool GripperController::requestValid(const GripperRequest& r) const {
    // 无动作或停止，直接允许
    if (r.action == GripperAction::None ||
        r.action == GripperAction::Stop)
        return true;
    // 清除故障只能在 Fault 状态请求
    if (r.action == GripperAction::ClearFault)
        return status_.state == GripperState::Fault;
    // 配置无效或存在故障，拒绝正常动作
    if (!status_.config_valid ||
        status_.state == GripperState::Fault)
        return false;
    // 归零不要求已经归零
    if (r.action == GripperAction::Home)
        return true;
    // 普通运动必须已经归零且电机在线
    if (!status_.homed || !status_.motor_online)
        return false;
    // 打开和释放不需要额外参数
    if (r.action == GripperAction::Open ||
        r.action == GripperAction::Release)
        return true;
    // 检查目标宽度
    if (r.action == GripperAction::MoveToWidth ||
        r.action == GripperAction::Grip) {
        const auto& m = config_.mechanical;
        if (!std::isfinite(r.width_mm) ||
            r.width_mm < m.min_width_mm ||
            r.width_mm > m.max_width_mm)
            return false;
        // 只有 Grip 需要检查保持力矩
        if (r.action == GripperAction::Grip) {
            const float torque = r.hold_torque_nm == 0.0f
                ? config_.holding.torque_nm
                : r.hold_torque_nm;

            if (!std::isfinite(torque) ||
                torque <= 0.0f ||
                torque >= config_.fault.torque_nm)
                return false;
        }
        return true;
    }
    return false;
}

bool GripperController::request(GripperAction action, float width_mm, float hold_torque_nm) {
  const GripperRequest next{action, width_mm, hold_torque_nm};
  if (!requestValid(next)) {
    status_.result = GripperResult::Rejected;
    return false;
  }
  if (action != GripperAction::None) {
    request_.width_mm = width_mm;
    request_.hold_torque_nm = hold_torque_nm;
    request_.action = action; // 最后发布动作；仅供拥有控制器的任务调用。
  }
  return true;
}

bool GripperController::acceptRequest(const GripperRequest &r,
                                      const IM6014::Feedback &feedback) {
  if (r.action == GripperAction::None) return false;
  if (!requestValid(r)) {
    status_.result = GripperResult::Rejected;
    return false;
  }
  if (r.action == GripperAction::Stop) {
    auto_home_pending_ = false;
    status_.contact_detected = false;
    transition(status_.homed ? GripperState::Ready : GripperState::Uninitialized,
               GripperResult::Stopped);
    return true;
  }
  if (r.action == GripperAction::Home || r.action == GripperAction::ClearFault) {
    if (r.action == GripperAction::ClearFault && !status_.motor_online) {
      status_.result = GripperResult::Rejected;
      return false;
    }
    status_.faults = GRIPPER_FAULT_NONE;
    status_.contact_detected = false;
    auto_home_pending_ = !status_.motor_online;
    transition(status_.motor_online ? GripperState::Homing : GripperState::Uninitialized);
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

uint8_t GripperController::currentFaults(const IM6014::Feedback &feedback) const {
  uint8_t faults = GRIPPER_FAULT_NONE;
  if (feedback_seen_) {
    if (feedback.driver_temperature >= config_.fault.driver_temperature_c ||
        feedback.winding_temperature >= config_.fault.winding_temperature_c)
      faults |= GRIPPER_FAULT_TEMPERATURE;
    if (std::fabs(feedback.torque) >= config_.fault.torque_nm)
      faults |= GRIPPER_FAULT_TORQUE;
  }
  return faults;
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
    if (status_.ramped_hold_torque_nm == status_.target_hold_torque_nm)
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

void GripperController::update(const IM6014::Feedback &feedback, uint32_t sequence,
                               uint32_t last_rx_ms, uint32_t now_ms,
                               IM6014::Command &command_out) {
  command_out = IM6014::Command{}; // 所有状态从完整的 Stop/零参数开始生成。
  if (!clock_started_) {
    clock_started_ = true;
    started_ms_ = last_update_ms_ = now_ms;
  }
  const float dt_s = static_cast<uint32_t>(now_ms - last_update_ms_) / kMillisecondsPerSecond;
  last_update_ms_ = now_ms;
  const bool fresh = sequence != last_feedback_sequence_;
  last_feedback_sequence_ = sequence;
  feedback_seen_ = feedback_seen_ || fresh;

  // 通信观测不依赖机构配置；失联时限本身未配置时，不能判定在线。
  const uint32_t age = now_ms - (feedback_seen_ ? last_rx_ms : started_ms_);
  const uint32_t timeout_ms = config_.fault.communication_timeout_ms;
  const bool timeout_valid = timeout_ms > 0;
  status_.motor_online = feedback_seen_ && timeout_valid && age < timeout_ms;

  // 请求仅暂存在一个槽中；动作最后提交，读完参数后再清空动作。
  const GripperRequest pending{request_.action, request_.width_mm, request_.hold_torque_nm};
  request_.action = GripperAction::None;
  if (!status_.config_valid) {
    if (pending.action == GripperAction::Stop) status_.result = GripperResult::Stopped;
    else if (pending.action != GripperAction::None) status_.result = GripperResult::Rejected;
    return; // 配置缺失不变成第四类故障，也不能使用 K=NaN 的模型。
  }

  const bool disconnected = age >= timeout_ms;
  if (fresh && status_.homed) status_.actual_width_mm = angleToWidth(feedback.position);
  uint8_t faults = currentFaults(feedback);
  if (disconnected) faults |= GRIPPER_FAULT_COMMUNICATION;
  if (faults != GRIPPER_FAULT_NONE) {
    status_.faults |= faults;
    auto_home_pending_ = false;
    status_.contact_detected = false;
    if (status_.state != GripperState::Fault) transition(GripperState::Fault);
    if (pending.action != GripperAction::None) status_.result = GripperResult::Rejected;
    return;
  }
  // 故障消失仍然锁存；只有人工 ClearFault 可以重新归零。
  if (status_.state == GripperState::Fault && pending.action != GripperAction::ClearFault) {
    if (pending.action != GripperAction::None) status_.result = GripperResult::Rejected;
    return;
  }
  bool action_started = acceptRequest(pending, feedback);
  if (auto_home_pending_ && status_.motor_online) {
    auto_home_pending_ = false;
    status_.contact_detected = false;
    transition(GripperState::Homing);
    action_started = true;
  }
  // 刚切换动作时的反馈属于前一个命令，不能拿它确认新动作的接触。
  advance(feedback, fresh && !action_started, action_started ? 0.0f : dt_s);
  makeCommand(command_out);
}
