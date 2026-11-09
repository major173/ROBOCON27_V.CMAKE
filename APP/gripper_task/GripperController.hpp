/**
 * @file GripperController.hpp
 * @brief 夹爪动作状态机：归零、宽度控制、接触搜索与力矩保持。
 */
#pragma once

#include "IM6014.hpp"
#include "gripper_config.hpp"
#include <cstdint>

enum class GripperState : uint8_t {
  Uninitialized,       // 尚未建立宽度坐标
  Homing,              // 向外寻找机械零点
  Ready,               // 归零完成，等待动作请求
  Opening,             // 正在打开
  Closing,             // 正在闭合
  Holding              // 正在保持夹持状态
};

enum class GripperAction : uint8_t {
  None,                // 无动作请求
  Home,                // 归零
  Open,                // 完全打开
  MoveToWidth,         // 移动到指定开口宽度
  Grip,                // 预夹、搜索并保持
  Release,             // 松开至最大宽度
  Stop                 // 停止当前动作
};

enum class GripperClosingPhase : uint8_t {
  Approach,            // 位置控制，移动到预夹宽度
  Search               // 速度控制，低作用力寻找物块
};

enum class GripperResult : uint8_t {
  None,                // 无结果
  Homed,               // 归零完成
  Reached,             // 目标宽度到位
  ContactStopped,      // 普通闭合遇到接触后停止
  NoObject,            // 搜索至最小宽度，未发现物体
  HoldTargetReached,   // 保持力矩斜坡到达目标
  Stopped,             // 已停止
  Rejected             // 请求参数或当前坐标条件不满足
};

struct GripperRequest {
  GripperAction action{GripperAction::None};
  float width_mm{0.0f};        // MoveToWidth 目标或 Grip 预夹宽度，mm
  float hold_torque_nm{0.0f};  // Grip 力矩幅值；0 使用配置值，N.m
};

struct GripperStatus {
  GripperState state{GripperState::Uninitialized};
  GripperClosingPhase closing_phase{GripperClosingPhase::Approach};
  GripperResult result{GripperResult::None};
  bool homed{false};                  // 绝对宽度坐标是否已建立
  bool contact_detected{false};       // 当前动作是否确认接触
  float home_position_rad{0.0f};      // homed 后有效
  float actual_width_mm{0.0f};        // homed 后有效，记录最近一次新反馈
  float target_width_mm{0.0f};
  float target_hold_torque_nm{0.0f};  // 保持力矩目标幅值
  float ramped_hold_torque_nm{0.0f};  // 前馈力矩斜坡幅值，不是夹紧力测量
};

class GripperController {
public:
  void init(const GripperConfig &config);
  // 仅由拥有控制器的任务调用；每个周期处理最后一个有效请求。
  bool request(GripperAction action, float width_mm = 0.0f,
               float hold_torque_nm = 0.0f);
  // fresh 表示本周期取得新反馈；首次新反馈启动自动归零。
  void update(const IM6014::Feedback &feedback, bool fresh, uint32_t now_ms,
              IM6014::Command &command_out);
  const GripperStatus &status() const { return status_; }

private:
  bool requestValid(const GripperRequest &request) const;
  bool acceptRequest(const GripperRequest &request,
                     const IM6014::Feedback &feedback);
  void advance(const IM6014::Feedback &feedback, bool fresh, float dt_s);
  void makeCommand(IM6014::Command &command) const;
  void transition(GripperState state, GripperResult result = GripperResult::None);
  bool contactConfirmed(bool condition, bool fresh);
  float angleToWidth(float angle_rad) const;
  float widthToAngle(float width_mm) const;

  GripperConfig config_{};
  GripperRequest request_{};
  GripperStatus status_{};
  bool clock_started_{false};
  bool feedback_seen_{false};       // 首帧前不执行运动
  bool auto_home_pending_{false};   // Stop 可取消尚未执行的自动归零
  bool grip_mode_{false};           // 区分普通宽度移动与夹取
  uint32_t last_update_ms_{0};
  uint32_t contact_count_{0};        // 连续满足接触条件的新反馈帧数
  float reference_width_mm_{0.0f};  // 平滑的位置参考
};
