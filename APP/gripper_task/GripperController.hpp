  /** 
  * @file GripperController.hpp
  * @author FN
  * @brief 夹爪控制器，状态机、动作请求、故障判定、通信超时
  *        外部提交一个动作请求->控制器判断当前状态是否能接受这个请求—
  *        每个控制周期会读取电机反馈—>根据电机反馈决定是否切换状态->生成IM6014电机控制命令
  * @version 0.1
  * @date 2026-04-21
  */
#pragma once

#include "IM6014.hpp"
#include "gripper_config.hpp"
#include <cstdint>
//夹爪当前处于什么状态
enum class GripperState : uint8_t {
  Uninitialized,       //尚未具备控制条件
  Homing,              //向外寻找机械零点
  Ready,               //归零完成，等待动作请求
  Opening,             //正在打开
  Closing,             //正在闭合
  Holding,             //正在保持夹持状态
  Fault                //故障状态
};
//夹爪动作请求类型
enum class GripperAction : uint8_t {
  None,                //无动作请求
  Home,                //归零
  Open,                //打开
  MoveToWidth,         //移动到指定开口宽度
  Grip,                //夹取
  Release,             //释放
  Stop,                //停止
  ClearFault           //清除故障
};

enum class GripperClosingPhase : uint8_t {
     Approach,      //位置控制 + Kp/Kd  移动到预夹宽度
     Search         //速度控制 + Kd     低作用力寻找物块
};

// 三类故障可同时锁存，不增加其他故障类别。
enum GripperFault : uint8_t {
  GRIPPER_FAULT_NONE = 0,
  GRIPPER_FAULT_COMMUNICATION = 1U << 0,
  GRIPPER_FAULT_TEMPERATURE = 1U << 1,
  GRIPPER_FAULT_TORQUE = 1U << 2
};

enum class GripperResult : uint8_t {
  None,                //无结果
  Homed,               //归零完成
  Reached,             //目标宽度到位
  ContactStopped,      //接触停止
  NoObject,            //无物体
  HoldTargetReached,   //保持目标到达
  Stopped,             //停止
  Rejected             //被拒绝
};

struct GripperRequest {
  GripperAction action{GripperAction::None};
  float width_mm{0.0f};         // MoveToWidth 目标，或 Grip 预夹宽度，mm
  float hold_torque_nm{0.0f};   // Grip 幅值，N.m；0 显式选择配置中的保持力矩
};
//夹爪对外状态信息
struct GripperStatus {
  GripperState state{GripperState::Uninitialized};
  GripperClosingPhase closing_phase{GripperClosingPhase::Approach};
  GripperResult result{GripperResult::None};
  uint8_t faults{GRIPPER_FAULT_NONE};
  bool config_valid{false};           //init() 时检查配置是否合法
  bool homed{false};                  //是否归零成功
  bool motor_online{false};           // 电机是否在线
  bool contact_detected{false};       // 是否接触成功
  float home_position_rad{0.0f};      // 归零角度；只有 homed 后有效
  float actual_width_mm{0.0f};        // homed 后有效；失联时为最后一次测量
  float target_width_mm{0.0f};        
  float target_hold_torque_nm{0.0f};  // 幅值
  float ramped_hold_torque_nm{0.0f};  // 当前前馈力矩斜坡幅值，不是夹紧力测量
};

// 夹爪的核心控制器
class GripperController {
public:
  //init 初始化参数
  bool init(const GripperConfig &config);
  //request (接收动作请求)
  bool request(GripperAction action,
               float width_mm = 0.0f,
               float hold_torque_nm = 0.0f
              );  
  //执行状态机
  void update(const IM6014::Feedback &feedback,
              uint32_t feedback_sequence,
              uint32_t last_rx_ms, 
              uint32_t now_ms,
              IM6014::Command &command_out);
  //对外读取状态
  const GripperStatus &status() const { return status_; }

private:
 /*要实现的功能
  1 接收并处理动作请求
  2 根据反馈推进状态并生成电机命令
  3 统一状态切换
  4 连续接触确认
  5 判断当前故障
  6 输出轴角度->开口宽度
  7 开口宽度->输出轴角度 
 */
    // 动作与状态机
    bool requestValid(const GripperRequest& request) const;
    bool acceptRequest(const GripperRequest& request,
                       const IM6014::Feedback& feedback);

    void advance(const IM6014::Feedback& feedback, bool fresh, float dt_s);
    // 仅由 update() 调用，命令已在 update() 入口完整初始化。
    void makeCommand(IM6014::Command& command) const;

    void transition(GripperState state,
                    GripperResult result = GripperResult::None);

    // 接触与故障
    bool contactConfirmed(bool condition, bool fresh);
    uint8_t currentFaults(const IM6014::Feedback& feedback) const;

    // 机械换算
    float angleToWidth(float angle_rad) const;
    float widthToAngle(float width_mm) const;

    // 配置、请求、状态
    GripperConfig config_{};    //夹爪的机械尺寸和控制参数
    GripperRequest request_{};  //尚未处理的动作请求
    GripperStatus status_{};    //当前的状态 动作结果 故障 开口信息

    // 运行时数据
    bool clock_started_{false};       // 时间值 0 也有效，单独记录是否开始计时。
    bool feedback_seen_{false};       //是否曾经接收到有效电机反馈 
    bool auto_home_pending_{false};   // Stop 可取消尚未执行的启动归零。
    bool grip_mode_{false};           //区分closing 当前执行的是普通位置闭合 还是grip

    uint32_t started_ms_{0};             //用于启动计时基准
    uint32_t last_update_ms_{0};         //计算两次更新的实际时间差
    uint32_t last_feedback_sequence_{0}; //判断当前反馈不是新收到的
    uint32_t contact_count_{0};          //连续满足接触条件的新反馈帧数

    float reference_width_mm_{0.0f};     //保存当前平滑的位置参考
};
