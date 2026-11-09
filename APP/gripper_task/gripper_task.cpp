#include "gripper_task.h"
#include "UartPort.hpp"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "usart.h"

osThreadId_t im6014TaskHandle;

namespace {
GripperController controller;
GripperRequest pending_request{};
IM6014::Feedback feedback{};
IM6014::Command command{};
IM6014::StreamParser parser{0};
DMA_BUFFER_ATTR uint8_t rx_dma[UartPort::kPacketPayloadSize];
UartPort motor_port(&huart2, rx_dma, sizeof(rx_dma));

void updateGripper(bool fresh, uint32_t now_ms) {
  // 与状态快照读取互斥，串口收发在临界区外。
  taskENTER_CRITICAL();
  controller.update(feedback, fresh, now_ms, command);
  taskEXIT_CRITICAL();
}
}

void gripperRequest(GripperAction action, float width_mm, float hold_torque_nm) {
  taskENTER_CRITICAL();
  pending_request = {action, width_mm, hold_torque_nm};
  taskEXIT_CRITICAL();
}

GripperStatus gripperStatus() {
  taskENTER_CRITICAL();
  auto status = controller.status();
  // 新请求尚未执行时，不把上一动作的完成结果交给自动流程。
  if (pending_request.action != GripperAction::None) status.result = GripperResult::None;
  taskEXIT_CRITICAL();
  return status;
}

void im6014Task(void *argument) { // USART2 / RS485_1，单任务拥有协议与控制器。
  (void)argument;
  taskENTER_CRITICAL();
  controller.init(GripperConfig{});
  taskEXIT_CRITICAL();
  motor_port.startRxDmaIdle();
  uint8_t tx_frame[IM6014::kTxSize]{};
  UartPort::Packet packet{};
  bool awaiting_reply = false;
  uint32_t last_tx_ms = 0;
  TickType_t wake_tick = xTaskGetTickCount();

  for (;;) {
    taskENTER_CRITICAL();
    const auto request = pending_request;
    pending_request = {};
    if (request.action != GripperAction::None)
      controller.request(request.action, request.width_mm, request.hold_torque_nm);
    taskEXIT_CRITICAL();

    const uint32_t now_ms = HAL_GetTick();
    bool received = false;
    for (std::size_t n = 0; n < UartPort::kRxQueueDepth && motor_port.Read(packet); ++n) {
      for (uint16_t i = 0; i < packet.len; ++i) {
        if (!parser.feed(packet.data[i], feedback)) continue;
        received = true;
        awaiting_reply = false;
        updateGripper(true, now_ms); // 每个新帧各自参与接触确认。
      }
    }
    if (!received) updateGripper(false, now_ms);

    // 一次等待一帧反馈，5 ms 未回复则开始下一次发送。
    if (now_ms - last_tx_ms >= 5) awaiting_reply = false;
    if (!awaiting_reply) {
      if (feedback.timed_out) {
        command = IM6014::Command{};
        command.timeout_enabled = false; // Stop 清除电机超时位，恢复后继续正常命令。
      }
      if (IM6014::encodeCommand(command, tx_frame, sizeof(tx_frame)) &&
          motor_port.write(tx_frame, sizeof(tx_frame), 5) == HAL_OK) {
        last_tx_ms = HAL_GetTick();
        awaiting_reply = true;
      }
    }
    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(1));
  }
}
