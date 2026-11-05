/**
 * @file com_config.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 全局通信配置，包含can设备、串口设备、协议解析等
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "com_config.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "stm32h7xx_hal_uart.h"
#include "task.h"

#include "Button.hpp"
#include "Canbus.hpp"
#include "GripperController.hpp"
#include "Motor.hpp"
#include "ROSCom.hpp"
#include "UartPort.hpp"
#include "UsbPort.hpp"
#include "XboxRemote.hpp"
#include "topics.hpp"
#include "topic_pool.h"
#include "usart.h"
#include <cstddef>
#include <cstdint>
#include <cstring>


osThreadId_t CAN1_Send_TaskHandle;
osThreadId_t CAN2_Send_TaskHandle;
osThreadId_t CAN3_Send_TaskHandle;
osThreadId_t im6014TaskHandle;
osThreadId_t uart3ProcessTaskHandle;
osThreadId_t usbcdcProcessTaskHandle;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

CanBus fdcan1_bus(hfdcan1);
CanBus fdcan2_bus(hfdcan2);
CanBus fdcan3_bus(hfdcan3);

// can设备

// 底盘电机
C620Motor chassis_motor1(&fdcan3_bus, 0x201, 0, 0x200, 0);
C620Motor chassis_motor2(&fdcan3_bus, 0x202, 0, 0x200, 0);
C620Motor chassis_motor3(&fdcan3_bus, 0x203, 0, 0x200, 0);
C620Motor chassis_motor4(&fdcan3_bus, 0x204, 0, 0x200, 0);

//
C610Motor arm2006_motor(&fdcan2_bus, 0x205, 0, 0x1FF, 0);
C620Motor arm3508_motor(&fdcan2_bus, 0x206, 0, 0x1FF, 0);
DM4310Motor arm4310_motor(&fdcan2_bus, 0x301, 0, 0x01, 0,
                         DM4310Motor::PosWithSpeed);

// 串口外设（回调+信号量唤醒处理线程进行解包）
void onUart3RxCb(const uint8_t *data, size_t len, void *user);

void onUsbRxCb(const uint8_t *data, size_t len, void *user);

extern UART_HandleTypeDef huart3;

DMA_BUFFER_ATTR static uint8_t uart3_rx_dma[64];
DMA_BUFFER_ATTR static uint8_t uart3_tx_dma[64];
UartPort uart3_port(&huart3, uart3_rx_dma, sizeof(uart3_rx_dma), uart3_tx_dma,
                    sizeof(uart3_tx_dma), onUart3RxCb, nullptr);
osSemaphoreId_t uart3_rx_semphore = NULL;

//IM6014
struct IM6014Status {
  HAL_StatusTypeDef rx_status{HAL_ERROR};  // RX DMA 启动结果
  HAL_StatusTypeDef tx_status{HAL_ERROR};  // 最近一次阻塞发送结果
  bool encode_ok{false};
  bool awaiting_reply{false};
  uint32_t tx_count{0};
  uint32_t rx_count{0};                   // 有效反馈序号，同时供控制器判断新帧
  uint32_t last_tx_ms{0};
  uint32_t last_rx_ms{0};
};

// Ozone 展开 im6014[0]
struct IM6014Device {
  IM6014::Feedback IM6014_core{};
  IM6014::Command IM6014_control{};
  IM6014Status status{};
  IM6014::StreamParser parser{0};
  GripperController gripper{};
};

// IM6014 复用 USART2 / RS485_1，只有 im6014Task 读写这个端口。
IM6014Device im6014[1];
DMA_BUFFER_ATTR static uint8_t uart2_rx_dma[UartPort::kPacketPayloadSize];
static UartPort uart2_port(&huart2, uart2_rx_dma, sizeof(uart2_rx_dma));

// Xbox控制器（基于uart3）
XboxRemote xbox_remote(uart3_port);
TypedTopicPublisher<pub_Xbox_Data> xbox_data_pub("xbox");
pub_Xbox_Data xbox_msg;

// usb
osSemaphoreId_t usbcdc_rx_semphore = NULL;
ROSProtocol ros_protocol(nullptr, &UsbPort::Instance());

uint8_t comServiceInit() {
  // can外设初始化
  canFilterInit(&hfdcan1, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
  canFilterInit(&hfdcan1, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
  bspCanInit(&hfdcan1);
  canFilterInit(&hfdcan2, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
  canFilterInit(&hfdcan2, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
  bspCanInit(&hfdcan2);
  canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
  canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
  bspCanInit(&hfdcan3);

  // can 总线初始化
  fdcan1_bus.init();
  fdcan2_bus.init();
  fdcan3_bus.init();

  chassis_motor1.init();
  chassis_motor2.init();
  chassis_motor3.init();
  chassis_motor4.init();

  arm2006_motor.init();
  arm3508_motor.init();
  arm4310_motor.init();

  fdcan3_bus.registerDevice(&chassis_motor1);
  fdcan3_bus.registerDevice(&chassis_motor2);
  fdcan3_bus.registerDevice(&chassis_motor3);
  fdcan3_bus.registerDevice(&chassis_motor4);


  fdcan2_bus.registerDevice(&arm2006_motor);
  fdcan2_bus.registerDevice(&arm3508_motor);
  fdcan2_bus.registerDevice(&arm4310_motor);

  // 串口外设
  uart3_rx_semphore = osSemaphoreNew(1, 0, NULL);
  uart3_port.startRxDmaIdle();

  // Xbox控制器初始化
  xbox_remote.init();

  // usb 外设
  usbcdc_rx_semphore = osSemaphoreNew(1, 0, NULL);
  ros_protocol.init();
  UsbPort::Instance().SetRxCallback(onUsbRxCb, NULL);
  return 0;
}

void onUart3RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && uart3_rx_semphore != NULL) {
    (void)osSemaphoreRelease(uart3_rx_semphore);
  }
}

void onUsbRxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && usbcdc_rx_semphore != NULL) {
    (void)osSemaphoreRelease(usbcdc_rx_semphore);
  }
}

void can1SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();

  for (;;) {

    vTaskDelayUntil(&currentTime, 1); // 每1ms执行一次发送任务
  }
}

void can2SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();
  CanBus::ClassicPack pack;
  pack.type = CanBus::Type::STANDARD;

  uint8_t len = 8;
  const uint32_t arm_motor_ids[4] = {0x205, 0x206, 0x207, 0x208};
  for (;;) {
    pack.id = 0x1FF; // DJI Group 2
    // 当前仅有 0x201(arm2006) 和 0x203(arm3508)，其余槽位置 0
    int16_t commands[4] = {0};

    // arm motor
    commands[0] = static_cast<int16_t>(arm2006_motor.cmdTrans()); // 0x201
    commands[1] = static_cast<int16_t>(arm3508_motor.cmdTrans()); // 0x203
    commands[2] = static_cast<int16_t>(0); // 0x203
    commands[3] = static_cast<int16_t>(0); // 0x204
    packDJIMotorCanMsg(pack.id, arm_motor_ids, commands, 4, pack.data, len);
    // fdcan2_bus.addCanMsg(pack);

    vTaskDelayUntil(&currentTime, 1); // 每1ms执行一次发送任务
  }
}


void can3SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();
  CanBus::ClassicPack pack;
  pack.type = CanBus::Type::STANDARD;

  uint8_t len = 8;  
  const uint32_t chassis_motor_ids[4] = {0x201, 0x202, 0x203, 0x204};

  for (;;) {
    // 一帧固定打包 4 个槽位：0x201~0x204
    pack.id = 0x200; // DJI Group 2

    // 当前仅有 0x201(arm2006) 和 0x203(arm3508)，其余槽位置 0
    int16_t commands[4] = {0};
    commands[0] = static_cast<int16_t>(chassis_motor1.cmdTrans()); // 0x201
    commands[1] = static_cast<int16_t>(chassis_motor2.cmdTrans()); // 0x202
    commands[2] = static_cast<int16_t>(chassis_motor3.cmdTrans()); // 0x203   
    commands[3] = static_cast<int16_t>(chassis_motor4.cmdTrans()); // 0x204
    packDJIMotorCanMsg(pack.id, chassis_motor_ids, commands, 4, pack.data, len);
    // arm3508_motor.manager_->addCanMsg(pack);
    fdcan3_bus.addCanMsg(pack);
    vTaskDelayUntil(&currentTime, 1); // 每1ms执行一次发送任务
  }
}
// 夹爪任务
void im6014Task(void *argument) { // uart2：IM6014 / RS485_1
  (void)argument;
  auto &motor = im6014[0];
  // 默认配置中的待配置项会让控制器保持 UNINITIALIZED、输出 Stop。
  (void)motor.gripper.init(GripperConfig{});
  motor.status.rx_status = uart2_port.startRxDmaIdle();
  uint8_t tx_frame[IM6014::kTxSize]{};
  UartPort::Packet packet{};
  TickType_t wake_tick = xTaskGetTickCount();

  for (;;) {
    uint32_t action;
    if (xTaskNotifyWait(0, UINT32_MAX, &action, 0) == pdTRUE) {
      motor.gripper.request(static_cast<GripperAction>(action), 350.0f);
    }
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t rx_before = motor.status.rx_count;
    // 每轮最多处理一队列数据，随后继续控制更新和发送。
    for (std::size_t n = 0; n < UartPort::kRxQueueDepth && uart2_port.Read(packet); ++n) {
      for (uint16_t i = 0; i < packet.len; ++i) {
        if (!motor.parser.feed(packet.data[i], motor.IM6014_core)) continue;
        ++motor.status.rx_count;
        motor.status.last_rx_ms = now_ms;
        motor.status.awaiting_reply = false;
        // 每个有效帧更新一次，保留连续接触确认。
        motor.gripper.update(motor.IM6014_core, motor.status.rx_count,
                             motor.status.last_rx_ms, now_ms,
                             motor.IM6014_control);
      }
    }
    // 本轮没有有效帧时，也要处理失联、动作请求和运动斜坡。
    if (motor.status.rx_count == rx_before)
      motor.gripper.update(motor.IM6014_core, motor.status.rx_count,
                           motor.status.last_rx_ms, now_ms,
                           motor.IM6014_control);
    // 单次回复等待和阻塞发送超时均为 5 ms。
    if (now_ms - motor.status.last_tx_ms >= 5)
      motor.status.awaiting_reply = false;
    if (!motor.status.awaiting_reply) {
      if (motor.IM6014_core.timed_out) {
        motor.IM6014_control = IM6014::Command{}; // 先发送 Stop。
        motor.IM6014_control.timeout_enabled = false; // 按协议清除超时状态。
      }
      motor.status.encode_ok = IM6014::encodeCommand(
          motor.IM6014_control, tx_frame, sizeof(tx_frame));
      if (motor.status.encode_ok) {
        motor.status.tx_status = uart2_port.write(tx_frame, sizeof(tx_frame), 5);
        if (motor.status.tx_status == HAL_OK) {
          ++motor.status.tx_count;
          motor.status.last_tx_ms = HAL_GetTick();
          motor.status.awaiting_reply = true;
        }
      }
    }
    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(1)); // 每 1 ms 调度一次。
  }
}

static void sendGripperAction(GripperAction action) {
  if (im6014TaskHandle != nullptr) {
    xTaskNotify(static_cast<TaskHandle_t>(im6014TaskHandle),
                static_cast<uint32_t>(action), eSetValueWithOverwrite);
  }
}
// XBOX遥控任务
void uart3RxProcessTask(void *argument) {
  (void)argument;
  if(!xbox_data_pub.IsValid()) {
    return;
  }
  Button lb_button;
  Button rb_button;

  for (;;) {
    (void)osSemaphoreAcquire(uart3_rx_semphore, osWaitForever);

    UartPort::Packet packet{};
    while (uart3_port.Read(packet)) {
      // 逐字节送进Xbox协议解析器
      for (uint16_t i = 0; i < packet.len; ++i) {
        uint8_t frame_id = xbox_remote.processByte(packet.data[i]);
        if (frame_id != 0) {
          // 帧解析完成，可以在这里获取控制器数据并做业务处理
          const auto &ctrl_data = xbox_remote.getControllerData();
          xbox_msg.btnY = ctrl_data.btnY;
          xbox_msg.btnA = ctrl_data.btnA;
          xbox_msg.btnLB = ctrl_data.btnLB;
          xbox_msg.btnRB = ctrl_data.btnRB;
          xbox_msg.trigLT = ctrl_data.trigLT;
          xbox_msg.trigRT = ctrl_data.trigRT;
          xbox_msg.btnDirUp = ctrl_data.btnDirUp;
          xbox_msg.btnDirDown = ctrl_data.btnDirDown;
          xbox_msg.btnDirLeft = ctrl_data.btnDirLeft;
          xbox_msg.btnDirRight = ctrl_data.btnDirRight;
          xbox_msg.btnB = ctrl_data.btnB;
          xbox_msg.btnX = ctrl_data.btnX;
          xbox_msg.joyLHori = ctrl_data.joyLHori;
          xbox_msg.joyLVert = ctrl_data.joyLVert;
          xbox_msg.joyRHori = ctrl_data.joyRHori;
          xbox_msg.joyRVert = ctrl_data.joyRVert;
          xbox_data_pub.Publish(xbox_msg);
          //短按和长按逻辑加入
          const uint32_t now_ms = HAL_GetTick();
          //LB RB 负责夹爪控制
          const auto lb = lb_button.update(ctrl_data.btnLB, now_ms);
          const auto rb = rb_button.update(ctrl_data.btnRB, now_ms);
          if (rb.long_press) {
            sendGripperAction(GripperAction::Home);
          } else if (rb.short_press) {
            sendGripperAction(GripperAction::Release);
          } else if (lb.short_press && !ctrl_data.btnRB) {
            sendGripperAction(GripperAction::Grip);
          }
        }
      }
    }
  }
}


void usbCdcProcessTask(void *argument) {

  (void)argument;

  for (;;) {
    (void)osSemaphoreAcquire(usbcdc_rx_semphore, osWaitForever);

    UsbPort::Packet packet{};
    while (UsbPort::Instance().Read(packet)) {
      // 逐个字节解析
      for (uint16_t i = 0; i < packet.len; ++i) {
        uint8_t frame_id = ros_protocol.processData(packet.data[i]);
        if (frame_id != 0) {
          uint8_t rev[64] = {0};
          memcpy(rev, ros_protocol.getSensorBagData().i16_data,
                 sizeof(ros_protocol.getSensorBagData().i16_data));
          memcpy(rev + sizeof(ros_protocol.getSensorBagData().i16_data),
                 ros_protocol.getSensorBagData().f_data,
                 sizeof(ros_protocol.getSensorBagData().f_data));
          UsbPort::Instance().WriteAsync(
              rev, sizeof(ros_protocol.getSensorBagData()));
        }
      }
    }
  }
}
