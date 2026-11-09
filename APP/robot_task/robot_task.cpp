/**
 * @file robot_task.h
 * @author 大帅将军 ，Keten (2863861004@qq.com)
 * @brief 任务管理和任务间通讯
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
/* RTOS层及mcu main接口 */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "task.h"

/* bsp 层接口头文件 */
#include "bsp_dwt.h"
#include "chassis_task.h"
#include "com_config.h"
#include "control_task.h"
#include "gripper_task.h"

/* module层接口头文件 */

/* Definitions for TaskHand */
extern osThreadId_t CAN1_Send_TaskHandle;
extern osThreadId_t CAN2_Send_TaskHandle;
extern osThreadId_t CAN3_Send_TaskHandle;
extern osThreadId_t im6014TaskHandle;
extern osThreadId_t uart3ProcessTaskHandle;
extern osThreadId_t BeltTaskHandle;
extern osThreadId_t ChassisTaskHandle;
extern osThreadId_t ControlTaskHandle;
extern osThreadId_t usbcdcProcessTaskHandle;

void osTaskInit(void) {
  // 一个任务独占 IM6014 通信及夹爪状态机，避免跨任务竞争电机命令。
  const osThreadAttr_t im6014TaskHandle_attributes = {
      .name = "IM6014_Gripper",
      .stack_size = 512 * 4,
      .priority = (osPriority_t)osPriorityNormal1,
  };
  im6014TaskHandle =
      osThreadNew(im6014Task, NULL, &im6014TaskHandle_attributes);

  const osThreadAttr_t CAN1_SendTaskHandle_attributes = {
      .name = "CAN1_Send_TaskHandle",
      .stack_size = 256 * 4, // 1024 字节，容纳 CAN 发送调用链和任务切换现场
      .priority = (osPriority_t)osPriorityNormal,
  };
  CAN1_Send_TaskHandle =
      osThreadNew(can1SendTask, NULL, &CAN1_SendTaskHandle_attributes);

  const osThreadAttr_t CAN2_SendTaskHandle_attributes = {
      .name = "CAN2_Send_TaskHandle",
      .stack_size = 256 * 4, // 1024 字节，容纳 CAN 发送调用链和任务切换现场
      .priority = (osPriority_t)osPriorityNormal,
  };
  CAN2_Send_TaskHandle =
      osThreadNew(can2SendTask, NULL, &CAN2_SendTaskHandle_attributes);

  const osThreadAttr_t CAN3_SendTaskHandle_attributes = {
      .name = "CAN3_Send_TaskHandle",
      .stack_size = 256 * 4, // 1024 字节，容纳 CAN 发送调用链和任务切换现场
      .priority = (osPriority_t)osPriorityNormal,
  };
  CAN3_Send_TaskHandle =
      osThreadNew(can3SendTask, NULL, &CAN3_SendTaskHandle_attributes);

  const osThreadAttr_t BeltTaskHandle_attributes = {
      .name = "Gripper_Belts",
      .stack_size = 256 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  BeltTaskHandle = osThreadNew(beltTask, NULL, &BeltTaskHandle_attributes);

  const osThreadAttr_t ChassisTaskHandle_attributes = {
      .name = "Chassis_TaskHandle",
      .stack_size = 256 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  ChassisTaskHandle =
      osThreadNew(chassisTask, NULL, &ChassisTaskHandle_attributes);

  const osThreadAttr_t ControlTaskHandle_attributes = {
      .name = "Control_TaskHandle",
      .stack_size = 256 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  ControlTaskHandle =
      osThreadNew(controlTask, NULL, &ControlTaskHandle_attributes);

  const osThreadAttr_t Uart3ProcessTaskHandle_attributes = {
      .name = "Uart3Process_TaskHandle",
      .stack_size = 256 * 4,
      .priority = (osPriority_t)osPriorityNormal1,
  };
  uart3ProcessTaskHandle =
      osThreadNew(uart3RxProcessTask, NULL, &Uart3ProcessTaskHandle_attributes);

  const osThreadAttr_t UsbcdcProcessTaskHandle_attributes = {
      .name = "UsbcdcProcess_TaskHandle",
      .stack_size = 256 * 4,
      .priority = (osPriority_t)osPriorityNormal1,
  };
  usbcdcProcessTaskHandle =
      osThreadNew(usbCdcProcessTask, NULL, &UsbcdcProcessTaskHandle_attributes);
}
