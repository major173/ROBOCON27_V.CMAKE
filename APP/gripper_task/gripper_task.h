#pragma once

#include "GripperController.hpp"

void gripperRequest(GripperAction action,
                    float width_mm = GRIPPER_PRE_WIDTH_MM,
                    float hold_torque_nm = 0.0f);
GripperStatus gripperStatus(); // 当前动作状态的快照

enum class BeltDirection : int8_t { Down = -1, Hold = 0, Up = 1 };
// 上移/下移持续执行，Hold 锁存当前位置并保持；不是物体距离控制。
void gripperBeltMove(BeltDirection direction);

void im6014Task(void *argument); // USART2 / RS485_1
void beltTask(void *argument);  // CAN2 两侧传送带
