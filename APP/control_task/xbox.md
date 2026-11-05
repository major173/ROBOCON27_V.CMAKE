# Xbox 按键映射

底盘链路：ESP32 → USART3 → `XboxRemote` → `xbox` topic → APP。
夹爪链路：USART3 按键事件 → 任务通知 → `im6014Task` → `gripper.request()`。
“已接入”表示当前代码已使用，实物效果需上板验证。

| 按键 / 摇杆 | 数据字段 | 功能 / 约定 | 当前使用情况 |
| --- | --- | --- | --- |
| **LB 短按** | `btnLB` | **夹紧 `Grip`**：预夹 350 mm → 搜索接触 → 保持 | 已接入，保持力矩使用配置值 |
| **RB 短按** | `btnRB` | **张开 `Release`**：按 Moving 参数打开至最大开口 | 已接入 |
| **RB 长按 3 秒** | `btnRB` | **归零 `Home`**：按 Homing 参数向外归零 | 已接入 |
| 左摇杆横轴 | `joyLHori` | 底盘 `linear_x_` | 已接入 |
| 左摇杆纵轴 | `joyLVert` | 底盘 `linear_y_`，取反映射 | 已接入 |
| 右摇杆横轴 | `joyRHori` | 底盘 `omega_` | 已接入 |
| 右摇杆纵轴 | `joyRVert` | 未分配 | 已接收发布，未使用 |
| A | `btnA` | 未分配 | 已接收发布，未使用 |
| Y | `btnY` | 旧调试：控制 PF9 电平 | 当前未启用 |
| X / B | `btnX` / `btnB` | 旧调试：2006 速度目标 +300 / −300 | 当前未启用 |
| 方向上 / 下 | `btnDirUp` / `btnDirDown` | 旧调试：舵机角度每帧 +0.1° / −0.1° | 当前未启用 |
| 方向左 / 右 | `btnDirLeft` / `btnDirRight` | 未分配 | 已接收发布，未使用 |
| LT / RT | `trigLT` / `trigRT` | 未分配 | 已接收发布，未使用 |
| 左 / 右摇杆按下 | `btnLS` / `btnRS` | 未分配 | Module 已解析，未发布到 APP topic |
| Share / Start / Select / Xbox | `btnShare` / `btnStart` / `btnSelect` / `btnXbox` | 未分配 | Module 已解析，未发布到 APP topic |

- 底盘摇杆中值 `32767`、死区 `±2000`，映射系数 `MAX_VELOCITY = 3.0f`；平移单位 m/s，旋转单位 rad/s。
- 短按在不足 3 秒松手时触发；长按在首次收到已按满 3 秒的新数据时触发一次，继续按住或松手不再触发短按。LB 长按未绑定动作。
- 同帧 RB 事件优先；按住 RB 时忽略 LB 短按。任务通知只保存最新待处理动作；归零、在线和故障条件由夹爪控制器判断。
- 通用检测见 [Button.hpp](../../Module/xbox/Button.hpp)，其他布尔按键可复用。当前简版按连续正常手柄数据计时，未加入断流取消或消抖。
- 旧调试映射位于 `debug_task.cpp`，受 `DEBUG_ARM` 控制，当前 CMake 未启用。旧 LB/RB 对应 3508 速度目标 −50 / +50；重新启用旧调试前须解除该重复绑定。
- Ozone 可展开 `xbox_msg` 查看已发布的按键和摇杆值。

依据：[控制映射](control_task.cpp)、[接收发布](../robot_com/com_config.cpp)、[旧调试映射](../debug_task/debug_task.cpp)。
