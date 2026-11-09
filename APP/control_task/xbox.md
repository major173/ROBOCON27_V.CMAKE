# Xbox 按键映射

ESP32 → USART3 → `XboxRemote`；夹爪按键提交 `gripperRequest()`，传送带经 `xbox` topic / `controlTask` 提交 `gripperBeltMove()`。

| 按键 / 摇杆 | 数据字段 | 当前功能 |
| --- | --- | --- |
| LB 短按 | `btnLB` | 夹取：预夹 350 mm → 搜索接触 → 保持 |
| RB 短按 | `btnRB` | 松开：按 Moving 参数打开至 412 mm |
| RB 长按 3 秒 | `btnRB` | 按 Homing 参数向外归零 |
| Y 按住 | `btnY` | 两条传送带上移，松手保持 |
| A 按住 | `btnA` | 两条传送带下移，松手保持 |
| 左摇杆横轴 | `joyLHori` | 底盘 `linear_x_` |
| 左摇杆纵轴 | `joyLVert` | 底盘 `linear_y_`，取反映射 |
| 右摇杆横轴 | `joyRHori` | 底盘 `omega_` |
| 右摇杆纵轴 | `joyRVert` | 已发布，未使用 |
| X / B、方向键 | `btnX` / `btnB` / `btnDir*` | 已发布，未使用 |
| LT / RT | `trigLT` / `trigRT` | 已发布，未使用 |
| 摇杆按下、Share / Start / Select / Xbox | `btnLS` / `btnRS` 等 | Module 已解析，未发布到 APP topic |

- 短按在不足 3 秒松手时触发；长按满 3 秒触发一次，松手不再产生短按。LB 长按未绑定。
- 同帧 RB 事件优先；按住 RB 时忽略 LB 短按。
- Y/A 同时按下视为保持。只在方向变化时提交请求，重复的中立数据不会覆盖自动流程命令。
- 无须选择 Xbox/Ozone 来源或修改使能变量；第一次 Y/A 请求启动两路闭环，之后松手保持各自当前位置。
- 底盘摇杆中值 32767，死区 ±2000，映射系数 `MAX_VELOCITY=3.0f`。`xbox_msg` 可用于观察按键数据。

控制参数、动作结果与自动流程调用见 [夹爪控制](../gripper_task/夹爪控制.md)。通用短按/长按检测见 [Button.hpp](../../Module/xbox/Button.hpp)。
