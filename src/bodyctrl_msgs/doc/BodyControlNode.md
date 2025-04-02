# Body Control Node

提供机体信息上报及机体基础控制功能。

## 1. Published Topics

| 名称 | 参数 | 说明 |
| --- | --- | --- |
| ~/motor_state | [MotorStatusMsg](../msg/MotorStatusMsg.msg) | 发布当前电机状态，包括速度、位置、电流。 |
| ~/imu | [Imu](../msg/Imu.msg) | 发布机体的IMU数据。 |
| ~/power_status | [PowerStatus](../msg/PowerStatus.msg) | 发布机体的IMU数据。 |

## 2. Subcribed Topics

| 名称 | 参数 | 说明 |
| --- | --- | --- |
| ~/motor_ctrl | [CmdMotorCtrl](../msg/CmdMotorCtrl.msg) | 订阅力位混合模式控制指令。 |
| ~/set_motor_position | [CmdSetMotorPosition](../msg/CmdSetMotorPosition.msg) | 订阅伺服位置控制模式指令。 |
| ~/set_motor_speed | [CmdSetMotorSpeed](../msg/CmdSetMotorSpeed.msg) | 订阅伺服速度控制模式指令。 |
| ~/set_motor_cur_tor | [CmdSetMotorCurTor](../msg/CmdSetMotorCurTor.msg) | 订阅电流、力矩控制模式和刹车控制指令。 |

## 3. Services

| 名称 | 参数 | 说明 |
| --- | --- | --- |
| ~/init | [MotorInit](../srv/MotorInit.srv) | 初始化控制节点，返回识别到的子控制台数量。 |

## 4. Calls

暂无

## 5. Parameters

暂无