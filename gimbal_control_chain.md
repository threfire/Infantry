# 平衡步兵云台控制链路梳理

本文用于学习当前云台控制代码，并作为后续移植到新工程时的链路索引。代码主体位于 `Task/gimbal_task.c`、`application/gimbal_behaviour.c`、`components/controller/pid.c`、`application/CAN_receive.c`。

## 1. 总体链路

系统启动后，`Src/main.c` 先完成外设初始化，再调用 `can_filter_init()`、`remote_control_init()`、`user_usart_init()`，随后进入 `MX_FREERTOS_Init()` 创建任务。`gimbal_task` 以 1 ms 周期运行，`INS_task` 实时更新姿态，`detect_task` 维护离线状态，`shoot_task` 与云台共享视觉和发射状态。

控制链路为：

```text
遥控/键鼠/图传输入 + 视觉自瞄输入 + IMU 姿态 + CAN 电机反馈
        -> gimbal_feedback_update()
        -> gimbal_behaviour_mode_set()
        -> gimbal_behaviour_control_set()
        -> gimbal_set_control()
        -> gimbal_control_loop()
        -> CAN_cmd_gimbal_yaw()/CAN_cmd_gimbal_pitch()
        -> DM4310 云台电机
```

关键入口：

- `gimbal_task()`：云台主循环，见 `Task/gimbal_task.c:46`。
- `gimbal_init()`：电机指针、IMU 指针、限位、PID 参数初始化，见 `Task/gimbal_task.c:115`。
- `gimbal_feedback_update()`：姿态和电机反馈统一成云台控制变量，见 `Task/gimbal_task.c:155`。
- `gimbal_set_control()`：生成 yaw/pitch 目标角，见 `Task/gimbal_task.c:189`。
- `gimbal_control_loop()`：按模式选择控制器，见 `Task/gimbal_task.c:284`。
- `CAN_cmd_gimbal_yaw()`、`CAN_cmd_gimbal_pitch()`：扭矩打包下发，见 `application/CAN_receive.c:303`、`application/CAN_receive.c:360`。

## 2. 初始化流程

`main()` 的初始化顺序：

```text
HAL/CubeMX 外设初始化
-> CAN 滤波与中断启动
-> 遥控接收 DMA 初始化
-> USART1 视觉通信 DMA 初始化
-> FreeRTOS 任务创建
-> 调度器启动
```

相关代码：

- CAN1/CAN2 启动和 FIFO0 接收中断：`boards/bsp_can.c:8`。
- 遥控/图传串口初始化：`Src/main.c:134`。
- 视觉串口初始化：`Src/main.c:135`。
- 云台任务创建：`Src/freertos.c:147`。
- IMU 任务创建：`Src/freertos.c:163`。

`gimbal_init()` 完成以下绑定：

- `yaw_motor_measure_point()`、`pitch_motor_measure_point()` 取得 DM4310 电机反馈指针。
- `get_INS_angle_point()`、`get_gyro_data_point()`、`get_accel_data_point()` 取得 IMU 数据指针。
- yaw 零偏 `-0.02f`，pitch 零偏 `2.678f`。
- yaw 相对角限位为 `[-2π, 2π]`，pitch 相对角限位为 `[-0.3, 0.4]`。
- 初始化 yaw/pitch 的绝对角度环 PID、绝对速度环 PID、自瞄 PID、相对角度 PID。

云台任务启动后先延时 201 ms，再进入电机在线和使能等待循环。等待期间调用 `gimbal_motor_wake()` 发送 DM4310 清错、使能或零扭矩命令。

## 3. 数据来源

### 3.1 IMU 姿态链路

`INS_task` 读取 BMI088 陀螺仪和加速度计，通过 AHRS 更新四元数，再计算欧拉角。云台读取的是指针形式的 `INS_angle`、`INS_gyro`、`INS_accel`。

关键代码：

- `INS_task()`：`Task/INS_task.c:187`。
- `AHRS_update()`：`Task/INS_task.c:273`。
- `get_angle()`：`Task/INS_task.c:284`。
- `get_INS_angle_point()`：`Task/INS_task.c:435`。
- `get_gyro_data_point()`：`Task/INS_task.c:445`。
- `get_accel_data_point()`：`Task/INS_task.c:455`。

`gimbal_feedback_update()` 对 IMU 坐标做了重映射：

```c
ins.yaw   = -INS_Angle[0];
ins.pitch =  INS_Angle[2];
ins.roll  = -INS_Angle[1];
ins.wx    = -INS_Gyro[1];
ins.wy    =  INS_Gyro[0];
ins.wz    = -INS_Gyro[2];
```

pitch 绝对角使用 `ins.pitch`，pitch 绝对速度使用 `ins.wy`。yaw 绝对角使用 `ins.yaw`，yaw 绝对速度使用 pitch 相对角修正后的投影：

```c
yaw_absolute_speed =
    cos(pitch_relative_angle) * ins.wz
  + sin(pitch_relative_angle) * ins.wx;
```

该投影用于补偿 pitch 俯仰后陀螺仪坐标轴与真实 yaw 轴的夹角。

### 3.2 云台电机反馈链路

CAN 中断在 `HAL_CAN_RxFifo0MsgPendingCallback()` 中解析电机反馈：

- CAN1 标识符 `0x05`：yaw DM4310 反馈，写入 `motor_gimbal[0]`。
- CAN2 标识符 `0x06`：pitch DM4310 反馈，写入 `motor_gimbal[1]`。
- 反馈字段包括 `Status`、`Position`、`Speed`、`Torque`。
- 每次有效反馈调用 `detect_hook(YAW_GIMBAL_MOTOR_TOE)` 或 `detect_hook(PITCH_GIMBAL_MOTOR_TOE)`。

DM4310 反馈转换范围定义在 `application/CAN_receive.h`：

```c
Position: [-3.14, 3.14] rad
Speed:    [-30, 30] rad/s
Torque:   [-10, 10] N*m
```

云台内部变量转换：

```c
relative_angle = -rad_format(motor.Position - offset_angle);
relative_speed = -motor.Speed;
absolute_angle = IMU angle;
absolute_speed = IMU gyro projection;
```

### 3.3 遥控/键鼠/图传链路

`remote_control_init()` 启动 USART3 DMA 接收。`PT_link_en` 为 1 时使用图传链路数据结构 `remote_data`，帧长 21 字节；为 0 时使用传统 DBUS `rc_ctrl`，帧长 18 字节。

关键代码：

- `remote_control_init()`：`application/remote_control.c:69`。
- `USART3_IRQHandler()`：`application/remote_control.c:78`。
- 图传解包 `sbus_to_pt()`：`application/remote_control.c:202`。
- DBUS 解包 `sbus_to_rc()`：`application/remote_control.c:248`。

当前工程 `User/Inc/config.h` 中 `PT_link_en` 为 1，因此主要输入来自 `remote_data`：

- `ch_0/ch_1`：底盘平移。
- `ch_2/ch_3`：云台 pitch/yaw 摇杆。
- `mode_sw`：模式开关。
- `pause/fn_1/fn_2/trigger`：图传按键。
- `mouse_x/mouse_y/mouse_left/mouse_right/mouse_middle`：键鼠控制。
- `key`：键盘位图。

### 3.4 视觉自瞄链路

USART1 与上位机通信。云台发送自身姿态和弹速，上位机回传目标 yaw/pitch。

关键代码：

- `user_usart_init()`：`application/USART_receive.c:25`。
- `USART1_IRQHandler()`：`application/USART_receive.c:30`。
- `user_data_pack_handle()`：`application/USART_receive.c:82`。
- `user_data_solve()`：`application/USART_receive.c:114`。

发送帧 `user_send_data_t` 包含：

- 四元数 `q[4]`
- yaw、yaw_vel
- pitch、pitch_vel
- bullet_speed
- bullet_count

接收帧 `auto_shoot_t` 解析：

```c
auto_shoot.mode  = buf[2];
auto_shoot.yaw   = -received_yaw;
auto_shoot.pitch = -received_pitch;
auto_shoot.NUC_GG_Detect = 0;
```

`NUC_GG_Detect` 在 `gimbal_set_control()` 中每 1 ms 自增，上限 600。自瞄数据在 `NUC_GG_Detect <= 550` 时参与控制。

## 4. 模式机

模式枚举位于 `Task/gimbal_task.h:57`：

```c
GIMBAL_ZERO_FORCE
GIMBAL_ABSOLUTE_ANGLE
GIMBAL_AUTO
GIMBAL_RETURN
GIMBAL_OPERATION
```

模式设置函数为 `gimbal_behaviour_mode_set()`，内部调用 `gimbal_behavour_set()`。

当前有效模式链路：

- 图传/遥控开关上拨：进入 `GIMBAL_OPERATION`；`Gimbal_Auto_Debug=1` 时进入 `GIMBAL_AUTO`。
- 图传/遥控开关中位：进入 `GIMBAL_ZERO_FORCE`；`fn_2`、`ZeroToFollowFlag` 或底盘站立状态触发后进入 `GIMBAL_ABSOLUTE_ANGLE`。
- 图传/遥控开关下拨：进入 `GIMBAL_ZERO_FORCE`。
- DBUS 离线：进入 `GIMBAL_ZERO_FORCE`。

`GIMBAL_RETURN` 有控制分支，当前工程内未看到模式设置入口；移植时可按保留功能处理。

## 5. 目标生成

`gimbal_behaviour_control_set()` 根据模式生成 `add_yaw_angle` 和 `add_pitch_angle`：

- `GIMBAL_ZERO_FORCE`：增量为 0。
- `GIMBAL_ABSOLUTE_ANGLE`：遥控摇杆或鼠标生成角度增量。
- `GIMBAL_AUTO`：摇杆生成小增量，同时允许视觉目标覆盖。
- `GIMBAL_OPERATION`：默认鼠标生成角度增量；按下右键时使用自瞄控制分支；按下 Z 时输出零增量。

`gimbal_set_control()` 将增量转换为目标角：

```text
手动链路：absolute_angle_set = 当前 absolute_angle + add_angle
自瞄链路：absolute_angle_set = auto_shoot.yaw / auto_shoot.pitch
回中链路：目标角跟随当前角或指定 pitch 角
```

pitch/yaw 限位由 `gimbal_absolute_angle_limit()` 实现。该函数先计算目标角与当前绝对角的误差 `bias_angle`，再用 `relative_angle + bias_angle + add` 判断机械角度边界，最后写回 `absolute_angle_set`。

限位机制：

```text
当前相对角 + 绝对角误差 + 本周期增量
-> 得到下一步相对角预测
-> 超出 min/max_relative_angle 时裁剪增量
-> 更新 absolute_angle_set
```

## 6. 控制器结构

云台使用串级 PID 控制：

```text
目标绝对角 - 当前绝对角
-> 角度环 gimbal_PID_calc()
-> 目标绝对角速度 absolute_speed_set
-> 速度环 PID_calc()
-> 扭矩指令 set_Tp
-> CAN 下发 DM4310 扭矩
```

角度环使用 `gimbal_PID_t`：

- `Kp/Ki/Kd/Kf`
- 积分分离阈值 `I_Band`
- 周期 `dt`
- 输出限幅 `max_out`
- 积分限幅 `max_iout`
- 角度误差使用 `rad_format(set - get)` 限制到 `[-π, π]`
- 前馈项 `Fout = Kf * (set - last_set)`

速度环使用 `pid_type_def`：

- `Kp/Ki/Kd`
- 积分分离阈值 `I_Band`
- 周期 `dt`
- 输出限幅 `max_out`
- 积分限幅 `max_iout`
- 误差 `error = set - ref`

PID 计算细节位于：

- `PID_init()`：`components/controller/pid.c:27`。
- `PID_calc()`：`components/controller/pid.c:50`。
- `gimbal_PID_init()`：`components/controller/pid.c:111`。
- `gimbal_PID_calc()`：`components/controller/pid.c:136`。

当前初始化参数：

```text
yaw 绝对角度环：Kp=20, Ki=0, Kd=0, Kf=0, I_Band=0, dt=0.001, max_out=10, max_iout=0
yaw 绝对速度环：Kp=1,  Ki=0, Kd=0,       I_Band=0, dt=0.001, max_out=5,  max_iout=0

pitch 绝对角度环：Kp=10, Ki=20, Kd=0.8, Kf=0, I_Band=0.1, dt=0.001, max_out=5, max_iout=2
pitch 绝对速度环：Kp=1,  Ki=0,  Kd=0,          I_Band=0,   dt=0.001, max_out=5, max_iout=0

yaw 相对角度环：Kp=10, Ki=0, Kd=0, Kf=0, I_Band=0, dt=0.001, max_out=4, max_iout=0
yaw 相对速度环：Kp=1,  Ki=5, Kd=0,       I_Band=3, dt=0.001, max_out=5, max_iout=5
```

`gimbal_motor_absolute_angle_control()` 和 `gimbal_motor_auto_control()` 当前控制公式一致，均使用绝对角度环和绝对速度环。`gimbal_motor_relative_angle_control()` 用于 yaw 相对角回正，目标相对角按当前 yaw 相对角选择 0 或 π。

## 7. 扭矩下发和保护

`gimbal_control_loop()` 得到 `set_Tp` 后，主循环每 2 ms 执行一次 CAN 发送分频：

```text
DBUS 或云台电机离线
-> 电机状态未使能时执行 gimbal_motor_wake()
-> 电机已使能时发送 0 扭矩

链路在线
-> 电机状态未使能时执行 gimbal_motor_wake()
-> 电机已使能时发送 -set_Tp
```

`gimbal_motor_wake()` 根据 DM4310 状态发送：

- `Status == 0x0D`：清错命令。
- `Status == 0x00`：使能命令。
- 其他状态：发送 0 扭矩。

扭矩打包：

```c
Tor = Data_Clipping(Tor, -10.0f, 10.0f);
Torque = DM_float_to_uint(Tor, -10.0f, 10.0f, 12);
```

发送标识符：

- yaw：CAN1，StdId `0x05`。
- pitch：CAN2，StdId `0x06`。

代码中下发 `-set_Tp`，移植时需要保持电机安装方向、反馈符号和输出符号三者一致。

## 8. 云台到底盘通信

`gimbal_to_chassis()` 每个云台周期计算底盘控制量，并在 DBUS 在线时交替发送 `CAN_cmd_chassis()` 和 `CAN_cmd_chassis2()`。

发送内容：

- `vx_set`、`vy_set`：底盘速度。
- `roll_set`：腿轮 roll 补偿。
- `legL_mode`：腿长模式。
- `mode`：底盘模式。
- `relative_angle_yaw`：云台 yaw 相对角。
- `chassis_power_limit`、`buffer_energy_chassis`：裁判系统功率和缓冲能量。
- `LeftPhi0Set/LeftLengthSet/RightPhi0Set/RightLengthSet`：腿部调试设定。

键鼠操作模式中：

- W/S 控制前后速度。
- Shift 进入小陀螺模式。
- A/D 在小陀螺模式中控制横移。
- Q/E 控制 roll。
- Ctrl/F 控制腿长模式。
- R 触发跳跃模式。
- C 触发趴下。
- V 切换 180 度头尾方向映射。
- Z 触发自救相关模式，并清零云台扭矩。

## 9. 调试和监测链路

离线检测通过 `detect_task` 完成。接收到遥控、云台电机、底盘、超级电容等数据时调用 `detect_hook()` 更新时间戳，`toe_is_error()` 给主控逻辑查询。

关键离线阈值：

- DBUS：100 个检测周期。
- yaw 电机：20 个检测周期。
- pitch 电机：20 个检测周期。
- 底盘到云台：10 个检测周期。
- 超级电容：10 个检测周期。

检测任务周期 `DETECT_CONTROL_TIME` 为 10 ms，因此 yaw/pitch 离线阈值约 200 ms。

## 10. 移植边界

移植云台控制需要保留的文件组：

```text
Task/gimbal_task.c
Task/gimbal_task.h
application/gimbal_behaviour.c
application/gimbal_behaviour.h
components/controller/pid.c
components/controller/pid.h
components/algorithm/user_lib.c
components/algorithm/user_lib.h
application/CAN_receive.c
application/CAN_receive.h
application/remote_control.c
application/remote_control.h
application/USART_receive.c
application/USART_receive.h
Task/INS_task.c
Task/INS_task.h
Task/detect_task.c
Task/detect_task.h
boards/bsp_can.c
```

移植时需要重建的外部接口：

- `yaw_motor_measure_point()`、`pitch_motor_measure_point()`：提供 yaw/pitch 电机反馈。
- `CAN_cmd_gimbal_yaw()`、`CAN_cmd_gimbal_pitch()`：输出 DM4310 扭矩。
- `get_INS_angle_point()`、`get_gyro_data_point()`、`get_accel_data_point()`：提供姿态数据。
- `remote_data` 或 `rc_ctrl`：提供遥控/键鼠输入。
- `auto_shoot`：提供视觉目标 yaw/pitch。
- `toe_is_error()`、`detect_hook()`：提供离线检测。
- `robot_state`、`power_heat_data_t`：提供裁判系统功率、热量和底盘功率限制。

移植参数需要按新车重新标定：

- yaw/pitch `offset_angle`。
- pitch `min_relative_angle/max_relative_angle`。
- yaw/pitch 符号映射。
- yaw 角速度投影公式中的 pitch 符号。
- PID 参数和输出限幅。
- DM4310 CAN 标识符、总线号、扭矩范围。
- 图传链路帧格式和按键定义。
- 云台到底盘 CAN 协议。

## 11. 移植顺序

1. 先移植传感器和电机反馈接口，让 `gimbal_feedback_update()` 能得到正确的绝对角、相对角、绝对角速度、相对速度。
2. 再移植模式机和目标生成，让 `absolute_angle_set` 能被遥控、鼠标和视觉链路正确写入。
3. 最后移植控制器和 CAN 输出，用小输出限幅验证方向，再逐步恢复原 PID 参数。

## 12. 学习抓手

阅读这套代码时可以按以下变量流追踪：

```text
remote_data / auto_shoot / INS_angle / motor_gimbal
-> gimbal_move.gimbal_yaw_motor.absolute_angle_set
-> gimbal_move.gimbal_yaw_motor.absolute_speed_set
-> gimbal_move.gimbal_yaw_motor.gimbal_motor.set_Tp
-> CAN_cmd_gimbal_yaw()
```

pitch 链路同理：

```text
remote_data / auto_shoot / INS_angle / motor_gimbal
-> gimbal_move.gimbal_pitch_motor.absolute_angle_set
-> gimbal_move.gimbal_pitch_motor.absolute_speed_set
-> gimbal_move.gimbal_pitch_motor.gimbal_motor.set_Tp
-> CAN_cmd_gimbal_pitch()
```

核心控制方法可以概括为：IMU 绝对姿态闭角度环，陀螺仪角速度闭速度环，DM4310 工作在扭矩控制输入模式，机械限位通过目标角预测在外层提前裁剪。
