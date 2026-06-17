# Infantry

`Infantry` 是一套面向 RoboMaster 步兵机器人的 STM32H723VGTx 控制板固件工程。主控基于 STM32 HAL、FreeRTOS CMSIS-RTOS v1、FDCAN、UART DMA、USB CDC 和 OCTOSPI2，实现云台、底盘、发射、自瞄通信、裁判数据解析、在线检测、功率控制、W25Q64 外部 Flash、VOFA 调试和外置灯板状态帧输出。

当前工程已经将机器人级参数文件整理为 `User/APP_Support/common/robot_param.h`，业务层代码规范见 `代码规范.md`。CubeMX 生成代码仍以 `CtrlBoard-H7_WS1812.ioc` 为入口，手写业务代码集中放在 `User` 目录。

## 功能特性

- 云台控制：yaw/pitch 双轴控制，支持陀螺仪绝对角、编码器相对角、RAW 输出、初始化、校准、静摩擦补偿、惯量前馈、目标曲线和 pitch 重力补偿。
- 发射控制：三路 3508 摩擦轮 ADRC 闭环、掉速前馈补偿、拨弹 DJI 电机单发/长按控制、反馈超时保护、电机温度保护、开火检测、弹速估计和热量模型禁发。
- 发射热量模型：初始热量为 0，开火检测确认一发弹丸后热量 `+100`，热量上限为 `200`，预测下一发会使热量 `>=200` 时禁止拨弹，热量每秒自然下降 `20`。
- 底盘控制：全向底盘速度规划、运动学正解/逆解、轮速 PID、整车动力学前馈、单轮制动补偿、实际车速反馈、动态电流限制和底盘功率控制。
- 自瞄通信：USB CDC + uproto + channel 框架，包含云台状态发布、自瞄增量注入、主机底盘/射击命令注入、相机触发和时间同步通道。
- 裁判与功控：裁判系统结构解析入口、枪口热量读取接口、底盘功率和缓冲能量读取接口、PM01 超级电容对象字典访问、底盘功率限制接口。
- 在线检测：DBUS、底盘电机、云台电机、摩擦轮和拨弹电机在线状态检测，供任务保护、零输出保护和灯板显示使用。
- 灯板显示：主控 UART8 输出 10 路 RGB 状态帧，外置 CH32V003F4P 灯板固件驱动 WS2812 灯珠；8 号提示灯显示发射热量，颜色从绿到黄再到红。
- 外部 Flash：OCTOSPI2 驱动 W25Q64，支持 JEDEC ID 校验、4 KB 扇区擦除、32 KB/64 KB 块擦除、整片擦除、页写入、连续读写和内存映射。
- Flash 错误日志：`flash_log` 把短文本错误事件缓存到 RAM 队列，再由 `service_task` 周期写入 W25Q64 末尾 64 KB 日志区。
- 服务任务：`service_task` 统一调度 HWT IMU 初始化、Flash 日志初始化、WS2812、蜂鸣器、VOFA 6 通道发送和日志落盘服务。
- 调试支持：VOFA 固定 6 通道数据发送、Keil 构建日志、弹道/惯量测试数据、MATLAB 拟合脚本和控制链路图。

## 技术栈

- MCU：STM32H723VGTx，Cortex-M7，LQFP100。
- 主控框架：STM32CubeMX 生成工程 + STM32H7 HAL Driver。
- RTOS：FreeRTOS，CMSIS-RTOS v1。
- 构建工具：Keil MDK-ARM 工程文件，当前本机使用 μVision V5.40 / ARMCC V5.06。
- 通信接口：FDCAN1、FDCAN2、FDCAN3、UART5、UART7、UART8、USART1、USART10、USB CDC HS、OCTOSPI2、SPI2、SPI6、TIM24。
- 外部存储：W25Q64，8 MB，JEDEC ID 为 `0xEF4017`，OSPI 内存映射起始地址为 `0x90000000`。
- 控制算法：PID、ADRC、Kalman Filter、Quaternion EKF、目标曲线、重力补偿、惯量前馈、摩擦补偿、低通滤波和数学工具函数。
- 上位机协议：uproto、channel manager、gimbal/camera/time_sync 通道。
- 外置灯板：CH32V003F4P + WS2812。
- 调试工具：Git、VOFA+、Keil build log、MATLAB、Flash 日志串口导出。

## 目录结构

```text
Infantry/
├── Core/                                   # STM32CubeMX 生成的主控基础工程
│   ├── Inc/                                # 外设句柄、HAL 配置、FreeRTOSConfig 和中断声明
│   │   ├── main.h                          # 全局 HAL 入口和 GPIO 宏
│   │   ├── FreeRTOSConfig.h                # FreeRTOS 裁剪配置
│   │   ├── fdcan.h / usart.h / tim.h       # FDCAN、UART、TIM 外设接口
│   │   ├── octospi.h / spi.h / dma.h       # OCTOSPI2、SPI、DMA 外设接口
│   │   └── stm32h7xx_it.h                  # 中断服务声明
│   └── Src/                                # 启动流程、外设初始化、任务创建和中断实现
│       ├── main.c                          # HAL、时钟、CAN、UART DMA、OCTOSPI2、TIM24、uproto 启动入口
│       ├── freertos.c                      # FreeRTOS 任务创建入口
│       ├── fdcan.c / usart.c / tim.c       # CubeMX 外设初始化实现
│       ├── octospi.c / spi.c / dma.c       # CubeMX 外设初始化实现
│       ├── stm32h7xx_it.c                  # 中断分发入口
│       └── system_stm32h7xx.c              # 系统时钟底层支持
├── Drivers/                                # ST 官方驱动和 CMSIS 支持包
│   ├── CMSIS/                              # Cortex-M、STM32H723 头文件和系统定义
│   └── STM32H7xx_HAL_Driver/               # HAL/LL 驱动源码和头文件
├── Middlewares/                            # 第三方和 ST 中间件
│   ├── Third_Party/FreeRTOS/Source/        # FreeRTOS 内核、CMSIS_RTOS 封装、heap_4
│   └── ST/STM32_USB_Device_Library/        # USB Device Core 和 CDC Class
├── USB_DEVICE/                             # USB CDC 设备层
│   ├── App/
│   │   ├── usb_device.c                    # USB 设备初始化入口
│   │   ├── usbd_cdc_if.c                   # CDC 收发回调和接口适配
│   │   └── usbd_desc.c                     # USB 描述符
│   └── Target/
│       └── usbd_conf.c                     # USB PCD 底层配置
├── User/
│   ├── Algorithm/                          # 控制算法和通用数学工具
│   │   ├── pid.c / pid.h                   # PID 控制器
│   │   ├── adrc.c / adrc.h                 # 摩擦轮 ADRC 控制器
│   │   ├── gravity_comp.c / gravity_comp.h # pitch 重力补偿
│   │   ├── target_curve.c / target_curve.h # 目标曲线工具
│   │   ├── kalman_filter.c / *.h           # Kalman Filter
│   │   ├── QuaternionEKF.c / *.h           # 四元数 EKF 姿态解算
│   │   ├── controller.c / controller.h     # 控制器辅助接口
│   │   └── user_lib.c / user_lib.h         # 角度归一化、限幅、滤波和数学工具
│   ├── APP/                                # FreeRTOS 应用任务层
│   │   ├── gimbal_task.c / gimbal_task.h   # 云台闭环主任务，调度发射控制
│   │   ├── chassis_task.c / chassis_task.h # 底盘主任务，调度底盘闭环和 CAN 下发
│   │   ├── auto_aim.c / auto_aim.h         # 自瞄误差缓存、在线状态和软开关
│   │   ├── detect_task.c / detect_task.h   # DBUS、电机和外设在线检测
│   │   ├── light_task.c / light_task.h     # 外置灯板状态渲染和 UART8 帧发送
│   │   ├── referee_usart_task.c / *.h      # 裁判串口任务入口
│   │   ├── service_task.c / *.h            # 蜂鸣器、IMU、VOFA、Flash 日志和板载灯服务入口
│   │   ├── Safewarning.c / Safewarning.h   # 安全提示/蜂鸣器相关逻辑
│   │   └── usb_task.c / usb_task.h         # USB 任务保留入口
│   ├── APP_Support/                        # 应用支撑层和参数层
│   │   ├── common/
│   │   │   ├── robot_param.h               # 全局模式、CAN ID、云台/底盘/发射/功率公共参数
│   │   │   ├── referee.c / referee.h       # 裁判系统数据结构和解析接口
│   │   │   ├── flash_log.c / flash_log.h   # W25Q64 错误日志队列、落盘、导出和清除
│   │   │   ├── protocol.h                  # 裁判协议结构体
│   │   │   └── struct_typedef.h            # 基础类型定义
│   │   ├── gimbal/
│   │   │   ├── gimbal_behaviour.c / *.h    # 云台行为状态机
│   │   │   ├── yaw_pitch_direct.c / *.h    # yaw/pitch 反馈、目标和 MIT 下发链路
│   │   │   └── target_curve.c / *.h        # 云台目标曲线工具
│   │   ├── chassis/
│   │   │   ├── Omni_chassis.c / *.h        # 底盘执行层、速度规划、轮速控制、制动补偿
│   │   │   ├── chassis_behaviour.c / *.h   # 底盘行为状态机
│   │   │   └── chassis_calculate.c / *.h   # 底盘运动学正解/逆解和坐标转换
│   │   ├── shoot/
│   │   │   ├── shoot_task.c / *.h          # 发射弱接口和发射状态结构
│   │   │   └── shoot_3508.c / *.h          # 摩擦轮、拨弹、开火检测、热量模型和保护逻辑
│   │   └── power_control/
│   │       ├── chassis_power_control.c / *.h # 底盘功率预测、限幅和电流缩放
│   │       ├── pm01_api.c / *.h              # PM01 超级电容 CAN 对象字典访问
│   │       └── wattmeter_api.c / *.h         # 功率计接口
│   ├── BSP/                                # 板级驱动和外设分发层
│   │   ├── bsp_fdcan.c / bsp_fdcan.h       # FDCAN 收发、MIT/DJI/PM01 反馈分发、CAN 命令封装
│   │   ├── bsp_usart.c / bsp_usart.h       # UART DMA ReceiveToIdle 回调、SBUS/IMU 分发、串口发送
│   │   ├── remote_control.c / *.h          # SBUS 到 RC_ctrl_t 解析
│   │   ├── bsp_tim24.c / *.h               # TIM24 微秒时间基
│   │   └── bsp_dwt.c / *.h                 # DWT 高精度计时
│   ├── Communication/                      # USB CDC 上位机通信协议栈
│   │   ├── core/                           # uproto、通道管理、CRC 和平台抽象
│   │   ├── channel/
│   │   │   ├── gimbal/                     # 云台状态发布、自瞄增量、射击/底盘命令通道
│   │   │   ├── camera/                     # 相机触发事件通道
│   │   │   └── time_sync/                  # 主机与设备时间同步通道
│   │   └── example/
│   │       ├── device/                     # STM32 设备端通信任务和 USB CDC 适配
│   │       ├── host/                       # C/C++ 主机端示例和 CMake 工程
│   │       └── shared/                     # 主机与设备共享协议 ID
│   └── Devices/                            # 具体设备驱动和调试输出
│       ├── hwt_imu.c / hwt_imu.h           # HWT101/HWT906 IMU 数据解析
│       ├── vofa.c / vofa.h                 # VOFA 调试数据发送
│       ├── w25q64.c / w25q64.h             # W25Q64 初始化、擦除、读写和内存映射
│       └── ws2812.c / ws2812.h             # 板载 WS2812 驱动
├── MDK-ARM/                                # Keil MDK-ARM 主控工程
│   ├── CtrlBoard-H7_WS1812.uvprojx         # Keil 工程文件
│   ├── CtrlBoard-H7_WS1812.uvoptx          # Keil 工程选项
│   ├── startup_stm32h723xx.s               # STM32H723 启动汇编
│   ├── CtrlBoard-H7_WS1812/                # axf、hex、map、build_log 和中间文件输出
│   ├── DebugConfig/                        # 调试器配置
│   └── RTE/                                # Keil RTE 组件配置
├── CtrlBoard-H7_WS1812.ioc                 # STM32CubeMX 工程配置
├── 代码规范.md                             # 当前工程代码规范
├── BUG_FIX_RECORD.md                       # 修复记录
├── W25Q64_移植说明.md                      # W25Q64 移植记录
├── 裁判链路迁移缺口与修改说明.md           # 裁判链路迁移记录
├── 麦克纳姆底盘闭环与急停减速链路.md       # 底盘闭环与急停减速链路记录
├── ST_Edge_AI_功率控制预测模型部署指南.md  # ST Edge AI 功率预测模型部署说明
├── NanoEdge_AI_Studio_底盘功率控制教程.md  # NanoEdge AI Studio 功率控制教程
├── 工程框架迁移提示词.md                   # 工程框架迁移说明
├── fit_yaw_inertia_from_vofa.m             # yaw 惯量拟合脚本
├── vofa+.csv                               # VOFA 采样数据
├── power_profile_filtered.csv              # 功率控制采样数据
├── shot.png                                # 发射/弹道相关图片
├── 开源报告.pdf                            # 开源报告文档
├── 调试日志.md                             # 调试记录
└── 弹道测试数据/                           # 弹道测试数据
```

## 环境要求

- Windows 开发环境。
- Keil MDK-ARM V5.27 或兼容版本；当前本机验证环境为 μVision V5.40。
- ARMCC V5.06 或 Keil 工程兼容工具链。
- STM32CubeMX，用于修改 `.ioc` 中的外设、RTOS、时钟树和中断配置。
- STM32H7xx Device Family Pack。
- 调试/下载器：J-Link 或 ST-Link，具体型号按现场硬件配置。
- 主控目标芯片：STM32H723VGTx。
- 外部存储硬件：W25Q64，连接 OCTOSPI2。
- 外置灯板硬件：CH32V003F4P + WS2812，主控通过 UART8 输出 RGB 状态帧。
- 可选工具：Git、MATLAB、VOFA+、CMake/C++ 编译器。

## 安装步骤

1. 获取代码：

```powershell
git clone <repo-url>
cd Infantry
```

2. 打开主控工程：

```text
MDK-ARM/CtrlBoard-H7_WS1812.uvprojx
```

3. 在 Keil 中安装 STM32H723VGTx 对应 Device Pack，并确认工程宏包含：

```text
USE_HAL_DRIVER,STM32H723xx,USE_PWR_LDO_SUPPLY
```

4. 使用 Keil 编译工程，输出文件位于：

```text
MDK-ARM/CtrlBoard-H7_WS1812/CtrlBoard-H7_WS1812.axf
MDK-ARM/CtrlBoard-H7_WS1812/CtrlBoard-H7_WS1812.hex
```

5. 连接调试器并下载到 STM32H723VGTx 控制板。

## 运行方式

主控上电后执行 `Core/Src/main.c`，启动链路为：

```text
HAL_Init()
-> SystemClock_Config()
-> MX_GPIO_Init() / MX_DMA_Init() / 外设初始化
-> bsp_can_init()
-> UART DMA ReceiveToIdle 启动
-> MX_OCTOSPI2_Init()
-> OSPI_W25Qxx_Init()
-> tim24_timebase_init()
-> proto_init_from_main()
-> MX_FREERTOS_Init()
-> osKernelStart()
```

FreeRTOS 创建的主要任务：

| 任务 | 入口 | 优先级 | 栈 | 周期/延时 | 职责 |
|---|---|---:|---:|---|---|
| defaultTask | `StartDefaultTask` | Normal | 128 | 1 ms | USB_DEVICE 初始化和保留循环 |
| auto_aim | `auto_aim_task` | Realtime | 256 | 1 ms | 自瞄在线状态、软开关、弹道下坠补偿和误差缓存 |
| COMM_APP | `comm_app_task` | Realtime | 640 | 1 ms | USB CDC、uproto、通道调度、主机命令注入 |
| gimbalTask | `gimbal_task` | High | 1024 | `GIMBAL_CONTROL_TIME` | 云台闭环、重力补偿、发射调度、VOFA 输出 |
| serviceTask | `ServiceTask_Init` 创建 | Low | 256 | `SERVICE_CONTROL_TIME` | HWT IMU、Flash 日志、WS2812、蜂鸣器和 VOFA 服务 |
| lightTask | `light_task` | Low | 256 | `LIGHT_TASK_PERIOD_MS` | 外置灯板状态帧生成和 UART8 发送 |
| detect | `detect_task` | Low | 128 | `DETECT_CONTROL_TIME` | DBUS、电机和外设在线检测 |
| chassis | `chassis_task` | High | 768 | `CHASSIS_CONTROL_TIME_MS` | 底盘控制、功控、CAN 输出入口 |

## 控制链路

云台任务链路固定为：

```text
gimbal_init
-> shoot_init
-> gimbal_set_mode
-> gimbal_mode_change_control_transit
-> gimbal_feedback_update
-> gimbal_set_control
-> gimbal_control_loop
-> gravity_comp_execute
-> shoot_control_loop
-> gimbal_send_cmd
-> vTaskDelayUntil(GIMBAL_CONTROL_TIME)
```

底盘任务链路固定为：

```text
chassis_init
-> chassis_set_mode
-> chassis_mode_change_control_transit
-> chassis_feedback_update
-> chassis_set_contorl
-> chassis_control_loop
-> chassis_send_cmd
-> osDelay(CHASSIS_CONTROL_TIME_MS)
```

底盘控制循环固定为：

```text
速度规划
-> chassis_body_feedforward_update
-> chassis_body_velocity_brake_update
-> chas_inv_cal
-> PID_Calc_Jump
-> chassis_dynamic_current_limit_update
-> chassis_power_control
-> CAN 电流发送
```

发射控制循环固定为：

```text
shoot_task_set_mode
-> shoot_task_update_feedback
-> shoot_task_update_heat_model
-> shoot_task_control_friction / shoot_task_stop_friction
-> shoot_task_control_strum
-> shoot_task_send_motor_current
```

发射热量链路：

```text
摩擦轮掉速和电流开火检测
-> shoot_task_set_fire_detected
-> fired_bullet_count++
-> heat += SHOOT_HEAT_PER_BULLET
-> shoot_task_fire_heat_would_over_limit
-> 拨弹入口禁止继续打弹
-> light_render_shoot_heat_status 显示热量色域
```

service 任务启动链路：

```text
ServiceTask_Init
-> osThreadNew(service_task)
-> vTaskDelay(SERVICE_TASK_INIT_TIME)
-> hwt_imu_init
-> flash_log_init
-> 周期服务循环
```

service 任务周期链路：

```text
service_time += SERVICE_CONTROL_TIME
-> ws2812_task
-> Beep_Task
-> VOFA_ServiceSend
-> flash_log_service
-> vTaskDelay(SERVICE_CONTROL_TIME)
```

通信接收链路：

```text
USB OUT ISR
-> CDC_Receive_HS
-> uproto_on_rx_bytes
-> uproto_process_rx_buffer
-> handler(UPROTO_MSG_MUX)
-> on_mux_rx
-> chmgr_dispatch_rx
-> channel.hooks.on_rx
```

通信发送链路：

```text
on_tick / on_rx
-> ch_uproto_queue_notify
-> ch_uproto_arbiter_tick
-> mux_encode
-> uproto_send_notify
-> USB CDC 发送
```

## 发射热量模型

热量模型位于 `User/APP_Support/shoot/shoot_3508.c` 和 `User/APP_Support/shoot/shoot_task.h`。

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `SHOOT_HEAT_PER_BULLET` | `100U` | 开火检测确认一发弹丸后的热量增量 |
| `SHOOT_HEAT_LIMIT` | `200U` | 热量上限，预测下一发达到或超过该值时禁止拨弹 |
| `SHOOT_HEAT_COOL_PER_SECOND` | `20U` | 每秒自然冷却热量 |
| `SHOOT_HEAT_DECAY_INTERVAL_MS` | `50U` | 每 50 ms 热量下降 1 |

控制状态字段：

| 字段 | 含义 |
|---|---|
| `fired_bullet_count` | 开火检测确认的打弹数量 |
| `heat` | 当前软件热量模型值 |
| `heat_cool_ticks` | 热量冷却计时 |
| `heat_limit_active` | 下一发会达到或超过热量上限时置位 |

禁发条件为：

```c
heat + SHOOT_HEAT_PER_BULLET >= SHOOT_HEAT_LIMIT
```

在当前参数下，第一发后热量变为 100；第二发会使热量达到 200，因此在热量冷却到 99 之前禁止继续拨弹。

## 灯板显示

主控通过 UART8 发送 10 路 RGB 状态帧，帧格式为：

```text
0xAA 0x55 + 10 * RGB + 0x55 0xAA
```

当前灯位分配：

| 灯位 | 含义 |
|---:|---|
| 0 | DBUS 在线状态 |
| 1 | 底盘电机在线状态 |
| 2 | 摩擦轮在线/工作状态 |
| 3 | 自瞄在线/激活状态 |
| 4~6 | 底盘行为状态 |
| 7 | 云台行为状态 |
| 8 | 发射热量状态，绿 -> 黄 -> 红 |
| 9 | light 任务心跳 |

热量灯显示规则：

- `heat = 0`：绿色。
- `heat = SHOOT_HEAT_LIMIT / 2`：黄色。
- `heat_limit_active = true` 或 `heat >= SHOOT_HEAT_LIMIT`：红色。

## 配置说明

### 工程与外设

- `CtrlBoard-H7_WS1812.ioc`：STM32CubeMX 配置入口，包含 STM32H723VGTx、FreeRTOS、USB CDC HS、FDCAN、UART DMA、SPI 和 TIM24。
- `MDK-ARM/CtrlBoard-H7_WS1812.uvprojx`：Keil 工程入口，目标名为 `CtrlBoard-H7_WS1812`，生成 `axf` 和 `hex` 文件。
- `Core/Src/main.c`：外设初始化顺序、CAN 启动、UART DMA 启动、TIM24 时间基和 uproto 初始化。
- `Core/Src/freertos.c`：任务创建入口。

### 关键外设参数

| 外设 | 配置 | 用途 |
|---|---|---|
| FDCAN1 | Classic CAN，1 Mbps | yaw MIT、拨弹 DJI、底盘电机、PM01 |
| FDCAN2 | Classic CAN，1 Mbps | pitch MIT、三路摩擦轮 |
| FDCAN3 | Classic CAN，0.25 Mbps | 预留接收入口 |
| UART5 | 100000 baud，8E1，DMA RX | DBUS/SBUS 遥控器 |
| UART7 | 115200 baud，DMA RX/TX | HWT101 接收 |
| UART8 | 115200 baud，DMA TX | 外置灯板发送 |
| USART1 | 921600 baud，DMA RX/TX | 裁判系统/串口通信入口 |
| USART10 | 921600 baud，DMA RX/TX | HWT906 或扩展通信 |
| USB_DEVICE | CDC HS | 上位机通信 |
| OCTOSPI2 | Quad SPI，8 MB 地址空间 | W25Q64 外部 Flash |
| TIM24 | 内部时钟，Prescaler 239 | 微秒时间基 |

### 外部 Flash 参数

OCTOSPI2 由 CubeMX 生成，业务驱动位于 `User/Devices/w25q64.c` 和 `User/Devices/w25q64.h`。

| 参数 | 当前值 | 作用 |
|---|---:|---|
| 实例 | `OCTOSPI2` | 外部 W25Q64 通信接口 |
| 句柄 | `hospi2` | HAL OSPI 操作对象 |
| FifoThreshold | `8` | OSPI FIFO 阈值 |
| DualQuad | `HAL_OSPI_DUALQUAD_DISABLE` | 单片 Quad Flash 模式 |
| MemoryType | `HAL_OSPI_MEMTYPE_MICRON` | HAL 存储器类型配置 |
| DeviceSize | `23` | 8 MB 地址空间 |
| ClockMode | `HAL_OSPI_CLOCK_MODE_3` | OSPI 时钟模式 |
| ClockPrescaler | `3` | OSPI 分频参数 |
| SampleShifting | `HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE` | 半周期采样移位 |

OCTOSPI2 引脚映射：

| 引脚 | 功能 |
|---|---|
| PA1 | `OCTOSPIM_P1_IO3` |
| PA3 | `OCTOSPIM_P1_IO2` |
| PB0 | `OCTOSPIM_P1_IO1` |
| PB2 | `OCTOSPIM_P1_CLK` |
| PE11 | `OCTOSPIM_P1_NCS` |
| PD11 | `OCTOSPIM_P1_IO0` |

W25Q64 容量参数：

| 宏 | 当前值 | 含义 |
|---|---:|---|
| `W25Qxx_PageSize` | `256` | 页大小，单位 byte |
| `W25Qxx_FlashSize` | `0x800000` | 总容量 8 MB |
| `W25Qxx_FLASH_ID` | `0xEF4017` | W25Q64 JEDEC ID |
| `W25Qxx_Mem_Addr` | `0x90000000` | 内存映射起始地址 |

W25Q64 驱动接口：

| 接口 | 功能 |
|---|---|
| `OSPI_W25Qxx_Init()` | 初始化并校验 JEDEC ID |
| `OSPI_W25Qxx_ReadID()` | 读取 W25Q64 ID，目标值为 `0xEF4017` |
| `OSPI_W25Qxx_MemoryMappedMode()` | 进入 OSPI 内存映射模式 |
| `OSPI_W25Qxx_SectorErase()` | 擦除 4 KB 扇区 |
| `OSPI_W25Qxx_BlockErase_32K()` | 擦除 32 KB 块 |
| `OSPI_W25Qxx_BlockErase_64K()` | 擦除 64 KB 块 |
| `OSPI_W25Qxx_ChipErase()` | 整片擦除 |
| `OSPI_W25Qxx_WritePage()` | 页写入，单页 256 byte |
| `OSPI_W25Qxx_WriteBuffer()` | 连续写入缓冲区 |
| `OSPI_W25Qxx_ReadBuffer()` | 连续读取缓冲区 |

### 主要参数文件

- `User/APP_Support/common/robot_param.h`：机器人模式、电容开关、CAN ID、通道映射、云台 PID、底盘几何、底盘速度规划、底盘制动、电机 ID、发射公共参数。
- `User/APP/chassis_task.h`：底盘控制结构体、底盘模式枚举、任务接口和弱接口声明。
- `User/APP/gimbal_task.h`：云台控制结构体、云台电机状态、PID 接口和任务接口声明。
- `User/APP/service_task.h`：服务任务周期、启动延时和 `service_control_t`。
- `User/APP_Support/shoot/shoot_task.h`：摩擦轮目标转速、电流限制、ADRC 参数、拨弹 PID、开火检测字段、热量模型参数。
- `User/APP_Support/common/flash_log.h`：Flash 日志区大小、记录结构、来源枚举、等级枚举和维护接口。
- `User/Devices/w25q64.h`：W25Q64 命令、容量、JEDEC ID、错误码和 OSPI 读写接口。
- `User/APP/light_task.h`：灯珠数量、帧长度、灯板任务周期、灯位映射和灯效状态结构。
- `User/Communication/example/device/comm_app_config.h`：通信任务栈、优先级、通道 ID、USB 枚举超时和主机命令注入通道映射。

### Flash 错误日志

Flash 日志位于 `User/APP_Support/common/flash_log.c` 和 `User/APP_Support/common/flash_log.h`，事件先进入 RAM 队列，再由 `service_task` 周期写入 W25Q64 末尾日志区。

控制开关位于 `User/APP_Support/common/robot_param.h`：

| 宏 | 当前值 | 含义 |
|---|---:|---|
| `FLASH_LOG_ENABLE` | `0U` | 外部 Flash 错误日志开关，`1U` 表示启用 |
| `FLASH_LOG_QUEUE_DEPTH` | `8U` | RAM 日志队列深度 |

日志区参数：

| 宏 | 当前值 | 含义 |
|---|---:|---|
| `FLASH_LOG_MAGIC` | `0x464C4F47UL` | 记录有效标识，ASCII 为 `FLOG` |
| `FLASH_LOG_RECORD_SIZE` | `64U` | 单条日志长度，单位 byte |
| `FLASH_LOG_TEXT_LEN` | `44U` | 单条日志文本长度，单位 byte |
| `FLASH_LOG_SECTOR_SIZE` | `4096U` | W25Q64 扇区大小，单位 byte |
| `FLASH_LOG_REGION_SIZE` | `64U * 1024U` | 日志区大小，单位 byte |
| `FLASH_LOG_REGION_BASE` | `W25Qxx_FlashSize - FLASH_LOG_REGION_SIZE` | 日志区起始地址 |
| `FLASH_LOG_REGION_END` | `W25Qxx_FlashSize` | 日志区结束地址 |
| `FLASH_LOG_RECORD_COUNT` | `FLASH_LOG_REGION_SIZE / FLASH_LOG_RECORD_SIZE` | 日志区记录容量 |

日志来源：

| 枚举 | 含义 |
|---|---|
| `FLASH_LOG_SOURCE_SYSTEM` | 系统初始化和全局状态 |
| `FLASH_LOG_SOURCE_DETECT` | 在线检测 TOE 状态 |
| `FLASH_LOG_SOURCE_OSPI` | 外部 Flash/OSPI 状态 |

日志接口：

| 接口 | 功能 |
|---|---|
| `flash_log_init()` | 扫描日志区并恢复下一写入地址和序号 |
| `flash_log_service()` | 从 RAM 队列取出一条记录并写入 Flash |
| `flash_log_enqueue_error()` | 入队错误事件 |
| `flash_log_dump_to_uart()` | 按序号输出日志到指定 UART |
| `flash_log_clear()` | 擦除日志区并重置 RAM 控制状态 |

日志写入链路：

```text
flash_log_enqueue_error()
-> RAM 队列保存 flash_log_record_t
-> service_task 周期调用 flash_log_service()
-> 扇区边界触发 OSPI_W25Qxx_SectorErase()
-> OSPI_W25Qxx_WriteBuffer() 写入 64 byte 记录
-> next_addr 到达 FLASH_LOG_REGION_END 后回到 FLASH_LOG_REGION_BASE
```

`FLASH_LOG_ENABLE` 为 `0U` 时，`flash_log_*` 接口保留空实现，调用链路保持稳定；启用日志时需要确认 W25Q64 初始化成功、OSPI2 引脚和时钟配置正确。

### 自瞄参数

自瞄参数位于 `User/APP/auto_aim.h`。

| 宏 | 当前值 | 含义 |
|---|---:|---|
| `AIM_INIT_TIME` | `500U` | 自瞄任务启动延时，单位 ms |
| `AUTO_AIM_TIMEOUT` | `2000U` | 自瞄反馈超时阈值，单位 ms |
| `AUTO_AIM_TIME` | `1U` | 自瞄任务周期，单位 ms |
| `AUTO_AIM_UDEG_TO_RAD` | `PI / 180000000.0f` | 微度到弧度换算系数 |
| `AUTO_AIM_BALLISTIC_DROP_K_MM_PER_M2` | `18.0f` | 弹道下坠补偿系数，单位 mm/m^2 |
| `AUTO_AIM_BALLISTIC_DISTANCE_M` | `3.9f` | 默认补偿距离，单位 m |
| `AUTO_AIM_MM_PER_M` | `1000.0f` | 米到毫米换算系数 |
| `AUTO_AIM_SOFT_ENABLE` | `0` | 自瞄软件开关默认值 |

自瞄接口：

| 接口 | 功能 |
|---|---|
| `auto_aim_task()` | 自瞄任务入口 |
| `auto_aim_apply_delta_udeg()` | 注入 yaw/pitch 微度增量、状态和时间戳 |
| `auto_aim_get_yaw_err_rad()` | 获取 yaw 弧度误差 |
| `auto_aim_get_pitch_err_rad()` | 获取 pitch 弧度误差 |
| `auto_aim_is_active()` | 读取自瞄在线并启用状态 |
| `auto_aim_reset_delta_accum()` | 清空增量累计值 |

### CAN ID 分配

| 设备 | 总线 | 命令 ID | 反馈 ID |
|---|---|---:|---|
| yaw MIT 电机 | FDCAN1 | `0x01` | `0x51` |
| pitch MIT 电机 | FDCAN2 | `0x02` | `0x52` |
| 拨弹 DJI 电机 | FDCAN1 | `0x200` | `0x204` |
| 摩擦轮 1/2/3 | FDCAN2 | `0x200` | `0x201` / `0x202` / `0x203` |
| 底盘 3508 电机组 | FDCAN1 | `0x1FF` | `0x205` ~ `0x208` |
| PM01 超级电容 | FDCAN1 | `0x600` ~ `0x603` | `0x600` ~ `0x603`、`0x610` ~ `0x613` |

## 代码规范

完整规范见 `代码规范.md`。README 保留当前工程必须遵守的核心规则。

### 模块归属

- `User/APP/*_task.c` 负责任务入口、全局控制对象、模块初始化调用、周期调度和弱接口声明。
- `User/APP/*_task.h` 负责任务数据结构、模块接口声明和跨模块状态字段。
- `User/APP_Support/common/robot_param.h` 负责机器人级宏、CAN ID、通道映射、底盘/云台/发射/功率等跨模块参数。
- `User/APP_Support/common/flash_log.*` 负责 W25Q64 日志区扫描、错误事件入队、周期落盘、串口导出和日志区清除。
- `User/APP_Support/chassis/chassis_behaviour.*` 负责遥控器、键鼠、掉线状态到底盘行为模式和底盘控制模式的映射。
- `User/APP_Support/chassis/chassis_calculate.*` 负责底盘运动学正解、逆解、坐标旋转和运动学限幅。
- `User/APP_Support/chassis/Omni_chassis.*` 负责底盘初始化、反馈更新、目标生成、速度规划、轮速控制、制动补偿和 CAN 发送。
- `User/APP_Support/gimbal/gimbal_behaviour.*` 负责云台行为模式映射。
- `User/APP_Support/gimbal/yaw_pitch_direct.*` 负责 yaw-pitch 云台反馈、目标生成、PID/前馈/摩擦补偿和 MIT/3508 输出。
- `User/APP_Support/shoot/shoot_3508.*` 负责 3508 摩擦轮和拨弹机构控制。
- `User/APP_Support/power_control/*` 负责底盘功率预测、功率限幅、电流缩放和功率模块通信。
- `User/BSP/*` 负责板级外设封装、CAN/UART/TIM/DWT 和遥控器底层数据。
- `User/Devices/w25q64.*` 负责 W25Q64 初始化、擦除、读写和内存映射。
- `User/Devices/*` 负责设备级封装和调试输出。

### 组织规则

- 新增功能先判断职责归属，沿已有模块文件、结构体、宏分组和调用链路接入。
- 结构体、枚举和宏定义统一放入对应模块头文件；`.c` 文件只保留静态变量、静态表、函数声明和函数实现。
- `.h` 文件顺序为头文件保护、依赖头文件、条件编译、宏定义、枚举、结构体、外部变量声明、函数声明。
- `.c` 文件顺序为模块头文件、依赖头文件、条件编译、局部静态表、全局状态、静态辅助函数、任务接口实现、控制辅助函数。
- 条件编译以 `ROBOT_TYPE`、`ROBOT_CHASSIS`、`ROBOT_GIMBAL`、`ROBOT_FRICTION` 为入口。
- 控制周期使用已有宏表达，例如 `CHASSIS_CONTROL_TIME_MS`、`GIMBAL_CONTROL_TIME`、`SHOOT_CONTROL_TIME`。

### 命名与注释

- 类型命名使用小写模块名加语义后缀：结构体用 `_t`，枚举用 `_e`。
- 控制函数使用模块前缀加动作：`chassis_*`、`gimbal_*`、`shoot_*`、`VOFA_*`。
- 全局控制对象使用模块名表达职责，例如 `chassis_move`、`gimbal_control`、`shoot_task_control`。
- 文件内私有函数使用 `static`，文件内私有表使用 `static const`。
- 宏使用全大写和下划线，数值宏名带单位语义。
- 文件编码统一使用 UTF-8，新增注释主要使用中文。
- 对外函数使用 Doxygen 风格块注释，说明功能、输入参数、返回值和单位。
- 结构体字段和宏定义使用行尾中文注释，说明物理含义、单位、取值含义或硬件映射。
- 控制参数宏注释写清单位、作用链路和调参直接效果。

### 数据与保护

- 外部传入指针在函数开头做空指针保护，保护后直接 `return` 或返回零值。
- 遥控器输入先经死区处理，再进入比例换算、滤波、限幅和控制量赋值。
- 角度进入控制前使用 `rad_format` 归一化。
- 输出速度、电流和功率进入执行前使用 `fp32_constrain`、`chassis_limit_abs` 或最大轮速比例缩放。
- 模式切换在 `*_mode_change_control_transit` 中处理，切换时保存目标角、相对角、规划速度、积分项或上一模式。
- 掉线和超时状态通过 `toe_is_error`、反馈时间戳、`online`、`error_code` 进入零输出、无力或保持逻辑。
- CAN 发送函数集中在 `*_send_cmd` 或 BSP CAN 封装中，控制函数只写目标值、中间控制量和最终电流命令。
- VOFA 输出固定 6 通道，统一经过 `VOFA_Send6`。

### CubeMX 修改约束

CubeMX 管理项包括 FreeRTOS 参数、任务/队列/信号量、外设初始化参数、GPIO、时钟树、中断优先级、DMA、NVIC、HAL tick、USB、FDCAN、UART、SPI、TIM。修改这些配置时先说明 CubeMX 路径、建议值和生成代码后的影响文件，再在 CubeMX 中修改并重新生成代码。

## 常用命令

查看仓库状态：

```powershell
git status --short
```

搜索源码：

```powershell
rg "GIMBAL_CONTROL_TIME"
rg "shoot_task_control" User
rg "CAN_cmd_MIT" User Core
rg "FLASH_LOG_ENABLE|flash_log_" User
rg "OSPI_W25Qxx|MX_OCTOSPI2" User Core
```

用 Keil 命令行构建主控工程，`UV4.exe` 路径按本机安装位置调整：

```powershell
& "D:\Keil5\UV4\UV4.exe" -j0 -b "MDK-ARM\CtrlBoard-H7_WS1812.uvprojx" -o "build_codex_current.log"
```

打开 STM32CubeMX 配置：

```powershell
start .\CtrlBoard-H7_WS1812.ioc
```

打开 Keil 工程：

```powershell
start .\MDK-ARM\CtrlBoard-H7_WS1812.uvprojx
```

主机端通信示例目录：

```powershell
cd .\User\Communication\example\host
.\build\uproto_host_cpp.exe --list
.\build\uproto_host_cpp.exe --quiet COM3
.\build\uproto_host_cpp.exe --sine yaw:1:10@50 COM3
```

## 开发说明

业务代码优先放在 `User` 目录；`Core`、`Drivers`、`Middlewares` 和 `USB_DEVICE` 中的 CubeMX 生成代码只在外设配置变更时同步调整。控制链路按“BSP/Devices 解析反馈 -> APP 任务读取输入 -> APP_Support 生成目标和控制量 -> Algorithm 计算 -> BSP 下发 CAN/UART/USB”的路径组织。

新增控制参数时优先放入对应模块头文件：全局、云台、底盘机械和跨模块公共参数放入 `robot_param.h`；发射参数放入 `shoot_task.h` 或 `shoot_3508.h`；通信参数放入 `comm_app_config.h`；Flash 日志参数放入 `flash_log.h` 或 `robot_param.h` 的日志开关区；W25Q64 命令和容量参数放入 `w25q64.h`；灯板参数放入 `light_task.h`。修改 `.ioc` 后需要用 CubeMX 重新生成代码，并检查 `USER CODE BEGIN/END` 区域内的手写逻辑是否保留。

当前底盘控制链路已经包含目标生成、速度规划、运动学计算、功控计算、电流变量写入和 CAN 下发。恢复实车输出或改动底盘控制时，需要沿 `chassis_set_mode -> chassis_feedback_update -> chassis_set_contorl -> chassis_control_loop -> chassis_send_cmd` 链路接入。

当前发射控制链路通过摩擦轮掉速和反馈电流判断开火，再由软件热量模型限制下一发拨弹。该模型不替代裁判系统热量数据；裁判热量接口仍在 `referee.c/referee.h` 中保留，可在后续需要时与软件模型融合。

当前外部 Flash 链路由 `Core/Src/main.c` 初始化 OCTOSPI2，再调用 `OSPI_W25Qxx_Init()` 校验 W25Q64 ID；Flash 日志由 `service_task` 初始化并周期服务，控制循环只调用 `flash_log_enqueue_error()` 入队事件。

OCTOSPI2、GPIO、时钟树、DMA、NVIC、USB、FDCAN、UART、SPI、TIM 和 FreeRTOS 属于 CubeMX 管理项。调整这些项时先给出 CubeMX 路径、建议值和生成代码后的影响文件，再通过 CubeMX 生成工程。

## 底盘动力学前馈与急停制动说明

底盘控制链路采用“速度规划 -> 整车动力学前馈 -> 单轮模型控制 -> 电流/功率限制”的结构。`vx_plan`、`vy_plan`、`wz_plan` 是底盘期望速度，整车前馈通过相邻控制周期的规划速度差分得到 `ax`、`ay`、`alpha`，再按整车质量和 yaw 转动惯量换算为车体所需的力和力矩，最后分解到 205、206、207、208 四个轮子的 `body_ff_current[]`。

四轮前馈分解使用几何公式和电机方向系数分开处理。几何公式只描述底盘坐标系下各轮对 `vx`、`vy`、`wz` 的贡献，实际电机默认正方向通过 `CHASSIS_WHEEL_205_DIRECTION`、`CHASSIS_WHEEL_206_DIRECTION`、`CHASSIS_WHEEL_207_DIRECTION`、`CHASSIS_WHEEL_208_DIRECTION` 统一变换。

松杆急停保留速度规划停车分支：当遥控输入导致 `cmd=0` 时，`chassis_s_curve_update()` 先用 `CHASSIS_STOP_DECEL` 生成与当前 `vx_plan` 或 `vy_plan` 方向相反的目标加速度，再用 `CHASSIS_STOP_JERK` 限制加速度变化速度，最后每个控制周期用 `plan = plan + accel * CHASSIS_CONTROL_TIME` 把规划速度拉向 0。

固定制动前馈在单轮模型控制中生效。轮速目标进入减速或接近 0，且实测轮速大于 `CHASSIS_BRAKE_ENTER_SPEED_EPS` 时，`stop_brake_active` 置位；此时普通摩擦前馈按 `CHASSIS_BRAKE_FRICTION_FF_SCALE` 缩放，固定制动前馈 `I_brake` 按实测轮速方向给反向电流。轮速低于 `CHASSIS_BRAKE_RELEASE_SPEED_EPS` 或重新给出速度目标时，制动状态释放。
