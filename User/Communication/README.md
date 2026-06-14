# Communication Module Guide (Device + Host)

本指南系统说明 Communication 模块的设计、数据流、回调触发点、常用配置与调优、以及故障排查。重点聚焦 Device 侧“收到消息后回调在哪里触发、处理链如何走完”，并给出 Host 使用姿势与对照。

---

## 快速上手

- Device（固件）
  - 确保 USB CDC 已启用（CubeMX 生成），FreeRTOS 存在。
  - 在应用中调用 `proto_init_from_main()`（见 `Core/Src/main.c`），并在 FreeRTOS 中创建通信任务：`comm_app_task`（见 `Core/Src/freertos.c`）。
  - 任务函数位于 `User/Communication/example/device/comm_app.c`，该任务会：等待 USB 枚举 → 绑定 uproto/MUX → 初始化各通道 → 周期 tick。
- Host（PC 工具）
  - 构建：`User/Communication/example/host/build`
    - MinGW 示例：`cmake -S .. -B . -G "MinGW Makefiles" && cmake --build . --config Release`
  - 运行（Windows）：
    - 仅监听：`uproto_host_cpp.exe COM3`
    - 安静模式：`uproto_host_cpp.exe --quiet COM3`
    - 发送云台正弦：`uproto_host_cpp.exe --sine yaw:1:10@50 COM3`（yaw 1Hz，±10°，50Hz 更新）

---

## 总体架构

- uproto：可靠消息与握手（`User/Communication/core/uproto.[ch]`）
- MUX：多通道复用（有 CRC16）（`User/Communication/core/comm.[ch]`）
- 通道管理：`channel` + `chmgr` + `ch_uproto_*`（注册、分发、仲裁、发送）
- 应用通道：gimbal（云台）、time_sync（时间同步）、camera（事件）

---

## Device 数据流（回调触发链）

下面按“字节流 → uproto → MUX → 通道回调 → 仲裁发送”的顺序，说明每一步谁来调用谁。

1) USB → uproto（底层收包入口）

- 位置：`USB_DEVICE/App/usbd_cdc_if.c: CDC_Receive_HS()`
- 核心：
  - `uproto_on_rx_bytes(&proto_ctx, Buf, *Len);`（把原始字节喂给协议栈）
  - 随后 `USBD_CDC_ReceivePacket(&hUsbDeviceHS);` 继续开启下次接收

2) uproto（帧解包与消息类型分发）

- 位置：`User/Communication/core/uproto.c`
- RX 处理链：`uproto_on_rx_bytes()` → `uproto_process_rx_buffer()` → 校验帧头/CRC → 依据 `type` 找到已注册的 handler
- 我们把 MUX 绑定为 `UPROTO_MSG_MUX`

3) MUX 解包 → chmgr 分发

- 位置：`User/Communication/core/comm.c`
- 在 `comm_app.c` 初始化时：
  - `ch_uproto_bind(&g_bind, &proto_ctx, UPROTO_MSG_MUX, &g_mgr);`
  - `ch_uproto_register_rx(&g_bind);`（内部注册 `UPROTO_MSG_MUX` 的 handler）
- handler：`on_mux_rx()` 会对 payload 做 `mux_decode()`，得到 `channel_id` 与内层 payload，然后调用：
  - `chmgr_dispatch_rx(mgr, channel_id, payload, len)`

4) chmgr → channel.hooks.on_rx（你的 on_rx 在这里触发）

- 位置：`User/Communication/core/comm.c: chmgr_dispatch_rx()`
- 行为：根据 `channel_id` 查找 `channel_t`，直接调用 `hooks.on_rx(ch, payload, len, ch->user)`
- 典型 on_rx 实现：
  - 云台：`User/Communication/channel/gimbal/gimbal_channel.c` → `static void on_rx(...)`
  - 相机：`User/Communication/channel/camera/camera_channel.c` → `static void camera_on_rx(...)`
  - 时间同步：`User/Communication/channel/time_sync/time_sync_channel.c` → `static void on_rx(...)`

5) on_tick（周期回调）从何而来？

- 位置：`User/Communication/core/comm.c: chmgr_tick()`
- 调用链：`comm_app.c` 的任务循环中调用 `chmgr_tick(&g_mgr)`，从而对每个通道触发 `hooks.on_tick()`
- 用途：
  - gimbal：到期发布状态（`gimbal_channel_publish`）
  - time_sync：发起方周期发 `TS_SID_REQ`

6) 入队与仲裁 → 真正发送

- 入队：`ch_uproto_queue_notify(bind, ch_id, sid, flags, pl, len)`（非阻塞，保留“最新”）
- 仲裁：`ch_uproto_arbiter_tick(&g_bind)`（在 `comm_app.c` 主循环内调用）
  - 优先级 > 饿死保护（age） > 同优先级轮转 → 选中一帧 → 组装 MUX → `uproto_send_notify(..., UPROTO_MSG_MUX, ...)` 写入端口 → USB CDC 发送
- 直接发送：`ch_uproto_send_notify(...)`（无需入队，但不利于控制带宽）

### ASCII 时序图（RX 路径）

```
USB OUT ISR → CDC_Receive_HS
              ↓
         uproto_on_rx_bytes ──► uproto_process_rx_buffer ──► handler(UPROTO_MSG_MUX)
                                                       ↓
                                                on_mux_rx(mux_decode)
                                                       ↓
                                         chmgr_dispatch_rx(id,payload)
                                                       ↓
                                       channel.hooks.on_rx(ch,p,len,user)
```

### ASCII 时序图（TX 路径）

```
on_tick / on_rx 触发发送 → ch_uproto_queue_notify
                              ↓(主循环)
                        ch_uproto_arbiter_tick  → mux_encode → uproto_send_notify
                                                             ↓
                                                        CDC IN 发送
```

---

## gimbal 通道（Device 重点）

- 配置：`User/Communication/channel/gimbal/gimbal_config.h`
  - `GIMBAL_CH_ID`（默认 4）、`GIMBAL_PRIORITY`、`GIMBAL_PUB_PERIOD_MS`
  - 字段裁剪：`GIMBAL_STATE_HAS_ENCODERS` / `GIMBAL_STATE_HAS_IMU`
  - DELTA 回执：`GIMBAL_DELTA_ACK_ENABLE`（默认 0=关闭）
- 消息格式（小端）：
  - STATE（设备→主机，SID=0x0201）：
    - 固定：`sid(2) + [enc_yaw(4),enc_pitch(4)]? + [yaw_udeg(4),pitch_udeg(4),roll_udeg(4)]? + ts_us(8)`
    - 实际字段由上述裁剪宏决定
  - DELTA（主机→设备，SID=0x0202）：
    - 固定：`sid(2) + delta_yaw_udeg(4) + delta_pitch_udeg(4) + status(2) + ts_us(8)`
- on_rx：解析 DELTA → 调用 `gimbal_on_delta()`（把命令写入邮箱）→ 可选回 ACK
- on_tick：到周期发布 STATE（`gimbal_channel_publish()`）

### 控制任务消费邮箱（示例）

```c
// 伪代码，放在你的云台控制任务循环中
#include "User/Communication/channel/gimbal/gimbal_channel.h"

static uint32_t mb_ver = 0; // 版本号用于判断是否有新命令
for(;;) {
    gimbal_cmd_t cmd;
    if (gimbal_mailbox_get(&cmd, &mb_ver)) {
        // 将“增量（微度）”转为角度/弧度，并叠加到目标
        double dyaw_deg   = (double)cmd.delta_yaw_udeg   / 1e6;
        double dpitch_deg = (double)cmd.delta_pitch_udeg / 1e6;
        target_yaw_deg   += dyaw_deg;
        target_pitch_deg += dpitch_deg;
        // 结合 cmd.status / cmd.ts_us 做额外处理（如时序对齐）
    }
    // 其余控制逻辑...
    osDelay(2); // 控制周期
}
```

---

## Host 使用说明（要点回顾）

- 参数：`--quiet` 关闭默认 TX/RX 打印；`--sine axis:freq:amp[@rate]` 启动正弦（发送 DELTA 增量）
- 关键代码：
  - `User/Communication/example/host/cpp/main.cpp`（参数解析、初始化、自动正弦）
  - `User/Communication/example/host/cpp/host_generic.hpp`（发送/接收、正弦生成、解析日志）
- 日志含义：
  - `TX CHx sid=...`：发送的 MUX 帧；`RX CHx sid=...`：接收的 MUX 帧
  - `[GIMBAL] ...`：云台状态解析后的数值
  - `[TS] RESP ...`：时间同步往返时延/偏移参考

---

## 调优建议

- 限制带宽：
  - 关闭 DELTA ACK：`#define GIMBAL_DELTA_ACK_ENABLE 0`
  - 降低发布周期：`#define GIMBAL_PUB_PERIOD_MS 20/50/100`
  - 裁剪状态字段：关闭 `GIMBAL_STATE_HAS_ENCODERS/IMU`
- USB CDC 发送堆积：
  - 增大 `USB_TX_RING_SIZE`、`USB_TX_CHUNK_MAX`（`USB_DEVICE/App/usbd_cdc_if.c`）
  - 确保 `ch_uproto_arbiter_tick(&g_bind)` 每次循环都调用
- Host 输出阻塞：使用 `--quiet`
- 时间基准：优先使用高精度 `DWT_GetTimeline_us`；time_sync 提供“设备逻辑时间”映射（`time_sync_channel_now_us`）

---

## 故障排查（Device/Host 对照）

- 完全收不到：
  - Host 端口确认：`uproto_host_cpp.exe --list`（Windows）
  - Device USB 枚举：`hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED`
  - 参数端点：`COMx` 或 `serial:COMx?baud=115200`
- 只见 TX，无响应：
  - （默认）ACK 关闭属正常；需要回执则打开 `GIMBAL_DELTA_ACK_ENABLE`
  - 绑定/注册检查：`ch_uproto_bind` → `ch_uproto_register_rx` → `setup_channels`
  - 分发链：`on_mux_rx → chmgr_dispatch_rx → channel.on_rx`
- 解析错误：
  - MUX 常量与 CRC：确保 `SOF/VER/CRC16` 一致，`COMM_MUX_TX_BUFFER_SIZE` 足够
  - USB OUT 回调：`CDC_Receive_HS()` 中必须 `uproto_on_rx_bytes()` 后 `USBD_CDC_ReceivePacket()`
  - on_rx 判长：`sid/len` 是否满足最小字段
- 时延/丢包：
  - 降频、裁剪字段、关闭 ACK；增大发送 ring，确保仲裁与 uproto tick 正常
- 正弦看不到效果：
  - 提高幅值/更新率；确认控制任务确实消费了 `gimbal_mailbox_get()` 并叠加到目标

---

## 扩展新通道（参考步骤）

1) 在 `example/shared/protocol_ids.h` 定义新 CH/SID
2) Device 侧：
   - 新建 `channel_xxx.[ch]`，实现 `on_rx/on_tick` 与 `pack`（若需要）
   - 在 `comm_app.c::setup_channels()` 调用 `channel_init`、`channel_bind_transport`、`chmgr_register`
3) Host 侧：
   - 在 `host_generic.hpp::handle()` 中识别新 CH/SID 并解析/打印

---

## 关键文件导航

- Host
  - 入口/参数：`User/Communication/example/host/cpp/main.cpp`
  - 发送/解析/正弦：`User/Communication/example/host/cpp/host_generic.hpp`
- Device
  - 通信任务：`User/Communication/example/device/comm_app.c`
  - USB CDC Port：`User/Communication/example/device/usb_cdc_port.c`
  - CDC 回调：`USB_DEVICE/App/usbd_cdc_if.c`
  - 通道管理/编码：`User/Communication/core/comm.[ch]`
  - uproto 核心：`User/Communication/core/uproto.[ch]`
  - 云台：`User/Communication/channel/gimbal/gimbal_channel.[ch]`、`gimbal_config.h`
  - 时间同步：`User/Communication/channel/time_sync/time_sync_channel.[ch]`

---

如需我将 Host 默认日志关闭（无需传 `--quiet`）、或把 DELTA ACK 改为“每 N 次回一次”，我可以继续调整默认策略与参数入口。