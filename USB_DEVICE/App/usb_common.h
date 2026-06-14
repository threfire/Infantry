#ifndef USB_COMMON_H__
#define USB_COMMON_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 usb_comm（创建线程、互斥等）
// 对于 CMSIS-RTOS v1，此函数可以在 RTOS 启动前调用（线程将被创建），
 // 或在 RTOS 启动后的初始化任务里调用。保证调用一次即可。
void usb_comm_init(void);

// 非阻塞发送：把 data 拷入发送队列并唤醒发送线程，返回实际入队字节数（0 表示队列满或忙）
uint32_t usb_comm_send(const uint8_t *data, uint32_t len);

// 供协议栈使用的写函数（可作为 uproto 的 port_ops.write）
// 原型：size_t (*)(void *user, const uint8_t *data, size_t len)
uint32_t usb_comm_port_write(void *user, const uint8_t *data, uint32_t len);

// （可选）直接把接收到的数据交给协议栈（若你要在 CDC 回调里直接调用）
// 我们提供此接口但默认实现中不需要修改原文件；如要直接触发接收，可调用它。
void usb_comm_on_rx_direct(uint8_t *buf, uint32_t len);

// 查询发送队列剩余空间（供上层决定是否重发）
uint32_t usb_comm_tx_free_space(void);

#ifdef __cplusplus
}
#endif

#endif // USB_COMM_H__

