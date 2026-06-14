// usb_comm.c - CMSIS-RTOS2 版本
#include "usb_common.h"
#include "cmsis_os2.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "..\\..\\User\\Communication\\core\\uproto.h"
#include <string.h>
#include <stdint.h>

/* 这些符号在你原有的 usbd_cdc_if.c / main.c 中定义（extern 引用） */
extern uint8_t  usb_buf[];    // usbd_cdc_if.c 中的接收缓冲（非 static）
extern uint16_t usb_buf_len;  // usbd_cdc_if.c 中的接收长度
extern uproto_context_t proto_ctx; // 你的协议上下文，在应用中定义并初始化
extern USBD_HandleTypeDef hUsbDeviceHS;

/* 配置（必要时调整） */
#ifndef USB_TX_RING_SIZE
#define USB_TX_RING_SIZE 2048U
#endif
#ifndef USB_TX_CHUNK_MAX
#define USB_TX_CHUNK_MAX 512U
#endif
#ifndef USB_RX_POLL_MS
#define USB_RX_POLL_MS 5U
#endif
#ifndef USB_TX_THREAD_STACK
#define USB_TX_THREAD_STACK 1024U
#endif

/* 发送队列 */
static uint8_t  usb_tx_ring[USB_TX_RING_SIZE];
static uint16_t usb_tx_wpos = 0;
static uint16_t usb_tx_count = 0;

/* RTOS 对象（CMSIS-RTOS2） */
static osMutexId_t usb_tx_mutex = NULL;
static osThreadId_t usb_tx_thread_id = NULL;
static osThreadId_t usb_rx_thread_id = NULL;

/* 信号位（线程间通知） */
#define USB_TX_SIGNAL (1U)

/* 计算读位置 */
static inline uint16_t usb_tx_read_pos(void)
{
    return (uint16_t)((usb_tx_wpos + USB_TX_RING_SIZE - usb_tx_count) % USB_TX_RING_SIZE);
}

/* 提交一段连续数据给 USB 库（在发送线程中安全调用） */
static void usb_do_send_from_queue(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
    if (hcdc == NULL) return;

    if (hcdc->TxState != 0) return; // USB 正忙

    if (usb_tx_count == 0) return;

    uint16_t read_pos = usb_tx_read_pos();
    uint16_t first_chunk = (uint16_t)((USB_TX_RING_SIZE - read_pos) < usb_tx_count ? (USB_TX_RING_SIZE - read_pos) : usb_tx_count);
    uint16_t send_len = (first_chunk > USB_TX_CHUNK_MAX) ? USB_TX_CHUNK_MAX : first_chunk;

    USBD_CDC_SetTxBuffer(&hUsbDeviceHS, &usb_tx_ring[read_pos], send_len);
    USBD_CDC_TransmitPacket(&hUsbDeviceHS);
}

/* 发送线程（被 usb_comm_send 唤醒） */
static void usb_tx_worker(void *arg)
{
    (void)arg;
    for (;;) {
        /* 等待 signal（新数据到来） */
        (void)osThreadFlagsWait(USB_TX_SIGNAL, osFlagsWaitAny, osWaitForever);

        /* 一旦被唤醒，循环发送直到队列为空 */
        while (1) {
            if (osMutexAcquire(usb_tx_mutex, 10) != osOK) {
                /* 获取互斥失败，短延时后重试 */
                osDelay(2);
                continue;
            }

            if (usb_tx_count == 0) {
                /* 没有数据，释放互斥并跳出循环回到等待 */
                osMutexRelease(usb_tx_mutex);
                break;
            }

            /* 如果 USB 空闲，则提交一段 */
            USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
            if (hcdc != NULL && hcdc->TxState == 0) {
                /* 提交一段发送（不更新 usb_tx_count，这将在完成后更新） */
                usb_do_send_from_queue();
                osMutexRelease(usb_tx_mutex);

                /* 等待发送完成（轮询 TxState）或超时 */
                uint32_t t0 = HAL_GetTick();
                while (1) {
                    USBD_CDC_HandleTypeDef *hh = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
                    if (hh == NULL) break;
                    if (hh->TxState == 0) {
                        /* 发送完成 —— 将刚刚发送的字节从队列中移除 */
                        if (osMutexAcquire(usb_tx_mutex, 100) == osOK) {
                            uint16_t read_pos = usb_tx_read_pos();
                            uint16_t first_chunk = (uint16_t)((USB_TX_RING_SIZE - read_pos) < usb_tx_count ? (USB_TX_RING_SIZE - read_pos) : usb_tx_count);
                            uint16_t send_len = (first_chunk > USB_TX_CHUNK_MAX) ? USB_TX_CHUNK_MAX : first_chunk;
                            if (send_len <= usb_tx_count) {
                                usb_tx_count = (uint16_t)(usb_tx_count - send_len);
                            } else {
                                usb_tx_count = 0;
                            }
                            osMutexRelease(usb_tx_mutex);
                        }
                        break;
                    }
                    if ((HAL_GetTick() - t0) > 200) {
                        /* 超时保护，放弃本次等待，回到循环 */
                        break;
                    }
                    osDelay(1);
                }
            } else {
                /* USB 忙或 hcdc NULL，释放互斥并稍候 */
                osMutexRelease(usb_tx_mutex);
                osDelay(2);
            }
        } // end inner while
    } // end forever
}

/* 接收线程：轮询 usb_buf_len（来自原 usbd_cdc_if.c），把数据拷到本地并调用 uproto 处理，随后清零 usb_buf */
static void usb_rx_worker(void *arg)
{
    (void)arg;
    for (;;) {
        if (usb_buf_len > 0) {
            uint16_t len;
            /* 短临界区复制并清空原始缓冲 */
            __disable_irq();
            len = usb_buf_len;
            if (len > 0) {
                /* local buffer 根据你的 USB_SIZE 大小调整，使用一个合理上限 */
                static uint8_t local_buf[4096];
                if (len > sizeof(local_buf)) len = sizeof(local_buf);
                memcpy(local_buf, usb_buf, len);
                /* 清零 usb_buf 中的有效区域并重置长度 */
                memset(usb_buf, 0, len);
                usb_buf_len = 0;
                __enable_irq();

                /* 在线程上下文中调用协议栈解析（安全） */
                uproto_on_rx_bytes(&proto_ctx, local_buf, len);
            } else {
                __enable_irq();
            }
        }
        osDelay(USB_RX_POLL_MS);
    }
}

/* 初始化：创建互斥与线程（CMSIS-RTOS2） */
void usb_comm_init(void)
{
    if (usb_tx_mutex == NULL) {
        usb_tx_mutex = osMutexNew(NULL);
    }

    if (usb_tx_thread_id == NULL) {
        static const osThreadAttr_t usb_tx_thread_attributes = {
            .name = "USB_TX_THREAD",
            .stack_size = USB_TX_THREAD_STACK * 4,
            .priority = (osPriority_t) osPriorityBelowNormal,
        };
        usb_tx_thread_id = osThreadNew(usb_tx_worker, NULL, &usb_tx_thread_attributes);
    }

    if (usb_rx_thread_id == NULL) {
        static const osThreadAttr_t usb_rx_thread_attributes = {
            .name = "USB_RX_THREAD",
            .stack_size = 512 * 4,
            .priority = (osPriority_t) osPriorityNormal,
        };
        usb_rx_thread_id = osThreadNew(usb_rx_worker, NULL, &usb_rx_thread_attributes);
    }

    /* 初始化 ring 状态 */
    usb_tx_wpos = 0;
    usb_tx_count = 0;
    memset(usb_tx_ring, 0, sizeof(usb_tx_ring));
}

/* 返回发送队列剩余空间 */
uint32_t usb_comm_tx_free_space(void)
{
    uint32_t free_space = 0;
    if (usb_tx_mutex && osMutexAcquire(usb_tx_mutex, 10) == osOK) {
        free_space = (size_t)(USB_TX_RING_SIZE - usb_tx_count);
        osMutexRelease(usb_tx_mutex);
    }
    return free_space;
}

/* 非阻塞地把 data 写入队列，返回写入的字节数 */
uint32_t usb_comm_send(const uint8_t *data, uint32_t len)
{
    if (!data || len == 0) return 0;

    if (usb_tx_mutex == NULL) return 0;

    if (osMutexAcquire(usb_tx_mutex, 0) != osOK) {
        return 0; // 无法马上获取互斥，返回 0
    }

    uint16_t free_space = (uint16_t)(USB_TX_RING_SIZE - usb_tx_count);
    uint16_t to_write = (len > free_space) ? free_space : (uint16_t)len;
    if (to_write == 0) {
        osMutexRelease(usb_tx_mutex);
        return 0;
    }

    uint16_t wpos = usb_tx_wpos;
    uint16_t first = (USB_TX_RING_SIZE - wpos);
    if (first > to_write) first = to_write;

    memcpy(&usb_tx_ring[wpos], data, first);
    if (to_write > first) {
        memcpy(&usb_tx_ring[0], data + first, (to_write - first));
        wpos = (uint16_t)(to_write - first);
    } else {
        wpos = (uint16_t)(wpos + first);
        if (wpos >= USB_TX_RING_SIZE) wpos -= USB_TX_RING_SIZE;
    }

    usb_tx_wpos = wpos;
    usb_tx_count = (uint16_t)(usb_tx_count + to_write);

    osMutexRelease(usb_tx_mutex);

    /* 唤醒发送线程（设置信号） */
    if (usb_tx_thread_id) {
        (void)osThreadFlagsSet(usb_tx_thread_id, USB_TX_SIGNAL);
    }

    return (uint32_t)to_write;
}

/* wrapper for uproto */
uint32_t usb_comm_port_write(void *user, const uint8_t *data, uint32_t len)
{
    (void)user;
    return usb_comm_send(data, len);
}

/* 若需要在 CDC 回调直接传入，可调用此接口（可选） */
void usb_comm_on_rx_direct(uint8_t *buf, uint32_t len)
{
    if (!buf || len == 0) return;
    uproto_on_rx_bytes(&proto_ctx, buf, len);
}



