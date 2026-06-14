#include "usb_cdc_port.h"
#include "stm32h7xx_hal.h"
#include "usbd_core.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

#ifndef USB_RX_RING_SIZE
#define USB_RX_RING_SIZE 1024u
#endif

#ifndef USB_RX_POLL_CHUNK
#define USB_RX_POLL_CHUNK 128u
#endif

static uint8_t s_rx_ring[USB_RX_RING_SIZE];
static volatile uint16_t s_rx_wpos = 0u;
static volatile uint16_t s_rx_rpos = 0u;
static volatile uint16_t s_rx_count = 0u;
static uproto_context_t *s_ctx = NULL;

static uint32_t usb_cdc_port_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void usb_cdc_port_exit_critical(uint32_t primask)
{
    if (primask == 0u) {
        __enable_irq();
    }
}

void usb_cdc_port_init(void)
{
    usb_cdc_port_reset_rx();
}

uint32_t usb_cdc_port_write(void *user, const uint8_t *data, uint32_t len)
{
    (void)user;
    if (!data || len == 0) return 0u;
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) return 0u;

    uint32_t accepted = 0u;
    while (accepted < len) {
        uint16_t chunk = ((len - accepted) > 512u) ? 512u : (uint16_t)(len - accepted);
        uint8_t res = CDC_Transmit_HS((uint8_t *)(data + accepted), chunk);
        if (res != USBD_OK) {
            break;
        }
        accepted += chunk;
    }
    return accepted;
}

static void usb_cdc_port_flush(void *user)
{
    (void)user; /* No-op */
}

static uint16_t usb_cdc_port_get_mtu(void *user)
{
    (void)user;
    return 512u; /* HS bulk IN packet size */
}

void usb_cdc_port_get_ops(uproto_port_ops_t *ops)
{
    if (!ops) return;
    ops->write = usb_cdc_port_write;
    ops->flush = usb_cdc_port_flush;
    ops->get_mtu = usb_cdc_port_get_mtu;
    ops->user = NULL;
}

void usb_cdc_port_bind_uproto(uproto_context_t *ctx)
{
    s_ctx = ctx;
}

void usb_cdc_port_on_rx(const uint8_t *data, uint32_t len)
{
    uint32_t primask;

    if (!data || len == 0u) return;

    primask = usb_cdc_port_enter_critical();
    for (uint32_t i = 0u; i < len; i++) {
        if (s_rx_count >= USB_RX_RING_SIZE) {
            s_rx_rpos++;
            if (s_rx_rpos >= USB_RX_RING_SIZE) {
                s_rx_rpos = 0u;
            }
            s_rx_count--;
        }

        s_rx_ring[s_rx_wpos++] = data[i];
        if (s_rx_wpos >= USB_RX_RING_SIZE) {
            s_rx_wpos = 0u;
        }
        s_rx_count++;
    }
    usb_cdc_port_exit_critical(primask);
}

void usb_cdc_port_poll_rx(void)
{
    uint8_t chunk[USB_RX_POLL_CHUNK];

    if (!s_ctx) return;

    while (1) {
        uint16_t n;
        uint32_t primask;

        primask = usb_cdc_port_enter_critical();
        n = s_rx_count;
        if (n > USB_RX_POLL_CHUNK) {
            n = USB_RX_POLL_CHUNK;
        }

        for (uint16_t i = 0u; i < n; i++) {
            chunk[i] = s_rx_ring[s_rx_rpos++];
            if (s_rx_rpos >= USB_RX_RING_SIZE) {
                s_rx_rpos = 0u;
            }
        }
        s_rx_count = (uint16_t)(s_rx_count - n);
        usb_cdc_port_exit_critical(primask);

        if (n == 0u) {
            break;
        }

        uproto_on_rx_bytes(s_ctx, chunk, n);
    }
}

void usb_cdc_port_reset_rx(void)
{
    uint32_t primask = usb_cdc_port_enter_critical();
    s_rx_wpos = 0u;
    s_rx_rpos = 0u;
    s_rx_count = 0u;
    usb_cdc_port_exit_critical(primask);
}
