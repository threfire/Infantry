#ifndef USB_CDC_PORT_H
#define USB_CDC_PORT_H

#include <stdint.h>
#include "uproto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize any internal state if needed (currently none) */
void usb_cdc_port_init(void);

/* Build default uproto port ops backed by USB CDC (HS) */
void usb_cdc_port_get_ops(uproto_port_ops_t *ops);

/* Direct write helper (non-blocking). Returns bytes accepted. */
uint32_t usb_cdc_port_write(void *user, const uint8_t *data, uint32_t len);


/* Bind uproto ctx so CDC RX callback can feed bytes in */
void usb_cdc_port_bind_uproto(uproto_context_t *ctx);

/* Queue RX bytes from CDC_Receive_xx callback */
void usb_cdc_port_on_rx(const uint8_t *data, uint32_t len);

/* Parse queued RX bytes from task context */
void usb_cdc_port_poll_rx(void);

/* Clear queued RX bytes on USB close/reset */
void usb_cdc_port_reset_rx(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_PORT_H */
