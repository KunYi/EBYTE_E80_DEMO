#ifndef __USER_UART_H_
#define __USER_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

typedef enum {
  OFF = 0x00,
  ON = 0x01,
} on_off_t;

void uart_write(uint8_t *buffer, uint16_t length);
void uart_read(uint8_t *buffer, uint16_t *length_out);
void uart_rx_interrupt_callback(void);
void uart_tx_interrupt_callback(void);
void uart_interrupt_callback(void);
void uart_fifo_init(void);

#ifdef __cplusplus
}
#endif
#endif
