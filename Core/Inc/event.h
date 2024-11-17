#ifndef __EVENT_H_
#define __EVENT_H_

#include <stdint.h>

typedef enum {
  EVENT_NONE = 0x0000,
  EVENT_UART_RX_DONE = 0x0001,
  EVENT_UART_TX_DONE = 0x0002,
  EVENT_RADIO_RX_DONE = 0x0004,
  EVENT_MODE_SWITCH = 0x0008,
  EVENT_RADIO_RESET = 0x0010,
  EVENT_ALL = 0xFFFF,
} event_t;

void event_clear(event_t event);
void event_set(event_t event);
uint8_t event_check(event_t event);
uint8_t event_check_clear(event_t event);

#endif
