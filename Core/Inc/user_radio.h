#ifndef __USER_RADIO_H_
#define __USER_RADIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "event.h"
#include "lr11xx_hal.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system.h"
#include "lr11xx_types.h"
#include "string.h"
#include "user_uart.h"


#define E80_CONTEXT "LR1121"

#define IRQ_MASK                                                               \
  (LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE |                     \
   LR11XX_SYSTEM_IRQ_TIMEOUT | LR11XX_SYSTEM_IRQ_HEADER_ERROR |                \
   LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR)

extern uint8_t rx_buffer_lenght;
extern uint8_t rx_buffer[255];
extern lr11xx_bsp_tx_cfg_output_params_t output_params;

void radio_tx_auto(void);
void radio_tx_custom(uint8_t *buffer, uint8_t lenght);
void radio_rx(void);
void radio_sleep(void);
void radio_wakeup(void);
void radio_init(lr11xx_pa_type_t pa_type, int8_t power, uint32_t freq);
void radio_inits(lr11xx_pa_type_t pa_type, int8_t power, uint32_t freq,
                 lr11xx_radio_lora_sf_t sf, lr11xx_radio_lora_bw_t bw,
                 lr11xx_radio_lora_cr_t cr, uint8_t ldros, uint8_t synword);
void radio_init_up(void);
void radio_irq_callback(void);

#ifdef __cplusplus
}
#endif
#endif
