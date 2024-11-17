#include "user_radio.h"
#include "event.h"
#include "lr11xx_radio.h"
#include "main.h"
#include "stdio.h"
#include "string.h"

#define SYNC_WORD_NO_RADIO 0x12
#define WIFI_WORD_NO_RADIO 0x21

extern UART_HandleTypeDef huart1;

/* 射频开关配置 */
lr11xx_system_rfswitch_cfg_t system_rf_switch_cfg = {
    .enable = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH |
              LR11XX_SYSTEM_RFSW2_HIGH | LR11XX_SYSTEM_RFSW3_HIGH,
    .standby = 0,
    .rx = LR11XX_SYSTEM_RFSW1_HIGH,
    .tx = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH,
    .tx_hp = LR11XX_SYSTEM_RFSW0_HIGH,
    .tx_hf = 0,
    .gnss = LR11XX_SYSTEM_RFSW2_HIGH,
    .wifi = LR11XX_SYSTEM_RFSW3_HIGH};

/* 射频参数配置 */
lr11xx_radio_pkt_params_lora_t radio_pkt_params = {
    .preamble_len_in_symb = 8,
    .header_type = LR11XX_RADIO_LORA_PKT_EXPLICIT,
    .pld_len_in_bytes = 255,
    .crc = true,
    .iq = false};

/* LoRa 参数配置 */
lr11xx_radio_mod_params_lora_t mod_params = {.ldro = 0,
                                             .bw = LR11XX_RADIO_LORA_BW_125,
                                             .cr = LR11XX_RADIO_LORA_CR_4_5,
                                             .sf = LR11XX_RADIO_LORA_SF8};

/* 低功耗配置 Supply current in power down mode */
lr11xx_system_sleep_cfg_t sleep_cfgs = {.is_rtc_timeout = false,
                                        .is_warm_start = false};

/* 用于存储射频发射参数（PA、功率等） */
lr11xx_bsp_tx_cfg_output_params_t output_params = {0};

/* 自动发送数据定义 */
uint8_t tx_buffer[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                       0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

/* 接收数据缓冲区 */
uint8_t rx_buffer[255] = {0};
uint8_t rx_buffer_lenght = 0;

/**
 * @brief GPIO中断服务函数
 * @param GPIO_Pin 引脚序号.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == E80_DIO9_Pin) {
    /* LR1121 中断服务函数 */
    radio_irq_callback();
  }
}

lr11xx_system_irq_mask_t radio_irq = LR11XX_SYSTEM_IRQ_NONE;
/**
 * @brief DIO9射频回调函数（射频接收及发送完成中断服务函数）
 * @param GPIO_Pin 引脚序号.
 * @retval None
 */
void radio_irq_callback(void) {
  // lr11xx_system_irq_mask_t radio_irq = LR11XX_SYSTEM_IRQ_NONE;
  /* 获取irq状态 */
  if (lr11xx_system_get_irq_status(E80_CONTEXT, &radio_irq) !=
      LR11XX_STATUS_OK) {
    HAL_UART_Transmit(&huart1, (const uint8_t *)"get_irq_status failed\r\n",
                      sizeof("get_irq_status failed\r\n"), 100);
  }
  /* LR1121 超时判断 */
  if ((radio_irq & LR11XX_SYSTEM_IRQ_TIMEOUT) == LR11XX_SYSTEM_IRQ_TIMEOUT) {
    /* LR1121 清除中断状态 */
    lr11xx_system_clear_irq_status(E80_CONTEXT, radio_irq);
    /* LR1121 进入接收 */
    radio_rx();
  } else if (radio_irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR) {
    HAL_UART_Transmit(&huart1, (const uint8_t *)"header_error\r\n",
                      sizeof("header_error\r\n"), 100);
    lr11xx_system_clear_irq_status(E80_CONTEXT, radio_irq);
    radio_rx();
  } else if (radio_irq & LR11XX_SYSTEM_IRQ_CRC_ERROR) {
    HAL_UART_Transmit(&huart1, (const uint8_t *)"crc_error\r\n",
                      sizeof("crc_error\r\n"), 100);
    lr11xx_system_clear_irq_status(E80_CONTEXT, radio_irq);
    radio_rx();
  }
  /* LR1121 接收完成 */
  else if ((radio_irq & LR11XX_SYSTEM_IRQ_RX_DONE) ==
           LR11XX_SYSTEM_IRQ_RX_DONE) {
    lr11xx_radio_rx_buffer_status_t rx_buffer_status;
    /* LR1121 获取接收数据包状态及参数 */
    lr11xx_radio_get_rx_buffer_status(E80_CONTEXT, &rx_buffer_status);
    rx_buffer_lenght = rx_buffer_status.pld_len_in_bytes;
    /* LR1121 读取FIFO 接收到的数据到 rx_buffer缓冲区*/
    lr11xx_regmem_read_buffer8(E80_CONTEXT, rx_buffer,
                               rx_buffer_status.buffer_start_pointer,
                               rx_buffer_status.pld_len_in_bytes);
    /* 产生有一个射频接收完成的事件 由主程序处理 */
    event_set(EVENT_RADIO_RX_DONE);
    /* 调试接口 */
    // HAL_UART_Transmit(&huart1, rx_buffer , rx_buffer_status.pld_len_in_bytes,
    // 100); memset(rx_buffer,0,sizeof(rx_buffer));

    /* LR1121 清除中断状态 */
    lr11xx_system_clear_irq_status(E80_CONTEXT, radio_irq);
    /* LR1121 进入接收 */
    radio_rx();
  }
  /* LR1121 发射完成 */
  else if ((radio_irq & LR11XX_SYSTEM_IRQ_TX_DONE) ==
           LR11XX_SYSTEM_IRQ_TX_DONE) {
    /* LR1121 清除中断状态 */
    lr11xx_system_clear_irq_status(E80_CONTEXT, radio_irq);
    // HAL_UART_Transmit(&huart1, (const uint8_t *)"TX DONE\r\n", sizeof("TX
    // DONE\r\n"), 100);
    /* LR1121 进入接收 */
    radio_rx();
  }
  /* LR1121 其他中断事件 */
  else {
    radio_rx();
  }
}

/**
 * @brief 射频初始化
 * @param lr11xx_pa_type_t pa_type：PA通道选择
 * @param int8_t power： 功率配置
 * @param uint32_t freq：频率配置
 * @retval None
 */
void radio_init(lr11xx_pa_type_t pa_type, int8_t power, uint32_t freq) {
  uint16_t errors;
  /* LR1121 reset */
  lr11xx_hal_reset(E80_CONTEXT);
  /* LR1121 wakeup */
  lr11xx_hal_wakeup(E80_CONTEXT);
  /* LR1121 进入standby */
  lr11xx_system_set_standby(E80_CONTEXT, LR11XX_SYSTEM_STANDBY_CFG_RC);
  /* LR1121 配置模式DC-DC */
  lr11xx_system_set_reg_mode(E80_CONTEXT, LR11XX_SYSTEM_REG_MODE_DCDC);
  /* LR1121 配置spi CRC 关闭 */
  lr11xx_system_enable_spi_crc(E80_CONTEXT, false);
  /* LR1121 配置LR1121 射频开关映射 */
  lr11xx_system_set_dio_as_rf_switch(E80_CONTEXT, &system_rf_switch_cfg);
  /* LR1121 配置TCXO供电电压和检测超时 */
  lr11xx_system_set_tcxo_mode(E80_CONTEXT, LR11XX_SYSTEM_TCXO_CTRL_1_8V, 320);
  /* LR1121 配置低频(LF)时钟 */
  lr11xx_system_cfg_lfclk(E80_CONTEXT, LR11XX_SYSTEM_LFCLK_XTAL, true);
  /* LR1121 清除错误状态 */
  lr11xx_system_clear_errors(E80_CONTEXT);
  /* LR1121 系统校准 */
  lr11xx_system_calibrate(E80_CONTEXT, 0x3F);
  /* LR1121 获取错误状态 */
  lr11xx_system_get_errors(E80_CONTEXT, &errors);

  if (errors & LR11XX_SYSTEM_ERRORS_IMG_CALIB_MASK) {
    HAL_UART_Transmit(&huart1, (const uint8_t *)"calibrate err\r\n",
                      sizeof("calibrate err\r\n"), 100);
  }
  /* LR1121 清除错误状态 */
  lr11xx_system_clear_errors(E80_CONTEXT);
  /* LR1121 清除irq标志 */
  lr11xx_system_clear_irq_status(E80_CONTEXT, LR11XX_SYSTEM_IRQ_ALL_MASK);
  /* LR1121 配置射频数据调制类型（LoRa） */
  lr11xx_radio_set_pkt_type(E80_CONTEXT, LR11XX_RADIO_PKT_TYPE_LORA);
  /* SPI 读写检查 */
  lr11xx_radio_pkt_type_t spi_check;
  lr11xx_radio_get_pkt_type(E80_CONTEXT, &spi_check);
  if (spi_check != LR11XX_RADIO_PKT_TYPE_LORA) {
    HAL_UART_Transmit(&huart1, (const uint8_t *)"spi_check_err\r\n",
                      sizeof("spi_check_err\r\n"), 100);
  }
  /* LR1121 配置射频数据包参数 */
  lr11xx_radio_set_lora_pkt_params(E80_CONTEXT, &radio_pkt_params);
  /* LR1121 配置射频同步字参数 */
  lr11xx_radio_set_lora_sync_word(E80_CONTEXT, SYNC_WORD_NO_RADIO);
  /* LR1121 配置射频数据调制参数 */
  lr11xx_radio_set_lora_mod_params(E80_CONTEXT, &mod_params);

  /* 2.4G LoRa 配置*/
  if (freq >= 2400000000) {
    /* LR1121 配置射频工作频点 */
    lr11xx_radio_set_rf_freq(E80_CONTEXT, freq);
    /* LR1121 获取射频发射配置参数 */
    lr11xx_get_tx_cfg(LR11XX_WITH_HF_PA, power, &output_params);
    /* LR1121 射频发射配置参数 */
    lr11xx_radio_set_pa_cfg(E80_CONTEXT, &output_params.pa_cfg);
  }
  /* SUB-G LoRa 配置*/
  else {
    /* LR1121 配置射频工作频点 */
    lr11xx_radio_set_rf_freq(E80_CONTEXT, freq);
    /* 高功率PA 配置*/
    if (pa_type == LR11XX_WITH_LF_HP_PA) {
      /* LR1121 获取射频发射配置参数 */
      lr11xx_get_tx_cfg(LR11XX_WITH_LF_HP_PA, power, &output_params);
      /* LR1121 射频发射配置参数 */
      lr11xx_radio_set_pa_cfg(E80_CONTEXT, &output_params.pa_cfg);
      lr11xx_radio_set_tx_params(E80_CONTEXT, power, LR11XX_RADIO_RAMP_48_US);
    }
    /* 低功率PA 配置*/
    else {
      /* LR1121 获取射频发射配置参数 */
      lr11xx_get_tx_cfg(LR11XX_WITH_LF_LP_PA, power, &output_params);
      /* LR1121 射频发射配置参数 */
      lr11xx_radio_set_pa_cfg(E80_CONTEXT, &output_params.pa_cfg);
    }
  }
  /* LR1121 模式自动退回配置 */
  lr11xx_radio_set_rx_tx_fallback_mode(E80_CONTEXT,
                                       LR11XX_RADIO_FALLBACK_STDBY_RC);
  /* LR1121 在接收中配置boost模式 允许增加~2dB的灵敏度
   * 代价是~2mA更高的电流消耗在RX模式 默认开启*/
  lr11xx_radio_cfg_rx_boosted(E80_CONTEXT, true);
}

/**
 * @brief 射频快速初始化（备用）
 * @param lr11xx_pa_type_t pa_type：PA通道选择
 * @param int8_t power： 功率配置
 * @param uint32_t freq：频率配置
 * @param lr11xx_radio_lora_sf_t sf：LoRa 扩频因子配置
 * @param lr11xx_radio_lora_bw_t bw：LoRa 带宽配置
 * @param lr11xx_radio_lora_cr_t cr：LoRa CRC配置
 * @param uint8_t ldros：低空速优化开关配置
 * @param uint8_t synword：LoRa 同步字配置
 * @retval None
 */
void radio_inits(lr11xx_pa_type_t pa_type, int8_t power, uint32_t freq,
                 lr11xx_radio_lora_sf_t sf, lr11xx_radio_lora_bw_t bw,
                 lr11xx_radio_lora_cr_t cr, uint8_t ldros, uint8_t synword) {
  uint16_t errors;

  lr11xx_hal_reset(E80_CONTEXT);

  lr11xx_hal_wakeup(E80_CONTEXT);

  lr11xx_system_set_standby(E80_CONTEXT, LR11XX_SYSTEM_STANDBY_CFG_XOSC);

  lr11xx_system_set_reg_mode(E80_CONTEXT, LR11XX_SYSTEM_REG_MODE_DCDC);

  lr11xx_system_enable_spi_crc(E80_CONTEXT, false);

  lr11xx_system_set_dio_as_rf_switch(E80_CONTEXT, &system_rf_switch_cfg);

  lr11xx_system_set_tcxo_mode(E80_CONTEXT, LR11XX_SYSTEM_TCXO_CTRL_1_8V, 320);

  lr11xx_system_cfg_lfclk(E80_CONTEXT, LR11XX_SYSTEM_LFCLK_XTAL, false);

  lr11xx_system_clear_errors(E80_CONTEXT);

  lr11xx_system_calibrate(E80_CONTEXT, 0x3F);

  lr11xx_system_get_errors(E80_CONTEXT, &errors);

  lr11xx_system_clear_errors(E80_CONTEXT);

  lr11xx_system_clear_irq_status(E80_CONTEXT, LR11XX_SYSTEM_IRQ_ALL_MASK);

  lr11xx_radio_set_pkt_type(E80_CONTEXT, LR11XX_RADIO_PKT_TYPE_LORA);

  lr11xx_radio_set_lora_pkt_params(E80_CONTEXT, &radio_pkt_params);

  lr11xx_radio_set_lora_sync_word(E80_CONTEXT, synword);

  mod_params.ldro = ldros;
  mod_params.bw = bw;
  mod_params.cr = cr;
  mod_params.sf = sf;
  lr11xx_radio_set_lora_mod_params(E80_CONTEXT, &mod_params);

  /* 2.4G LoRa */
  if (freq >= 2400000000) {
    lr11xx_radio_set_rf_freq(E80_CONTEXT, freq);
    lr11xx_get_tx_cfg(LR11XX_WITH_HF_PA, power, &output_params);
    lr11xx_radio_set_pa_cfg(E80_CONTEXT, &output_params.pa_cfg);
  }
  /* SUB-G LoRa */
  else {
    lr11xx_radio_set_rf_freq(E80_CONTEXT, freq);
    /* 高功率PA */
    if (pa_type == LR11XX_WITH_LF_HP_PA) {
      lr11xx_get_tx_cfg(LR11XX_WITH_LF_HP_PA, power, &output_params);
      lr11xx_radio_set_pa_cfg(E80_CONTEXT, &output_params.pa_cfg);
      lr11xx_radio_set_tx_params(E80_CONTEXT, power, LR11XX_RADIO_RAMP_48_US);
    }
    /* 低功率PA */
    else {
      lr11xx_get_tx_cfg(LR11XX_WITH_LF_LP_PA, power, &output_params);
      lr11xx_radio_set_pa_cfg(E80_CONTEXT, &output_params.pa_cfg);
    }
  }
  lr11xx_radio_set_rx_tx_fallback_mode(E80_CONTEXT,
                                       LR11XX_RADIO_FALLBACK_STDBY_RC);

  lr11xx_radio_cfg_rx_boosted(E80_CONTEXT, true);
}

/**
 * @brief 射频进入接收
 * @param None
 * @retval None
 */
void radio_rx(void) {
  /* LR1121 配置射频数据包长度 */
  radio_pkt_params.pld_len_in_bytes = 255;
  /* LR1121 配置射频数据长度 */
  lr11xx_radio_set_lora_pkt_params(E80_CONTEXT, &radio_pkt_params);
  /* LR1121 配置DIO9中断 */
  lr11xx_system_set_dio_irq_params(
      E80_CONTEXT,
      LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT |
          LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_CRC_ERROR,
      LR11XX_SYSTEM_IRQ_NONE);
  /* LR1121 清除DIO9中断状态 */
  lr11xx_system_clear_irq_status(E80_CONTEXT, LR11XX_SYSTEM_IRQ_ALL_MASK);
  /* LR1121 进入接收模式 */
  lr11xx_radio_set_rx(E80_CONTEXT, 0);
}

/**
 * @brief 射频发送
 * @param None
 * @retval None
 */
void radio_tx_auto(void) {
  /* LR1121 配置DIO9中断 */
  lr11xx_system_set_dio_irq_params(E80_CONTEXT, LR11XX_SYSTEM_IRQ_TX_DONE,
                                   LR11XX_SYSTEM_IRQ_NONE);
  /* LR1121 清除DIO9中断状态 */
  lr11xx_system_clear_irq_status(E80_CONTEXT, LR11XX_SYSTEM_IRQ_ALL_MASK);
  /* LR1121 获取需要发射数据长度 */
  radio_pkt_params.pld_len_in_bytes = sizeof(tx_buffer);
  /* LR1121 配置发射数据长度 */
  lr11xx_radio_set_lora_pkt_params(E80_CONTEXT, &radio_pkt_params);
  /* 写入数据到LR1121到FIFO */
  lr11xx_regmem_write_buffer8(E80_CONTEXT, tx_buffer, sizeof(tx_buffer));
  /* LR1121 开始发送 */
  lr11xx_radio_set_tx(E80_CONTEXT, 0);
}

/**
 * @brief 射频自定义发送API
 * @param uint8_t *buffer：指向发送数据
 * @param uint8_t lenght： 数据长度
 * @retval None
 */
void radio_tx_custom(uint8_t *buffer, uint8_t lenght) {
  /* LR1121 配置发射数据长度 */
  lr11xx_system_set_dio_irq_params(E80_CONTEXT, LR11XX_SYSTEM_IRQ_TX_DONE,
                                   LR11XX_SYSTEM_IRQ_NONE);
  /* LR1121 清除DIO9中断状态 */
  lr11xx_system_clear_irq_status(E80_CONTEXT, LR11XX_SYSTEM_IRQ_ALL_MASK);
  /* LR1121 获取需要发射数据长度 */
  radio_pkt_params.pld_len_in_bytes = lenght;
  /* LR1121 配置发射数据长度 */
  lr11xx_radio_set_lora_pkt_params(E80_CONTEXT, &radio_pkt_params);
  /* LR1121 配置发射数据长度 */
  lr11xx_regmem_write_buffer8(E80_CONTEXT, buffer, lenght);
  /* LR1121 开始发送 */
  lr11xx_radio_set_tx(E80_CONTEXT, 0);
}

/**
 * @brief 射频进入低功耗
 * @param None
 * @retval None
 */
void radio_sleep(void) {
  /* LR1121 进入standby */
  lr11xx_system_set_standby(E80_CONTEXT, LR11XX_SYSTEM_STANDBY_CFG_RC);
  /* LR1121 配置休眠时DIO状态 */
  lr11xx_system_drive_dio_in_sleep_mode(E80_CONTEXT, true);
  /* LR1121 进入休眠模式 */
  lr11xx_system_set_sleep(NULL, sleep_cfgs, 0);
}

/**
 * @brief 射频唤醒
 * @param None
 * @retval None
 */
void radio_wakeup(void) {
  /* LR1121 休眠唤醒 */
  lr11xx_hal_wakeup(E80_CONTEXT);
}
