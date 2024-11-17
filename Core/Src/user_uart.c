#include "user_uart.h"
#include "event.h"
#include "fifo.h"


static fifo_t fifo_uart_tx;
static fifo_t fifo_uart_rx;
static volatile uint8_t uart_rx_is_busy = OFF;
static volatile uint8_t uart_tx_is_busy = OFF;

uint8_t uart_rx_timeout = 0;

extern UART_HandleTypeDef huart1;

static uint8_t uart0_interrupt_read(void) {
  uint8_t value;
  HAL_UART_Receive(&huart1, &value, 1, 1000);
  return value;
}

static void uart0_interrupt_send(uint8_t value) { huart1.Instance->DR = value; }

void uart_recvidle(void) {
  if (uart_rx_timeout != 0) {
    uart_rx_timeout--;
    if (uart_rx_timeout == 0) {
      event_set(EVENT_UART_RX_DONE);
      uart_rx_is_busy = OFF;
    }
  }
}

void uart_fifo_init(void) {
  /* 创建缓冲区 FIFO*/
  fifo_create(&fifo_uart_tx);
  fifo_create(&fifo_uart_rx);
}

void uart_write(uint8_t *buffer, uint16_t length) {
  uint8_t value;

  if (uart_tx_is_busy == ON) {
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);

    fifo_write(&fifo_uart_tx, buffer, length);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
  } else {
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);

    fifo_write(&fifo_uart_tx, buffer, length);

    fifo_read(&fifo_uart_tx, &value, 1);

    uart0_interrupt_send(value);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);

    uart_tx_is_busy = ON;
  }
}

void uart_read(uint8_t *buffer, uint16_t *length_out) {
  uint16_t read_length;

  read_length = fifo_get_length(&fifo_uart_rx);

  fifo_read(&fifo_uart_rx, buffer, read_length);

  *length_out = read_length;
}

void uart_rx_interrupt_callback(void) {
  uint8_t value;

  /* 读取接收字节 */
  value = uart0_interrupt_read();

  /* 写入FIFO */
  fifo_write(&fifo_uart_rx, &value, 1);

  /*刷新串口超时2ms*/
  uart_rx_timeout = 2;
}

void uart_tx_interrupt_callback(void) {
  /* 如果发送队列中还有数据 */
  if (fifo_get_length(&fifo_uart_tx)) {
    uint8_t value;
    /* 出队一字节 */
    fifo_read(&fifo_uart_tx, &value, 1);
    /* 调用硬件层串口字节发送函数 完成后自动触发下一次发送中断 */
    uart0_interrupt_send(value);
  } else {
    event_set(EVENT_UART_TX_DONE);
    uart_tx_is_busy = OFF;
  }
}

void uart_interrupt_callback(void) {
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) {
    //__HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_RXNE);
    uart_rx_interrupt_callback();
  } else if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != RESET) {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
    uart_tx_interrupt_callback();
  }
}
