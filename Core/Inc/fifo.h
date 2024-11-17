/*
 * fifo.h
 *
 *  Created on: 2021年10月9日
 *      Author: jiangH
 */

#ifndef _FIFO_H_
#define _FIFO_H_

#include <stdint.h>

// Warning ! Size must be 2^n ! Please view linux kfifo
#define USER_FIFO_SIZE 1024
typedef struct {
  uint32_t in;
  uint32_t out;
  uint32_t size;
  uint8_t buffer[USER_FIFO_SIZE];

} fifo_t;

uint8_t fifo_create(fifo_t *fifo);
uint8_t fifo_write(fifo_t *fifo, uint8_t *pData, uint32_t length);
uint32_t fifo_get_length(fifo_t *fifo);
uint32_t fifo_get_remain_length(fifo_t *fifo);
uint8_t fifo_read(fifo_t *fifo, uint8_t *pData, uint32_t length);
uint8_t fifo_clear(fifo_t *fifo);

#endif /* GENERALLIBRARY_FIFO_H_ */
