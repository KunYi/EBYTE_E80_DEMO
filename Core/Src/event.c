#include "event.h"

///事件组
static uint32_t volatile event_record = 0;

/**
 * @brief 设置全局事件标识
 *
 * @param event 事件标识类型，见结构体event_t
 */
void event_set(event_t event) { event_record |= event; }

/**
 * @brief 检查全局事件标识设置情况
 *
 * @param event 事件标识类型，见结构体event_t
 * @retval 0:未被设置  1：已设置
 */
uint8_t event_check(event_t event) { return (event_record & event) ? 1 : 0; }

/**
 * @brief 检查全局事件标识设置情况,如果已被设置则清除
 *
 * @param event 事件标识类型，见结构体event_t
 * @retval 0:未被设置  1：已设置
 */
uint8_t event_check_clear(event_t event) {
  uint8_t result = 0;

  if (event_record & event) {
    event_record &= ~event;

    result = 1;
  }

  return result;
}

/**
 * @brief 清除全局事件标识
 *
 * @param event 事件标识类型，见结构体event_t
 */
void event_clear(event_t event) { event_record &= ~event; }
