/**
 * @file lv_fw_common_utility
 * @brief
 * @autor GROOVE X,Inc
 * @version
 * @date 2018-04-18
 */

#include "lv_fw_common_utility.h"

static uint32_t lasttick_ms = 0;
static uint32_t lasttick_us = 0;
static uint64_t sync_starttime = 0L;

/**
 * @brief 時間取得
 * @return ustick
 */
uint64_t lv_fw_common_get_ustick(void) {
  uint32_t ms = HAL_GetTick();
  uint32_t us =
      (uint32_t) (((uint64_t) (SysTick->LOAD - SysTick->VAL) * 1000 / (SysTick->LOAD + 1)));
  if (us < lasttick_us && ms == lasttick_ms) {
    ms++;
  }
  lasttick_ms = ms;
  lasttick_us = us;
  return (uint64_t) ms * 1000 + us;
}

/**
 * @brief fps同期
 * @param fps
 * @return
 */
LV_FW_COMMON_SYNC_RESULT lv_fw_common_sync_fps(uint32_t fps) {
  LV_FW_COMMON_SYNC_RESULT ret = LV_FW_COMMON_SYNC_PROC_OVERTIME;

  if (fps == 0) {
    return LV_FW_COMMON_SYNC_INVALID_ARG;
  }

  uint64_t waitus = 1000000 / fps;
  uint64_t now = lv_fw_common_get_ustick();

  while ((now - sync_starttime) < waitus) {
    __NOP();
    ret = LV_FW_COMMON_SYNC_OK;
    now = lv_fw_common_get_ustick();
  }
  sync_starttime = now;
  return ret;
}

/**
 * @brief μsecDelay
 * @param time
 */
void lv_fw_common_udelay(uint64_t time) {
  uint64_t waitus = lv_fw_common_get_ustick() + time;

  while (waitus > lv_fw_common_get_ustick()) {
    __NOP();
  }
}
