/*
 * lv_fw_common_utility.h
 *
 */

#ifndef LV_FW_COMMON_UTILITY_H_
#define LV_FW_COMMON_UTILITY_H_

#include "stm32f4xx_hal.h"
//#include "lv_fw_common_platform.h"
#include <stdint.h>

#define LV_FW_UP_ERROR_NONE 0

typedef enum {
  LV_FW_RUN_STATE_NONE = 0,
  LV_FW_RUN_STATE_BOOT,
  LV_FW_RUN_STATE_APP
} LV_FW_RUN_STATE;

typedef enum {
  LV_FW_JUMP_MODE_NONE = 0,
  LV_FW_JUMP_MODE_BOOT,
  LV_FW_JUMP_MODE_APP,
  LV_FW_JUMP_MODE_RESET = 0xFF,
} LV_FW_JUMP_MODE;

typedef enum {
  LV_FW_UP_FLAG_FACE = 0x01,
  LV_FW_UP_FLAG_TOUCH = 0x02,
  LV_FW_UP_FLAG_SERVO = 0x04,
  LV_FW_UP_FLAG_BODY = 0x08,
  LV_FW_UP_FLAG_TAIL = 0x10,
  LV_FW_UP_FLAG_WHEEL = 0x20,
  LV_FW_UP_FLAG_POWER = 0x40,
  LV_FW_UP_FLAG_HORN = 0x01,
  LV_FW_UP_FLAG_RADAR = 0x01,
  LV_FW_UP_FLAG_BASE = 0x01
} LV_FW_UP_FLAG;

typedef enum {
  LV_FW_UP_ERROR_FLAG_FACE = 0x100,
  LV_FW_UP_ERROR_FLAG_TOUCH = 0x200,
  LV_FW_UP_ERROR_FLAG_SERVO = 0x400,
  LV_FW_UP_ERROR_FLAG_BODY = 0x800,
  LV_FW_UP_ERROR_FLAG_TAIL = 0x1000,
  LV_FW_UP_ERROR_FLAG_WHEEL = 0x2000,
  LV_FW_UP_ERROR_FLAG_POWER = 0x4000,
  LV_FW_UP_ERROR_FLAG_BASE = 0x100,
} LV_FW_UP_ERROR_FLAG;

typedef enum {
  LV_FW_UP_CMD_ERASE = 0x01,
  LV_FW_UP_CMD_FLASH_WRITE = 0x02,
  LV_FW_UP_CMD_CHILD_FWUP = 0x03,
  LV_FW_UP_CMD_STATE_CLEAR = 0x04,
  LV_FW_UP_CMD_RCV_CNT_CLEAR = 0x5,
  LV_FW_UP_CMD_FWUP_CMP = 0x6,
  LV_FW_UP_CMD_BACKUP_APP = 0x7,
} LV_FW_UP_CMD;

typedef enum {
  LV_FW_UP_CHILD_RESULT_HORN        = 0x00000001,
  LV_FW_UP_CHILD_RESULT_FACE        = 0x00000002,
  LV_FW_UP_CHILD_RESULT_BODY        = 0x00000004,
  LV_FW_UP_CHILD_RESULT_NECK_YAW    = 0x00000008,
  LV_FW_UP_CHILD_RESULT_NECK_FRONT  = 0x00000010,
  LV_FW_UP_CHILD_RESULT_NECK_LEFT   = 0x00000020,
  LV_FW_UP_CHILD_RESULT_NECK_RIGHT  = 0x00000040,
  LV_FW_UP_CHILD_RESULT_LEFT_ARM    = 0x00000080,
  LV_FW_UP_CHILD_RESULT_RIGHT_ARM   = 0x00000100,
  LV_FW_UP_CHILD_RESULT_LEFT_HAND   = 0x00000200,
  LV_FW_UP_CHILD_RESULT_RIGHT_HAND  = 0x00000400,
  LV_FW_UP_CHILD_RESULT_LEFT_LEG    = 0x00000800,
  LV_FW_UP_CHILD_RESULT_RIGHT_LEG   = 0x00001000,
  LV_FW_UP_CHILD_RESULT_CASTER      = 0x00002000,
  LV_FW_UP_CHILD_RESULT_WHEEL_LEFT  = 0x00004000,
  LV_FW_UP_CHILD_RESULT_WHEEL_RIGHT = 0x00008000,
  LV_FW_UP_CHILD_RESULT_TAIL        = 0x00010000,
  LV_FW_UP_CHILD_RESULT_POWER       = 0x00020000,
  LV_FW_UP_CHILD_RESULT_BASE        = 0x00040000,
  LV_FW_UP_CHILD_RESULT_TOUCH_HEAD  = 0x00080000,
  LV_FW_UP_CHILD_RESULT_TOUCH_ARM   = 0x00100000,
  LV_FW_UP_CHILD_RESULT_TOUCH_LOWER = 0x00200000
} LV_FW_UP_CHILD_RESULT;

/**
 * @brief sync実行結果
 */
typedef enum {
  LV_FW_COMMON_SYNC_OK,
  LV_FW_COMMON_SYNC_PROC_OVERTIME,
  LV_FW_COMMON_SYNC_INVALID_ARG
} LV_FW_COMMON_SYNC_RESULT;

/**
 * @brief Lovot Model情報
 */
typedef enum {
  LV_FW_LOVOT_MODEL_LV100,
  LV_FW_LOVOT_MODEL_LV101
} LV_FW_LOVOT_MODEL;

/**
 * @brief NestMode定義
 */
typedef enum {
  LV_FW_NEST_MODE_RANGING = 0,
  LV_FW_NEST_MODE_SONAR_SEARCH,
  LV_FW_NEST_MODE_SONAR,
  LV_FW_NEST_MODE_DEPARTURE
} LV_FW_NEST_MODE;

/**
 * @brief 時間取得
 * @return
 */
uint64_t lv_fw_common_get_ustick(void);

/**
 * @brief fps同期
 * @param fps
 * @return
 */
LV_FW_COMMON_SYNC_RESULT lv_fw_common_sync_fps(uint32_t fps);

/**
 * @brief μsecDelay
 */
void lv_fw_common_udelay(uint64_t time);

#endif /* FW_COMMON_UTILITY_H_ */
