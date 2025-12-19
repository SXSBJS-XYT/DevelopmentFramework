/**
 * @file    util_log.c
 * @brief   Utils层 - 轻量级日志系统实现
 * @author  SXSBJS_XYT (https://github.com/SXSBJS-XYT)
 *
 * 依赖: printf重定向到串口（需用户在main.c实现fputc）
 */

#include "util_log.h"
#include <stdio.h>
#include <stdarg.h>

#if UTIL_LOG_ENABLE

/*============================================================================*/
/*                              内部数据                                       */
/*============================================================================*/

static const char *s_level_str[] = {
    [LOG_LVL_ERROR] = "ERROR",
    [LOG_LVL_WARN]  = "WARN ",
    [LOG_LVL_INFO]  = "INFO ",
    [LOG_LVL_DEBUG] = "DEBUG",
};

static bool s_initialized      = false;
static Util_LogLevel_t s_level = LOG_LVL_DEBUG;
static bool s_timestamp_en     = true;

/* 获取时间戳（弱定义，用户可覆盖） */
__attribute__((weak)) uint32_t Util_Log_GetTick(void)
{
  extern uint32_t HAL_GetTick(void);
  return HAL_GetTick();
}

/*============================================================================*/
/*                              API实现                                        */
/*============================================================================*/

void Util_Log_Init(void)
{
  s_level        = LOG_LVL_DEBUG;
  s_timestamp_en = true;
  s_initialized  = true;
}

void Util_Log_SetLevel(Util_LogLevel_t level)
{
  s_level = level;
}

void Util_Log_EnableTimestamp(bool enable)
{
  s_timestamp_en = enable;
}

void Util_Log_Output(Util_LogLevel_t level, const char *tag, const char *fmt, ...)
{
  /* 未初始化不输出 */
  if (!s_initialized)
    return;

  /* 级别过滤 */
  if (level > s_level)
    return;

  /* 时间戳 */
  if (s_timestamp_en)
  {
    printf("[%08lu]", (unsigned long)Util_Log_GetTick());
  }

  /* 级别 + 标签 */
  printf("[%s][%s] ", s_level_str[level], tag);

  /* 用户消息 */
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);

  printf("\r\n");
}

#endif /* UTIL_LOG_ENABLE */
