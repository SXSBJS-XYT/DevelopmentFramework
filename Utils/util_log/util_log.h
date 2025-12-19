/**
 * @file    util_log.h
 * @brief   Utils层 - 轻量级日志系统
 * @author  SXSBJS_XYT (https://github.com/SXSBJS-XYT)
 *
 * 格式: [00001234][INFO][TAG] message
 *
 * 使用:
 *   LOG_I("APP", "Hello %d", 123);
 */

#ifndef __UTIL_LOG_H__
#define __UTIL_LOG_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/*                              配置                                           */
/*============================================================================*/

/* 日志总开关（Release设为0） */
#define UTIL_LOG_ENABLE 1

  /*============================================================================*/
  /*                              类型                                           */
  /*============================================================================*/

  typedef enum
  {
    LOG_LVL_ERROR = 0,
    LOG_LVL_WARN,
    LOG_LVL_INFO,
    LOG_LVL_DEBUG,
  } Util_LogLevel_t;

  /*============================================================================*/
  /*                              API                                            */
  /*============================================================================*/

  /**
   * @brief  初始化日志（设置默认级别）
   */
  void Util_Log_Init(void);

  /**
   * @brief  设置日志级别
   */
  void Util_Log_SetLevel(Util_LogLevel_t level);

  /**
   * @brief  开关时间戳
   */
  void Util_Log_EnableTimestamp(bool enable);

  /**
   * @brief  日志输出（内部用，建议用宏）
   */
  void Util_Log_Output(Util_LogLevel_t level, const char *tag, const char *fmt, ...);

  /*============================================================================*/
  /*                              日志宏                                         */
  /*============================================================================*/

#if UTIL_LOG_ENABLE
#define LOG_E(tag, fmt, ...) Util_Log_Output(LOG_LVL_ERROR, tag, fmt, ##__VA_ARGS__)
#define LOG_W(tag, fmt, ...) Util_Log_Output(LOG_LVL_WARN, tag, fmt, ##__VA_ARGS__)
#define LOG_I(tag, fmt, ...) Util_Log_Output(LOG_LVL_INFO, tag, fmt, ##__VA_ARGS__)
#define LOG_D(tag, fmt, ...) Util_Log_Output(LOG_LVL_DEBUG, tag, fmt, ##__VA_ARGS__)
#else
#define LOG_E(tag, fmt, ...) ((void)0)
#define LOG_W(tag, fmt, ...) ((void)0)
#define LOG_I(tag, fmt, ...) ((void)0)
#define LOG_D(tag, fmt, ...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __UTIL_LOG_H__ */
