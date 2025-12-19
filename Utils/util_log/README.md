# Utils层 - 轻量级日志

## 文件

```
Utils/
├── util_log.h
└── util_log.c
```

## 使用

**1. main.c 添加printf重定向：**

```c
#include <stdio.h>

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}
```

**2. 初始化日志：**

```c
#include "util_log.h"

int main(void)
{
    // HAL初始化...
    
    Util_Log_Init();  /* 必须先初始化 */
    
    LOG_I("APP", "System started");
}
```

**3. 使用日志：**

```c
LOG_E("TAG", "Error: %d", err);   /* 错误 */
LOG_W("TAG", "Warning");          /* 警告 */
LOG_I("TAG", "Info");             /* 信息 */
LOG_D("TAG", "Debug: %s", str);   /* 调试 */
```

## 输出示例

```
[00001234][ERROR][NET] Error: -1
[00001235][WARN ][APP] Warning
[00001236][INFO ][APP] System started
[00001237][DEBUG][FSM] Debug: Idle
```

## 编译配置

在 `util_log.h` 中修改：

```c
#define UTIL_LOG_ENABLE   1   /* 总开关，Release设0 */
```

## 运行时控制

```c
/* 级别过滤（只输出ERROR和WARN） */
Util_Log_SetLevel(LOG_LVL_WARN);

/* 关闭时间戳 */
Util_Log_EnableTimestamp(false);
```

## 日志级别

| 级别 | 宏 | 说明 |
|------|-----|------|
| ERROR | LOG_E | 错误，必须处理 |
| WARN | LOG_W | 警告，可能有问题 |
| INFO | LOG_I | 普通信息 |
| DEBUG | LOG_D | 调试信息 |

设置级别后，**低于该级别的日志不输出**：
- `LOG_LVL_ERROR` - 只输出 ERROR
- `LOG_LVL_WARN` - 输出 ERROR + WARN
- `LOG_LVL_INFO` - 输出 ERROR + WARN + INFO
- `LOG_LVL_DEBUG` - 全部输出（默认）
