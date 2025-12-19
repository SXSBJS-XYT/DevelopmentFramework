# 嵌入式代码框架设计规范

> 版本：V1.0  
> 适用范围：STM32系列MCU项目

---

## 目录

1. [概述](#1-概述)
2. [设计理念](#2-设计理念)
3. [分层架构](#3-分层架构)
4. [各层详解](#4-各层详解)
5. [调用规则](#5-调用规则)
6. [命名规范](#6-命名规范)
7. [目录结构](#7-目录结构)
8. [开发流程](#8-开发流程)
9. [示例代码](#9-示例代码)
10. [常见问题](#10-常见问题)

---

## 1. 概述

### 1.1 文档目的

本文档定义了嵌入式软件开发的代码框架规范，旨在：

- 统一团队代码风格，提高代码可读性
- 明确模块边界，降低耦合度
- 提高代码复用性，便于跨项目移植
- 规范开发流程，提升团队协作效率

### 1.2 适用场景

- 基于STM32的裸机开发
- 基于RT-Thread/FreeRTOS的RTOS开发
- 中小型嵌入式项目（代码量1万~10万行）

### 1.3 框架特点

| 特点 | 说明 |
|------|------|
| **分层清晰** | 7层架构，职责明确，边界清晰 |
| **高内聚低耦合** | 模块独立，依赖单向，易于维护 |
| **可移植性强** | 硬件相关代码隔离，移植只需修改底层 |
| **可扩展性好** | 新增功能只需在对应层添加模块 |
| **规范统一** | 命名、注释、文件组织统一规范 |

---

## 2. 设计理念

### 2.1 核心原则

#### 原则一：分层隔离

```
高层模块不应依赖低层模块的实现细节，两者都应依赖抽象。
```

**实践**：App层调用Device层的`Dev_Led_On()`，而不是直接调用`HAL_GPIO_WritePin()`。当硬件变更时，只需修改Device层，App层无需改动。

#### 原则二：单向依赖

```
依赖关系只能从上到下，禁止下层调用上层。
```

**实践**：Driver层不能调用App层的函数。如果底层需要通知上层，使用**回调函数**或**事件机制**。

#### 原则三：单一职责

```
每个模块只做一件事，并把它做好。
```

**实践**：`drv_uart.c`只负责UART硬件操作，不包含协议解析；协议解析放在Protocol层。

#### 原则四：接口稳定

```
对外接口保持稳定，内部实现可以变化。
```

**实践**：`Dev_Sensor_GetTemp()`接口不变，内部可以从ADC读取改为I2C读取。

### 2.2 设计目标

```
┌─────────────────────────────────────────────────────────────┐
│                        设计目标                              │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   可读性        可维护性        可移植性        可测试性       │
│      │             │              │               │         │
│      ↓             ↓              ↓               ↓         │
│   命名规范      低耦合         硬件隔离        模块独立        │
│   注释完整      高内聚         抽象接口        依赖注入        │
│   结构清晰      单一职责       配置分离        Mock友好        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 分层架构

### 3.1 架构总览

```
┌─────────────────────────────────────────────────────────────┐
│                        App 应用层                           │
│                    (业务逻辑、任务调度)                      │
└─────────────────────────────────────────────────────────────┘
                              │
           ┌──────────────────┼──────────────────┐
           ↓                  ↓                  ↓
┌─────────────────┐    ┌───────────────┐  ┌────────────────┐
│  Device 设备层   │--->│ Service 服务层│  │ Protocol 协议层 │
│   (设备抽象)     │--->│  (中间件/框架) │  │   (通信协议)   │
└─────────────────┘    └───────────────┘  └────────────────┘
           │                  │                  │
           └──────────────────┼──────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                      Driver 驱动层                          │
│                      (外设驱动)                             │
└─────────────────────────────────────────────────────────────┘
                              │
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                       BSP 板级层                            │
│                   (硬件配置、引脚定义)                       │
└─────────────────────────────────────────────────────────────┘
                              │
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                      HAL / Lib 库层                         │
│                   (ST HAL库、第三方库)                       │
└─────────────────────────────────────────────────────────────┘


                    ┌─────────────────┐
                    │   Utils 工具层   │
                    │  (纵向贯穿)      │
                    │  日志/FIFO/CRC   │
                    └─────────────────┘
                           ↑
                      所有层可调用
```

### 3.2 层次说明

| 层级 | 名称 | 前缀 | 职责 |
|------|------|------|------|
| L6 | App | App_ | 业务逻辑、任务流程、状态机实例 |
| L5 | Device | Dev_ | 设备抽象、设备管理、统一接口 |
| L5 | Service | Srv_ | 中间件、框架、算法 |
| L5 | Protocol | Proto_ | 通信协议封装与解析 |
| L4 | Driver | Drv_ | 外设驱动、硬件操作封装 |
| L3 | BSP | Bsp_ | 板级支持、引脚定义、时钟配置 |
| L2 | HAL/Lib | - | ST HAL库、第三方库 |
| - | Utils | Util_ | 通用工具（纵向贯穿，所有层可用） |

---

## 4. 各层详解

### 4.1 App 应用层

**职责**：
- 实现业务逻辑
- 任务调度与流程控制
- 状态机实例化与运行
- 系统初始化入口

**特点**：
- 项目相关，通常不可复用
- 调用Device/Service/Protocol层完成功能
- 不直接操作硬件

**典型文件**：
```
App/
├── app_main.c           # 主入口、系统初始化
├── app_task_motor.c     # 电机控制任务
├── app_task_comm.c      # 通信任务
└── app_task_display.c   # 显示任务
```

**示例**：
```c
/* app_task_motor.c */

#include "app_task_motor.h"
#include "dev_motor.h"
#include "srv_fsm.h"
#include "srv_pid.h"

/* 电机控制任务 */
void App_Task_Motor_Run(void)
{
    /* 状态机处理 */
    Srv_Fsm_Process(&s_motor_fsm);
    
    /* PID控制 */
    float speed = Dev_Motor_GetSpeed();
    float output = Srv_Pid_Compute(&s_pid, speed);
    Dev_Motor_SetPwm(output);
}
```

---

### 4.2 Device 设备层

**职责**：
- 设备抽象与封装
- 提供统一的设备操作接口
- 设备状态管理
- 屏蔽硬件差异

**特点**：
- 面向"设备"而非"外设"
- 一个设备可能使用多个外设
- 接口稳定，实现可变

**典型文件**：
```
Device/
├── dev_led.c            # LED设备
├── dev_key.c            # 按键设备
├── dev_motor.c          # 电机设备
├── dev_sensor_temp.c    # 温度传感器
└── dev_display.c        # 显示屏设备
```

**Device vs Driver 区别**：
```c
/* Driver层 - 面向外设，关注"怎么操作" */
void Drv_Gpio_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t state);
void Drv_Pwm_SetDuty(TIM_HandleTypeDef *htim, uint32_t ch, uint16_t duty);
void Drv_Adc_Read(ADC_HandleTypeDef *hadc, uint16_t *value);

/* Device层 - 面向设备，关注"是什么" */
void Dev_Led_On(uint8_t id);              // 内部调用Drv_Gpio
void Dev_Motor_SetSpeed(int16_t rpm);     // 内部调用Drv_Pwm
float Dev_Sensor_GetTemp(void);           // 内部调用Drv_Adc + 转换公式
```

**示例**：
```c
/* dev_motor.c */

#include "dev_motor.h"
#include "drv_pwm.h"
#include "drv_gpio.h"
#include "drv_encoder.h"

/* 设备上下文 */
static Dev_MotorCtx_t s_motor_ctx;

void Dev_Motor_Init(void)
{
    Drv_Pwm_Init();
    Drv_Encoder_Init();
    s_motor_ctx.speed = 0;
    s_motor_ctx.state = MOTOR_STATE_STOP;
}

void Dev_Motor_SetSpeed(int16_t rpm)
{
    /* 方向控制 */
    if (rpm >= 0) {
        Drv_Gpio_Write(MOTOR_DIR_PORT, MOTOR_DIR_PIN, 1);
    } else {
        Drv_Gpio_Write(MOTOR_DIR_PORT, MOTOR_DIR_PIN, 0);
        rpm = -rpm;
    }
    
    /* 速度转PWM占空比 */
    uint16_t duty = (uint16_t)(rpm * 100 / MOTOR_MAX_RPM);
    Drv_Pwm_SetDuty(&htim1, TIM_CHANNEL_1, duty);
    
    s_motor_ctx.target_speed = rpm;
}

int16_t Dev_Motor_GetSpeed(void)
{
    /* 从编码器读取实际转速 */
    int32_t count = Drv_Encoder_GetCount();
    return (int16_t)(count * 60 / ENCODER_PPR / SAMPLE_TIME_S);
}
```

---

### 4.3 Service 服务层

**职责**：
- 提供通用中间件
- 实现与业务无关的框架
- 封装通用算法

**特点**：
- 业务无关，高度可复用
- 跨项目移植
- 不依赖具体硬件

**典型文件**：
```
Service/
├── Fsm/
│   ├── srv_fsm.c        # 状态机框架
│   └── srv_fsm.h
├── Pid/
│   ├── srv_pid.c        # PID算法
│   └── srv_pid.h
├── Timer/
│   ├── srv_soft_timer.c # 软件定时器
│   └── srv_soft_timer.h
└── Filter/
    ├── srv_filter.c     # 滤波算法
    └── srv_filter.h
```

**示例**：
```c
/* srv_pid.c */

#include "srv_pid.h"

void Srv_Pid_Init(Srv_Pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
}

float Srv_Pid_Compute(Srv_Pid_t *pid, float setpoint, float measured)
{
    float error = setpoint - measured;
    
    pid->integral += error;
    /* 积分限幅 */
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    /* 输出限幅 */
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    
    return output;
}
```

---

### 4.4 Protocol 协议层

**职责**：
- 通信协议的封装与解析
- 数据帧的组包与拆包
- 协议状态管理

**特点**：
- 只关注数据格式，不关注传输方式
- 可独立测试
- 支持多种协议并存

**典型文件**：
```
Protocol/
├── proto_modbus.c       # Modbus协议
├── proto_can.c          # CAN协议
├── proto_frame.c        # 自定义帧协议
└── proto_json.c         # JSON协议
```

**Protocol vs Service 区别**：
```c
/* Protocol层 - 关注数据格式 */
int Proto_Modbus_Pack(uint8_t *buf, Proto_ModbusFrame_t *frame);
int Proto_Modbus_Parse(uint8_t *data, uint16_t len, Proto_ModbusFrame_t *frame);

/* Service层 - 关注通用机制 */
void Srv_Fsm_Dispatch(Srv_Fsm_t *fsm, uint8_t event);
float Srv_Pid_Compute(Srv_Pid_t *pid, float setpoint, float measured);
```

**示例**：
```c
/* proto_frame.c - 自定义帧协议 */

/*
 * 帧格式：[HEAD][LEN][CMD][DATA...][CRC16]
 *         0xAA  1B   1B   N Bytes  2B
 */

#include "proto_frame.h"
#include "util_crc.h"

int Proto_Frame_Pack(uint8_t *buf, Proto_Frame_t *frame)
{
    uint16_t idx = 0;
    
    buf[idx++] = PROTO_FRAME_HEAD;
    buf[idx++] = frame->data_len + 1;  /* LEN = CMD + DATA */
    buf[idx++] = frame->cmd;
    
    for (uint16_t i = 0; i < frame->data_len; i++) {
        buf[idx++] = frame->data[i];
    }
    
    uint16_t crc = Util_Crc16_Compute(buf, idx);
    buf[idx++] = crc & 0xFF;
    buf[idx++] = (crc >> 8) & 0xFF;
    
    return idx;
}

int Proto_Frame_Parse(uint8_t *buf, uint16_t len, Proto_Frame_t *frame)
{
    if (len < PROTO_FRAME_MIN_LEN) return -1;
    if (buf[0] != PROTO_FRAME_HEAD) return -2;
    
    uint16_t frame_len = buf[1] + 4;  /* HEAD + LEN + DATA + CRC */
    if (len < frame_len) return -3;
    
    /* CRC校验 */
    uint16_t crc_calc = Util_Crc16_Compute(buf, frame_len - 2);
    uint16_t crc_recv = buf[frame_len - 2] | (buf[frame_len - 1] << 8);
    if (crc_calc != crc_recv) return -4;
    
    frame->cmd = buf[2];
    frame->data_len = buf[1] - 1;
    memcpy(frame->data, &buf[3], frame->data_len);
    
    return 0;
}
```

---

### 4.5 Driver 驱动层

**职责**：
- 外设初始化与配置
- 外设操作封装
- 中断处理

**特点**：
- 面向外设，与具体芯片相关
- 封装HAL库，提供简洁接口
- 同一外设不同配置由BSP层决定

**典型文件**：
```
Driver/
├── drv_gpio.c           # GPIO驱动
├── drv_uart.c           # UART驱动
├── drv_spi.c            # SPI驱动
├── drv_i2c.c            # I2C驱动
├── drv_adc.c            # ADC驱动
├── drv_pwm.c            # PWM驱动
├── drv_timer.c          # 定时器驱动
└── drv_encoder.c        # 编码器驱动
```

**示例**：
```c
/* drv_uart.c */

#include "drv_uart.h"
#include "util_fifo.h"

/* 接收FIFO */
static uint8_t s_rx_buf[DRV_UART_RX_BUF_SIZE];
static Util_Fifo_t s_rx_fifo;

/* 回调函数 */
static Drv_UartRxCallback_t s_rx_callback = NULL;

void Drv_Uart_Init(UART_HandleTypeDef *huart)
{
    Util_Fifo_Init(&s_rx_fifo, s_rx_buf, sizeof(s_rx_buf));
    
    /* 使能空闲中断 */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, s_dma_buf, DRV_UART_DMA_BUF_SIZE);
}

int Drv_Uart_Send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    return HAL_UART_Transmit(huart, data, len, 1000);
}

int Drv_Uart_Read(uint8_t *data, uint16_t len)
{
    return Util_Fifo_Read(&s_rx_fifo, data, len);
}

void Drv_Uart_RegisterRxCallback(Drv_UartRxCallback_t callback)
{
    s_rx_callback = callback;
}

/* 空闲中断处理（在stm32f4xx_it.c中调用） */
void Drv_Uart_IdleHandler(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        
        /* 计算接收长度，写入FIFO */
        uint16_t len = DRV_UART_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        Util_Fifo_Write(&s_rx_fifo, s_dma_buf, len);
        
        /* 重启DMA */
        HAL_UART_DMAStop(huart);
        HAL_UART_Receive_DMA(huart, s_dma_buf, DRV_UART_DMA_BUF_SIZE);
        
        /* 通知上层 */
        if (s_rx_callback) {
            s_rx_callback(len);
        }
    }
}
```

---

### 4.6 BSP 板级层

**职责**：
- 板级硬件初始化
- 引脚定义与映射
- 时钟配置
- 硬件资源分配

**特点**：
- 与具体电路板相关
- 更换硬件只需修改此层
- 提供硬件配置表

**典型文件**：
```
BSP/
├── bsp.c                # 板级初始化总入口
├── bsp.h
├── bsp_gpio.c           # GPIO引脚定义
├── bsp_clock.c          # 时钟配置
└── bsp_config.h         # 硬件配置表
```

**示例**：
```c
/* bsp_config.h - 硬件配置表 */

#ifndef __BSP_CONFIG_H__
#define __BSP_CONFIG_H__

/*============================================================================*/
/*                              LED配置                                        */
/*============================================================================*/
#define BSP_LED1_PORT           GPIOB
#define BSP_LED1_PIN            GPIO_PIN_0
#define BSP_LED1_ACTIVE_LOW     1           /* 1=低电平点亮 */

#define BSP_LED2_PORT           GPIOB
#define BSP_LED2_PIN            GPIO_PIN_1
#define BSP_LED2_ACTIVE_LOW     1

/*============================================================================*/
/*                              按键配置                                       */
/*============================================================================*/
#define BSP_KEY1_PORT           GPIOA
#define BSP_KEY1_PIN            GPIO_PIN_0
#define BSP_KEY1_ACTIVE_LOW     1           /* 1=低电平有效 */
#define BSP_KEY1_EXTI_IRQ       EXTI0_IRQn

/*============================================================================*/
/*                              UART配置                                       */
/*============================================================================*/
#define BSP_UART_DEBUG          huart1
#define BSP_UART_COMM           huart2

/*============================================================================*/
/*                              电机配置                                       */
/*============================================================================*/
#define BSP_MOTOR_PWM_TIM       htim1
#define BSP_MOTOR_PWM_CH        TIM_CHANNEL_1
#define BSP_MOTOR_DIR_PORT      GPIOA
#define BSP_MOTOR_DIR_PIN       GPIO_PIN_8

#endif /* __BSP_CONFIG_H__ */
```

```c
/* bsp.c - 板级初始化 */

#include "bsp.h"
#include "bsp_gpio.h"
#include "bsp_clock.h"

void Bsp_Init(void)
{
    /* 时钟初始化（CubeMX已完成） */
    // Bsp_Clock_Init();
    
    /* GPIO初始化 */
    Bsp_Gpio_Init();
    
    /* 其他板级初始化 */
}
```

---

### 4.7 Utils 工具层

**职责**：
- 通用工具函数
- 基础数据结构
- 平台适配

**特点**：
- 与业务无关，与硬件无关
- 所有层都可以调用
- 高度可复用

**典型文件**：
```
Utils/
├── util_log.c           # 日志系统
├── util_fifo.c          # FIFO队列
├── util_crc.c           # CRC校验
├── util_delay.c         # 延时函数
├── util_assert.c        # 断言
└── util_common.h        # 通用宏定义
```

**示例**：
```c
/* util_log.h */

#ifndef __UTIL_LOG_H__
#define __UTIL_LOG_H__

/* 日志级别 */
typedef enum {
    LOG_LEVEL_ERROR = 0,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
} Util_LogLevel_t;

/* 设置日志级别 */
void Util_Log_SetLevel(Util_LogLevel_t level);

/* 日志输出 */
void Util_Log_Print(Util_LogLevel_t level, const char *tag, const char *fmt, ...);

/* 便捷宏 */
#define LOG_E(tag, fmt, ...)  Util_Log_Print(LOG_LEVEL_ERROR, tag, fmt, ##__VA_ARGS__)
#define LOG_W(tag, fmt, ...)  Util_Log_Print(LOG_LEVEL_WARN,  tag, fmt, ##__VA_ARGS__)
#define LOG_I(tag, fmt, ...)  Util_Log_Print(LOG_LEVEL_INFO,  tag, fmt, ##__VA_ARGS__)
#define LOG_D(tag, fmt, ...)  Util_Log_Print(LOG_LEVEL_DEBUG, tag, fmt, ##__VA_ARGS__)

#endif
```

```c
/* util_log.c */

#include "util_log.h"
#include <stdio.h>
#include <stdarg.h>

static Util_LogLevel_t s_log_level = LOG_LEVEL_DEBUG;

static const char *s_level_str[] = {
    [LOG_LEVEL_ERROR] = "E",
    [LOG_LEVEL_WARN]  = "W",
    [LOG_LEVEL_INFO]  = "I",
    [LOG_LEVEL_DEBUG] = "D",
};

void Util_Log_SetLevel(Util_LogLevel_t level)
{
    s_log_level = level;
}

void Util_Log_Print(Util_LogLevel_t level, const char *tag, const char *fmt, ...)
{
    if (level > s_log_level) return;
    
    printf("[%s/%s] ", s_level_str[level], tag);
    
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    
    printf("\r\n");
}
```

---

## 5. 调用规则

### 5.1 依赖关系矩阵

```
调用方 →      App   Device  Service  Protocol  Driver  BSP   Utils
被调用 ↓
─────────────────────────────────────────────────────────────────
App           -      ✗       ✗        ✗         ✗      ✗      ✗
Device        ✓      -       ✗        ✗         ✗      ✗      ✗
Service       ✓      ✓       -        ✗         ✗      ✗      ✗
Protocol      ✓      ✓       ✗        -         ✗      ✗      ✗
Driver        ✗      ✓       ✓        ✓         -      ✗      ✗
BSP           ✗      ✗       ✗       ✗         ✓      -      ✗
Utils         ✓      ✓       ✓        ✓         ✓      ✓      -

✓ = 允许调用
✗ = 禁止调用
- = 同层
```

### 5.2 调用规则说明

1. **上层可以调用下层**
   - App → Device/Service/Protocol → Driver → BSP → HAL

2. **禁止下层调用上层**
   - Driver不能调用Device
   - 如需通知上层，使用回调函数

3. **同层之间谨慎调用**
   - Device之间一般不互相调用
   - Service之间可以调用（如FSM调用Timer）

4. **Utils是特殊层**
   - 所有层都可以调用Utils
   - Utils不调用任何其他层

### 5.3 回调机制

当底层需要通知上层时，使用回调函数：

```c
/* Driver层定义回调类型 */
typedef void (*Drv_UartRxCallback_t)(uint16_t len);

/* Driver层提供注册接口 */
void Drv_Uart_RegisterRxCallback(Drv_UartRxCallback_t callback);

/* App层注册回调 */
void App_Uart_RxHandler(uint16_t len)
{
    /* 处理接收数据 */
}

void App_Init(void)
{
    Drv_Uart_RegisterRxCallback(App_Uart_RxHandler);
}
```

---

## 6. 命名规范

### 6.1 文件命名

```
格式：[层前缀]_[模块名].c/h

app_main.c          # 应用层 - 主程序
app_task_motor.c    # 应用层 - 电机任务

dev_led.c           # 设备层 - LED设备
dev_motor.c         # 设备层 - 电机设备

srv_fsm.c           # 服务层 - 状态机
srv_pid.c           # 服务层 - PID算法

proto_modbus.c      # 协议层 - Modbus协议
proto_frame.c       # 协议层 - 帧协议

drv_uart.c          # 驱动层 - UART驱动
drv_gpio.c          # 驱动层 - GPIO驱动

bsp.c               # BSP层 - 板级初始化
bsp_config.h        # BSP层 - 硬件配置

util_log.c          # 工具层 - 日志
util_fifo.c         # 工具层 - FIFO
```

### 6.2 函数命名

```c
格式：[层前缀]_[模块]_[动作]()

/* App层 */
void App_Task_Motor_Init(void);
void App_Task_Motor_Run(void);

/* Device层 */
void Dev_Led_Init(void);
void Dev_Led_On(uint8_t id);
void Dev_Led_Off(uint8_t id);
void Dev_Motor_SetSpeed(int16_t speed);
int16_t Dev_Motor_GetSpeed(void);

/* Service层 */
void Srv_Fsm_Init(Srv_Fsm_t *fsm);
void Srv_Fsm_Start(Srv_Fsm_t *fsm);
void Srv_Fsm_Dispatch(Srv_Fsm_t *fsm, uint8_t event);
void Srv_Pid_Init(Srv_Pid_t *pid, float kp, float ki, float kd);
float Srv_Pid_Compute(Srv_Pid_t *pid, float setpoint, float measured);

/* Protocol层 */
int Proto_Modbus_Pack(uint8_t *buf, Proto_ModbusFrame_t *frame);
int Proto_Modbus_Parse(uint8_t *data, uint16_t len, Proto_ModbusFrame_t *frame);

/* Driver层 */
void Drv_Uart_Init(UART_HandleTypeDef *huart);
int Drv_Uart_Send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);
int Drv_Uart_Read(uint8_t *data, uint16_t len);

/* BSP层 */
void Bsp_Init(void);
void Bsp_Gpio_Init(void);

/* Utils层 */
void Util_Log_Print(Util_LogLevel_t level, const char *tag, const char *fmt, ...);
int Util_Fifo_Write(Util_Fifo_t *fifo, uint8_t *data, uint16_t len);
uint16_t Util_Crc16_Compute(uint8_t *data, uint16_t len);
```

### 6.3 类型命名

```c
格式：[层前缀]_[模块][描述]_t

/* 结构体 */
typedef struct { ... } Srv_Fsm_t;
typedef struct { ... } Srv_Pid_t;
typedef struct { ... } Dev_MotorCtx_t;
typedef struct { ... } Proto_ModbusFrame_t;
typedef struct { ... } Util_Fifo_t;

/* 枚举 */
typedef enum { ... } Dev_LedId_t;
typedef enum { ... } Srv_FsmEvent_t;
typedef enum { ... } Util_LogLevel_t;

/* 函数指针 */
typedef void (*Drv_UartRxCallback_t)(uint16_t len);
typedef void (*Srv_FsmHandler_t)(Srv_Fsm_t *fsm);
```

### 6.4 宏定义命名

```c
格式：[层前缀]_[模块]_[描述]

/* 配置宏 */
#define BSP_LED1_PORT           GPIOB
#define BSP_LED1_PIN            GPIO_PIN_0
#define DRV_UART_RX_BUF_SIZE    256
#define SRV_FSM_EVENT_QUEUE_SIZE 16

/* 功能宏 */
#define UTIL_MIN(a, b)          ((a) < (b) ? (a) : (b))
#define UTIL_MAX(a, b)          ((a) > (b) ? (a) : (b))
#define UTIL_ARRAY_SIZE(arr)    (sizeof(arr) / sizeof((arr)[0]))
```

### 6.5 变量命名

```c
/* 全局变量：g_前缀 */
Srv_Fsm_t g_motor_fsm;

/* 静态变量：s_前缀 */
static uint8_t s_rx_buffer[256];
static Util_Fifo_t s_rx_fifo;

/* 局部变量：无前缀，小写下划线 */
uint16_t data_len;
uint8_t *buf_ptr;

/* 常量：大写下划线 */
const uint16_t MOTOR_MAX_SPEED = 3000;
```

---

## 7. 目录结构

### 7.1 完整目录树

```
Project/
│
├── App/                        # 应用层
│   ├── app_main.c
│   ├── app_main.h
│   ├── app_task_motor.c
│   ├── app_task_motor.h
│   ├── app_task_comm.c
│   └── app_task_comm.h
│
├── Device/                     # 设备层
│   ├── dev_led.c
│   ├── dev_led.h
│   ├── dev_key.c
│   ├── dev_key.h
│   ├── dev_motor.c
│   └── dev_motor.h
│
├── Service/                    # 服务层
│   ├── Fsm/
│   │   ├── srv_fsm.c
│   │   └── srv_fsm.h
│   ├── Pid/
│   │   ├── srv_pid.c
│   │   └── srv_pid.h
│   └── Timer/
│       ├── srv_soft_timer.c
│       └── srv_soft_timer.h
│
├── Protocol/                   # 协议层
│   ├── proto_modbus.c
│   ├── proto_modbus.h
│   ├── proto_frame.c
│   └── proto_frame.h
│
├── Driver/                     # 驱动层
│   ├── drv_gpio.c
│   ├── drv_gpio.h
│   ├── drv_uart.c
│   ├── drv_uart.h
│   ├── drv_adc.c
│   └── drv_adc.h
│
├── BSP/                        # 板级层
│   ├── bsp.c
│   ├── bsp.h
│   ├── bsp_gpio.c
│   ├── bsp_gpio.h
│   └── bsp_config.h
│
├── Utils/                      # 工具层
│   ├── util_log.c
│   ├── util_log.h
│   ├── util_fifo.c
│   ├── util_fifo.h
│   ├── util_crc.c
│   ├── util_crc.h
│   └── util_common.h
│
├── OS/                         # 操作系统（可选）
│   └── rt-thread/
│
├── Lib/                        # 库
│   └── STM32F4xx_HAL_Driver/
│
├── Core/                       # CubeMX生成
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f4xx_it.h
│   │   └── stm32f4xx_hal_conf.h
│   └── Src/
│       ├── main.c              # 调用App_Main_Init/Run
│       ├── stm32f4xx_it.c
│       └── system_stm32f4xx.c
│
├── Drivers/                    # CMSIS
│   └── CMSIS/
│
├── MDK-ARM/                    # Keil工程
│   └── Project.uvprojx
│
└── Doc/                        # 文档
    ├── 框架设计规范.md
    └── API文档.md
```

### 7.2 头文件包含路径

Keil中添加以下Include Path：

```
../App
../Device
../Service/Fsm
../Service/Pid
../Service/Timer
../Protocol
../Driver
../BSP
../Utils
../Core/Inc
```

---

## 8. 开发流程

### 8.1 新增功能流程

以"添加温度传感器"为例：

```
1. BSP层：定义引脚
   └── bsp_config.h: 添加传感器I2C地址、引脚定义

2. Driver层：实现I2C驱动（如果没有）
   └── drv_i2c.c/h: I2C读写接口

3. Device层：实现传感器设备
   └── dev_sensor_temp.c/h: 温度读取、校准接口

4. App层：使用设备
   └── app_task_monitor.c: 周期读取温度，超限报警
```

### 8.2 移植到新硬件

```
1. 修改BSP层
   └── bsp_config.h: 更新引脚定义
   └── bsp_gpio.c: 更新GPIO初始化

2. 检查Driver层
   └── 如果外设变化，更新对应驱动

3. Device/Service/Protocol/App层
   └── 通常不需要修改
```

### 8.3 代码审查要点

- [ ] 是否遵循分层原则，无跨层调用
- [ ] 命名是否符合规范
- [ ] 是否有适当的注释
- [ ] 是否有错误处理
- [ ] 是否有资源泄漏风险

---

## 9. 示例代码

### 9.1 完整调用链示例

**场景**：按键按下 → 切换LED状态

```c
/*=== App层 ===*/
/* app_main.c */

#include "app_main.h"
#include "dev_led.h"
#include "dev_key.h"
#include "util_log.h"

#define TAG "APP"

/* 按键回调 */
static void App_Key_Handler(uint8_t key_id, uint8_t event)
{
    if (event == DEV_KEY_EVENT_PRESS) {
        LOG_I(TAG, "Key %d pressed", key_id);
        Dev_Led_Toggle(DEV_LED_1);
    }
}

void App_Main_Init(void)
{
    LOG_I(TAG, "System init...");
    
    Dev_Led_Init();
    Dev_Key_Init();
    Dev_Key_RegisterCallback(App_Key_Handler);
    
    LOG_I(TAG, "System ready");
}

void App_Main_Run(void)
{
    Dev_Key_Scan();  /* 按键扫描 */
}


/*=== Device层 ===*/
/* dev_led.c */

#include "dev_led.h"
#include "drv_gpio.h"
#include "bsp_config.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t active_low;
    uint8_t state;
} Dev_LedInfo_t;

static Dev_LedInfo_t s_leds[] = {
    [DEV_LED_1] = {BSP_LED1_PORT, BSP_LED1_PIN, BSP_LED1_ACTIVE_LOW, 0},
    [DEV_LED_2] = {BSP_LED2_PORT, BSP_LED2_PIN, BSP_LED2_ACTIVE_LOW, 0},
};

void Dev_Led_Init(void)
{
    for (int i = 0; i < DEV_LED_MAX; i++) {
        Dev_Led_Off(i);
    }
}

void Dev_Led_On(uint8_t id)
{
    if (id >= DEV_LED_MAX) return;
    
    uint8_t level = s_leds[id].active_low ? 0 : 1;
    Drv_Gpio_Write(s_leds[id].port, s_leds[id].pin, level);
    s_leds[id].state = 1;
}

void Dev_Led_Off(uint8_t id)
{
    if (id >= DEV_LED_MAX) return;
    
    uint8_t level = s_leds[id].active_low ? 1 : 0;
    Drv_Gpio_Write(s_leds[id].port, s_leds[id].pin, level);
    s_leds[id].state = 0;
}

void Dev_Led_Toggle(uint8_t id)
{
    if (id >= DEV_LED_MAX) return;
    
    if (s_leds[id].state) {
        Dev_Led_Off(id);
    } else {
        Dev_Led_On(id);
    }
}


/*=== Driver层 ===*/
/* drv_gpio.c */

#include "drv_gpio.h"

void Drv_Gpio_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t state)
{
    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t Drv_Gpio_Read(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin);
}


/*=== main.c (CubeMX生成，少量修改) ===*/

#include "main.h"
#include "app_main.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    
    /* USER CODE BEGIN 2 */
    App_Main_Init();
    /* USER CODE END 2 */
    
    while (1)
    {
        /* USER CODE BEGIN 3 */
        App_Main_Run();
        /* USER CODE END 3 */
    }
}
```

---

## 10. 常见问题

### Q1: Driver层和Device层的区别？

**Driver层**：面向"外设"，封装硬件操作，不包含业务逻辑。
- `Drv_Gpio_Write(GPIOB, GPIO_PIN_0, 1)`

**Device层**：面向"设备"，提供业务语义，屏蔽硬件细节。
- `Dev_Led_On(DEV_LED_1)`

### Q2: Service层和Protocol层的区别？

**Service层**：通用机制、框架、算法，与具体协议无关。
- 状态机、PID、软件定时器、滤波器

**Protocol层**：特定通信协议的封装与解析。
- Modbus、CAN协议、自定义帧

### Q3: Utils层可以调用Driver层吗？

**不可以**。Utils是最底层，不依赖任何其他层（除标准C库）。

如果工具需要硬件支持（如延时），应通过**函数指针注入**：
```c
/* 初始化时注入 */
Util_Delay_Init(HAL_Delay);
```

### Q4: 如何处理中断？

中断服务函数放在Driver层，通过回调通知上层：
```c
/* Driver层 */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Drv_Key_IrqHandler(GPIO_Pin);  /* Driver层处理 */
}

/* Device层注册回调 */
Drv_Key_RegisterCallback(Dev_Key_Handler);
```

### Q5: CubeMX生成的代码怎么处理？

- **Core/Src/main.c**：只在USER CODE区域添加代码
- **其他文件**：尽量不修改，便于重新生成
- **硬件初始化**：使用CubeMX配置，Bsp层只做补充

---

## 附录

### A. 检查清单

新模块开发检查清单：

- [ ] 文件放在正确的层级目录
- [ ] 文件名符合 `[层前缀]_[模块名].c/h` 格式
- [ ] 函数名符合 `[层前缀]_[模块]_[动作]()` 格式
- [ ] 类型名符合 `[层前缀]_[模块][描述]_t` 格式
- [ ] 无跨层调用（除Utils）
- [ ] 头文件有防重复包含
- [ ] 关键函数有注释说明
- [ ] 对外接口声明为extern，内部函数声明为static

### B. 版本历史

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| V1.0 | 2025-12-18 | 初始版本 |

### C. 架构模块完成情况
```
Project/
│
├── App/                        # 应用层
│   └── 项目相关，通常不可复用, 需用户自行实现
│
├── Device/                     # 设备层
│   └── 待完善
│
├── Service/                    # 服务层
│   └── 待完善
│
├── Protocol/                   # 协议层
│   └── 需按照协议实现
│
├── Driver/                     # 驱动层
│   └── 待完善
│
├── BSP/                        # 板级层
│   └── 待完善
│
├── Utils/                      # 工具层
│   ├── util_log.c
│   └── util_log.h
│
├── OS/                         # 操作系统（可选）
│   └── rt-thread/
│
├── Lib/                        # 库
│   └── STM32F4xx_HAL_Driver/
│
├── Core/                       # CubeMX生成
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f4xx_it.h
│   │   └── stm32f4xx_hal_conf.h
│   └── Src/
│       ├── main.c              # 调用App_Main_Init/Run
│       ├── stm32f4xx_it.c
│       └── system_stm32f4xx.c
│
├── Drivers/                    # CMSIS
│   └── CMSIS/
│
├── MDK-ARM/                    # Keil工程
│   └── Project.uvprojx
│
└── Doc/                        # 文档
    ├── 框架设计规范.md
    └── API文档.md
```

---
**文档结束**