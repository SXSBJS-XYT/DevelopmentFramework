/**
 * @file    srv_fsm.h
 * @brief   Service层 - 有限状态机框架
 * 
 * 功能特性:
 * =========
 * - 状态表驱动，结构清晰
 * - 支持 Entry / Exit / Run 回调
 * - 支持转移动作(Action)和守卫条件(Guard)
 * - 内置状态超时处理
 * - 事件队列，中断安全
 * - 调试日志
 * - 纯静态内存，无malloc
 * 
 * 使用流程:
 * =========
 * 1. 定义状态枚举、事件枚举
 * 2. 定义状态表 (Srv_FsmState_t 数组)
 * 3. 定义转移表 (Srv_FsmTrans_t 数组)
 * 4. 创建状态机实例 (Srv_Fsm_t)
 * 5. Srv_Fsm_Init() 初始化
 * 6. Srv_Fsm_Start() 启动
 * 7. 主循环调用 Srv_Fsm_Process()
 * 8. 中断或其他地方调用 Srv_Fsm_PostEvent()
 */

#ifndef __SRV_FSM_H__
#define __SRV_FSM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "util_common.h"
#include "util_platform.h"

/*============================================================================*/
/*                              配置项                                         */
/*============================================================================*/

/* 事件队列大小（必须是2的幂） */
#define SRV_FSM_EVENT_QUEUE_SIZE    8

/* 调试日志开关 */
#define SRV_FSM_DEBUG_ENABLE        1

/*============================================================================*/
/*                              类型定义                                       */
/*============================================================================*/

/* 前向声明 */
typedef struct Srv_Fsm Srv_Fsm_t;

/**
 * @brief  状态回调函数类型
 */
typedef void (*Srv_FsmStateHandler_t)(Srv_Fsm_t *fsm);

/**
 * @brief  转移动作函数类型
 */
typedef void (*Srv_FsmActionHandler_t)(Srv_Fsm_t *fsm, uint8_t event);

/**
 * @brief  守卫条件函数类型
 * @return true=允许转移, false=阻止转移
 */
typedef bool (*Srv_FsmGuardHandler_t)(Srv_Fsm_t *fsm);

/**
 * @brief  状态定义
 */
typedef struct {
    const char *name;               /* 状态名称 */
    Srv_FsmStateHandler_t on_entry; /* 进入回调 */
    Srv_FsmStateHandler_t on_exit;  /* 退出回调 */
    Srv_FsmStateHandler_t on_run;   /* 运行回调 */
    uint32_t timeout_ms;            /* 超时时间(0=不超时) */
    uint8_t timeout_event;          /* 超时事件 */
} Srv_FsmState_t;

/**
 * @brief  转移定义
 */
typedef struct {
    uint8_t event;                  /* 触发事件 */
    uint8_t src_state;              /* 源状态 */
    uint8_t dst_state;              /* 目标状态 */
    Srv_FsmActionHandler_t action;  /* 转移动作 */
    Srv_FsmGuardHandler_t guard;    /* 守卫条件 */
} Srv_FsmTrans_t;

/**
 * @brief  事件队列
 */
typedef struct {
    uint8_t buffer[SRV_FSM_EVENT_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} Srv_FsmEventQueue_t;

/**
 * @brief  状态机实例
 */
struct Srv_Fsm {
    /* 配置 */
    const char *name;               /* 状态机名称 */
    const Srv_FsmState_t *states;   /* 状态表 */
    const Srv_FsmTrans_t *trans;    /* 转移表 */
    uint8_t state_cnt;              /* 状态数量 */
    uint8_t trans_cnt;              /* 转移数量 */
    uint8_t init_state;             /* 初始状态 */
    
    /* 运行时 */
    uint8_t current;                /* 当前状态 */
    uint8_t previous;               /* 上一状态 */
    uint32_t state_start_tick;      /* 进入状态的时间 */
    bool running;                   /* 运行标志 */
    
    /* 事件队列 */
    Srv_FsmEventQueue_t queue;
    
    /* 用户数据 */
    void *user_data;
};

/*============================================================================*/
/*                              特殊值                                         */
/*============================================================================*/

#define SRV_FSM_ANY_STATE       0xFF    /* 任意源状态 */
#define SRV_FSM_STATE_SELF      0xFE    /* 保持当前状态 */

/*============================================================================*/
/*                              API                                            */
/*============================================================================*/

/**
 * @brief  初始化状态机
 */
void Srv_Fsm_Init(Srv_Fsm_t *fsm);

/**
 * @brief  启动状态机
 */
void Srv_Fsm_Start(Srv_Fsm_t *fsm);

/**
 * @brief  停止状态机
 */
void Srv_Fsm_Stop(Srv_Fsm_t *fsm);

/**
 * @brief  同步派发事件（立即处理）
 * @return true=已处理, false=无匹配转移
 */
bool Srv_Fsm_Dispatch(Srv_Fsm_t *fsm, uint8_t event);

/**
 * @brief  异步投递事件（中断安全）
 * @return true=成功, false=队列满
 */
bool Srv_Fsm_PostEvent(Srv_Fsm_t *fsm, uint8_t event);

/**
 * @brief  主循环处理
 */
void Srv_Fsm_Process(Srv_Fsm_t *fsm);

/**
 * @brief  获取当前状态
 */
uint8_t Srv_Fsm_GetState(Srv_Fsm_t *fsm);

/**
 * @brief  获取当前状态名称
 */
const char *Srv_Fsm_GetStateName(Srv_Fsm_t *fsm);

/**
 * @brief  获取状态持续时间(ms)
 */
uint32_t Srv_Fsm_GetStateTime(Srv_Fsm_t *fsm);

/**
 * @brief  判断是否运行中
 */
bool Srv_Fsm_IsRunning(Srv_Fsm_t *fsm);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_FSM_H__ */
