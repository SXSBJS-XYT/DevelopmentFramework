/**
 * @file    srv_fsm.c
 * @brief   Service层 - 有限状态机框架实现
 */

#include "srv_fsm.h"

/*============================================================================*/
/*                              调试日志                                       */
/*============================================================================*/

#if SRV_FSM_DEBUG_ENABLE
    #define FSM_LOG(fmt, ...)   UTIL_LOG(fmt, ##__VA_ARGS__)
#else
    #define FSM_LOG(fmt, ...)   ((void)0)
#endif

/*============================================================================*/
/*                              事件队列                                       */
/*============================================================================*/

static void Queue_Init(Srv_FsmEventQueue_t *q)
{
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

static bool Queue_Push(Srv_FsmEventQueue_t *q, uint8_t event)
{
    UTIL_ENTER_CRITICAL();
    
    if (q->count >= SRV_FSM_EVENT_QUEUE_SIZE) {
        UTIL_EXIT_CRITICAL();
        return false;
    }
    
    q->buffer[q->tail] = event;
    q->tail = (q->tail + 1) & (SRV_FSM_EVENT_QUEUE_SIZE - 1);
    q->count++;
    
    UTIL_EXIT_CRITICAL();
    return true;
}

static bool Queue_Pop(Srv_FsmEventQueue_t *q, uint8_t *event)
{
    UTIL_ENTER_CRITICAL();
    
    if (q->count == 0) {
        UTIL_EXIT_CRITICAL();
        return false;
    }
    
    *event = q->buffer[q->head];
    q->head = (q->head + 1) & (SRV_FSM_EVENT_QUEUE_SIZE - 1);
    q->count--;
    
    UTIL_EXIT_CRITICAL();
    return true;
}

/*============================================================================*/
/*                              内部函数                                       */
/*============================================================================*/

/**
 * @brief  查找匹配的转移
 */
static const Srv_FsmTrans_t *FindTrans(Srv_Fsm_t *fsm, uint8_t event)
{
    for (uint8_t i = 0; i < fsm->trans_cnt; i++) {
        const Srv_FsmTrans_t *t = &fsm->trans[i];
        
        /* 事件匹配 */
        if (t->event != event) continue;
        
        /* 源状态匹配 */
        if (t->src_state != SRV_FSM_ANY_STATE && t->src_state != fsm->current) continue;
        
        /* 守卫条件 */
        if (t->guard != NULL && !t->guard(fsm)) continue;
        
        return t;
    }
    return NULL;
}

/**
 * @brief  执行状态转移
 */
static void DoTransition(Srv_Fsm_t *fsm, const Srv_FsmTrans_t *trans, uint8_t event)
{
    uint8_t src = fsm->current;
    uint8_t dst = trans->dst_state;
    
    /* 处理自转移 */
    if (dst == SRV_FSM_STATE_SELF) {
        dst = src;
    }
    
    const Srv_FsmState_t *src_state = &fsm->states[src];
    const Srv_FsmState_t *dst_state = &fsm->states[dst];
    
    /* 日志 */
    FSM_LOG("[FSM] %s: %s -> %s (evt=%d)\r\n",
            fsm->name,
            src_state->name ? src_state->name : "?",
            dst_state->name ? dst_state->name : "?",
            event);
    
    /* 真正的状态切换 */
    if (src != dst) {
        /* Exit */
        if (src_state->on_exit != NULL) {
            src_state->on_exit(fsm);
        }
        
        /* Action */
        if (trans->action != NULL) {
            trans->action(fsm, event);
        }
        
        /* 更新状态 */
        fsm->previous = src;
        fsm->current = dst;
        fsm->state_start_tick = UTIL_GET_TICK();
        
        /* Entry */
        if (dst_state->on_entry != NULL) {
            dst_state->on_entry(fsm);
        }
    } else {
        /* 自转移：只执行Action */
        if (trans->action != NULL) {
            trans->action(fsm, event);
        }
    }
}

/**
 * @brief  检查超时
 */
static void CheckTimeout(Srv_Fsm_t *fsm)
{
    const Srv_FsmState_t *state = &fsm->states[fsm->current];
    
    if (state->timeout_ms == 0) return;
    
    uint32_t elapsed = UTIL_GET_TICK() - fsm->state_start_tick;
    
    if (elapsed >= state->timeout_ms) {
        FSM_LOG("[FSM] %s: Timeout in %s (%lu ms)\r\n",
                fsm->name,
                state->name ? state->name : "?",
                elapsed);
        
        Srv_Fsm_Dispatch(fsm, state->timeout_event);
    }
}

/*============================================================================*/
/*                              公共API                                        */
/*============================================================================*/

void Srv_Fsm_Init(Srv_Fsm_t *fsm)
{
    if (fsm == NULL) return;
    
    fsm->current = fsm->init_state;
    fsm->previous = fsm->init_state;
    fsm->state_start_tick = 0;
    fsm->running = false;
    
    Queue_Init(&fsm->queue);
    
    FSM_LOG("[FSM] %s: Init (states=%d, trans=%d)\r\n",
            fsm->name, fsm->state_cnt, fsm->trans_cnt);
}

void Srv_Fsm_Start(Srv_Fsm_t *fsm)
{
    if (fsm == NULL || fsm->running) return;
    
    fsm->running = true;
    fsm->current = fsm->init_state;
    fsm->previous = fsm->init_state;
    fsm->state_start_tick = UTIL_GET_TICK();
    
    const Srv_FsmState_t *init = &fsm->states[fsm->init_state];
    
    FSM_LOG("[FSM] %s: Start -> %s\r\n",
            fsm->name, init->name ? init->name : "?");
    
    /* Entry */
    if (init->on_entry != NULL) {
        init->on_entry(fsm);
    }
}

void Srv_Fsm_Stop(Srv_Fsm_t *fsm)
{
    if (fsm == NULL || !fsm->running) return;
    
    const Srv_FsmState_t *curr = &fsm->states[fsm->current];
    
    /* Exit */
    if (curr->on_exit != NULL) {
        curr->on_exit(fsm);
    }
    
    fsm->running = false;
    Queue_Init(&fsm->queue);
    
    FSM_LOG("[FSM] %s: Stop\r\n", fsm->name);
}

bool Srv_Fsm_Dispatch(Srv_Fsm_t *fsm, uint8_t event)
{
    if (fsm == NULL || !fsm->running) return false;
    
    const Srv_FsmTrans_t *trans = FindTrans(fsm, event);
    
    if (trans != NULL) {
        DoTransition(fsm, trans, event);
        return true;
    }
    
    return false;
}

bool Srv_Fsm_PostEvent(Srv_Fsm_t *fsm, uint8_t event)
{
    if (fsm == NULL) return false;
    return Queue_Push(&fsm->queue, event);
}

void Srv_Fsm_Process(Srv_Fsm_t *fsm)
{
    if (fsm == NULL || !fsm->running) return;
    
    /* 1. 处理事件队列 */
    uint8_t event;
    while (Queue_Pop(&fsm->queue, &event)) {
        Srv_Fsm_Dispatch(fsm, event);
    }
    
    /* 2. 执行状态的Run */
    const Srv_FsmState_t *curr = &fsm->states[fsm->current];
    if (curr->on_run != NULL) {
        curr->on_run(fsm);
    }
    
    /* 3. 检查超时 */
    CheckTimeout(fsm);
}

uint8_t Srv_Fsm_GetState(Srv_Fsm_t *fsm)
{
    if (fsm == NULL) return 0;
    return fsm->current;
}

const char *Srv_Fsm_GetStateName(Srv_Fsm_t *fsm)
{
    if (fsm == NULL) return "NULL";
    return fsm->states[fsm->current].name;
}

uint32_t Srv_Fsm_GetStateTime(Srv_Fsm_t *fsm)
{
    if (fsm == NULL) return 0;
    return UTIL_GET_TICK() - fsm->state_start_tick;
}

bool Srv_Fsm_IsRunning(Srv_Fsm_t *fsm)
{
    if (fsm == NULL) return false;
    return fsm->running;
}
