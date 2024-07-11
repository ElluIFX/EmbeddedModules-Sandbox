#include "scheduler.h"
#include "ulist.h"

#define _CR_STATE_READY 0     // 就绪态
#define _CR_STATE_RUNNING 1   // 运行态
#define _CR_STATE_CRING 2     // 等待态
#define _CR_STATE_SLEEPING 3  // 睡眠态
#define _CR_STATE_STOPPED 4   // 停止态

typedef struct {  // 协程句柄结构
    long ptr;     // 协程跳入地址
    void* local;  // 协程局部变量储存区
} __cortn_data_t;

typedef struct {           // 协程句柄结构
    uint8_t state;         // 协程状态
    ulist_t dataList;      // 协程数据列表(__cortn_data_t)
    __cortn_data_t* data;  // 协程数据指针(指向当前深度)
    uint8_t runDepth;      // 协程当前运行深度
    uint8_t actDepth;      // 协程实际执行深度
    uint64_t sleepUntil;   // 等待态结束时间(us), 0表示暂停
    void* msg;             // 协程消息指针
    const char* name;      // 协程名
} __cortn_handle_t;

// 试图报个用户友好的错误
// 协程句柄兼参数检查
#define __chd__ You_forgot___async___in_the_argument
// 初始化检查
#define __async_check__ You_forgot_CR_INIT_in_top_of_function
// 拒绝多次初始化
#define __crap__ Do_not_init_coroutine_more_than_once

#define __async__ __cortn_handle_t* __chd__

extern const char* __cortn_internal_get_name(void);
extern void* __cortn_internal_init_local(size_t size);
extern uint8_t __cortn_internal_await_enter(void);
extern uint8_t __cortn_internal_await_return(void);
extern void __cortn_internal_delay(uint64_t delay_us);
extern uint8_t __cortn_internal_acq_mutex(const char* name);
extern void __cortn_internal_rel_mutex(const char* name);
extern uint8_t __cortn_internal_await_bar(const char* name);
extern void __cortn_internal_await_msg(__async__, void** msgPtr);

#define __CR_INIT                                               \
    __crap__ :;                                                 \
    void* __async_check__ = &&__crap__;                         \
    do {                                                        \
        if ((__chd__->data[__chd__->runDepth].ptr) != 0)        \
            goto*(void*)(__chd__->data[__chd__->runDepth].ptr); \
    } while (0);

#define __CR_INIT_LOCAL_BEGIN struct _cr_local {
#define __CR_INIT_LOCAL_END                                               \
    }                                                                     \
    *_cr_local_p = __cortn_internal_init_local(sizeof(struct _cr_local)); \
    if (_cr_local_p == NULL)                                              \
        return;                                                           \
    __CR_INIT

#define __CR_LOCAL(var) (_cr_local_p->var)

#define __CR_YIELD()                                          \
    do {                                                      \
        __label__ l;                                          \
        (__chd__->data[__chd__->runDepth].ptr) = (long) && l; \
        return;                                               \
    l:;                                                       \
        (__chd__->data[__chd__->runDepth].ptr) = 0;           \
        __async_check__ = __async_check__;                    \
    } while (0)

#define __CR_AWAIT(func_cmd, args...)                         \
    do {                                                      \
        __label__ l;                                          \
        (__chd__->data[__chd__->runDepth].ptr) = (long) && l; \
    l:;                                                       \
        if (__cortn_internal_await_enter()) {                 \
            func_cmd(__chd__, ##args);                        \
            if (!__cortn_internal_await_return())             \
                return;                                       \
            (__chd__->data[__chd__->runDepth].ptr) = 0;       \
        }                                                     \
        __async_check__ = __async_check__;                    \
    } while (0)

#define __CR_DELAY_US(us)                    \
    do {                                     \
        __cortn_internal_delay(us);          \
        __chd__->state = _CR_STATE_SLEEPING; \
        __CR_YIELD();                        \
    } while (0)

#define __CR_YIELD_UNTIL(cond) \
    do {                       \
        __CR_YIELD();          \
    } while (!(cond))

#define __CR_DELAY_UNTIL(cond, delay_ms)  \
    do {                                  \
        __CR_DELAY_US((delay_ms) * 1000); \
    } while (!(cond))

#define __CR_ACQUIRE_MUTEX(mutex_name)                 \
    do {                                               \
        if (!__cortn_internal_acq_mutex(mutex_name)) { \
            __chd__->state = _CR_STATE_CRING;          \
            __CR_YIELD();                              \
        }                                              \
    } while (0)
