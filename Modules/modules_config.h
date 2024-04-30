#ifndef _MODULES_CONFIG_H_
#define _MODULES_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MOD_CFG_COMPILER_AC6 1
#define MOD_CFG_CPU_CM7 1
#define MOD_CFG_USE_OS_KLITE 1
#define MOD_CFG_TIME_MATHOD_PERF_COUNTER 1
#define MOD_CFG_HEAP_MATHOD_KLITE 1
#define MOD_CFG_DELAY_MATHOD_KLITE 1
#define MOD_CFG_ENABLE_ATOMIC 1
#define MOD_ENABLE_XYMODEM 1
#define MOD_ENABLE_JSON 1
#define MOD_ENABLE_LFBB 1
#define MOD_ENABLE_LFIFO 1
#define MOD_ENABLE_SDS 1
#define MOD_ENABLE_STRUCT2JSON 1
#define MOD_ENABLE_UDICT 1
#define MOD_ENABLE_ULIST 1
#define MOD_ENABLE_UTHASH 1
#define MOD_ENABLE_CM_BACKTRACE 1
#define CMB_USING_OS_PLATFORM 1
#define CMB_OS_PLATFORM_USE_KLITE 1
#define CMB_CPU_USE_ARM_CORTEX_M7 1
#define CMB_PRINT_USE_ENGLISH 1
#define MOD_ENABLE_LOG 1
#define LOG_CFG_ENABLE 1
#define LOG_CFG_ENABLE_TIMESTAMP 1
#define LOG_CFG_ENABLE_COLOR 1
#define LOG_CFG_ENABLE_MODULE_NAME 1
#define LOG_CFG_LEVEL_USE_PASS 1
#define LOG_CFG_R_COLOR T_BLUE
#define LOG_CFG_D_COLOR T_CYAN
#define LOG_CFG_P_COLOR T_LGREEN
#define LOG_CFG_I_COLOR T_GREEN
#define LOG_CFG_W_COLOR T_YELLOW
#define LOG_CFG_E_COLOR T_RED
#define LOG_CFG_F_COLOR T_MAGENTA
#define LOG_CFG_A_COLOR T_RED
#define LOG_CFG_T_COLOR T_YELLOW
#define LOG_CFG_R_STR "TRACE"
#define LOG_CFG_D_STR "DEBUG"
#define LOG_CFG_P_STR "PASS"
#define LOG_CFG_I_STR "INFO"
#define LOG_CFG_W_STR "WARN"
#define LOG_CFG_E_STR "ERROR"
#define LOG_CFG_F_STR "FATAL"
#define LOG_CFG_A_STR "ASSERT"
#define LOG_CFG_T_STR "TIMEIT"
#define LOG_CFG_PRINTF printf
#define LOG_CFG_TIMESTAMP_FMT "%.3fs"
#define LOG_CFG_TIMESTAMP_FUNC ((float)((uint64_t)m_time_ms()) / 1000)
#define LOG_CFG_PREFIX ""
#define LOG_CFG_SUFFIX ""
#define LOG_CFG_NEWLINE "\r\n"
#define MOD_ENABLE_KEY 1
#define MOD_ENABLE_SPIF 1
#define MOD_ENABLE_BOARD_LED 1
#define MOD_ENABLE_EE 1
#define MOD_ENABLE_LL_I2C 1
#define LL_IIC_CFG_CONVERT_7BIT_ADDR 1
#define LL_IIC_CFG_USE_IT 1
#define LL_I2C_CFG_SEM_TIMEOUT_MS 1000
#define MOD_ENABLE_UNI_IO 1
#define UIO_CFG_ENABLE_UART 1
#define UIO_CFG_UART_ENABLE_DMA_RX 1
#define UIO_CFG_UART_ENABLE_FIFO_TX 1
#define UIO_CFG_UART_TX_TIMEOUT 5
#define UIO_CFG_UART_TX_USE_DMA 1
#define UIO_CFG_UART_TX_USE_IT 1
#define UIO_CFG_UART_REWRITE_HANLDER 1
#define UIO_CFG_PRINTF_REDIRECT 1
#define UIO_CFG_PRINTF_USE_UART 1
#define UIO_CFG_PRINTF_UART_PORT huart1
#define UIO_CFG_PRINTF_REDIRECT_PUTX 1
#define UIO_CFG_PRINTF_ENDL "\r\n"
#define MOD_ENABLE_LITTLEFS 1
#define MOD_ENABLE_MINIFLASHDB 1
#define MOD_ENABLE_KLITE 1
#define KLITE_CFG_FREQ 100000
#define KLITE_CFG_MAX_PRIO 7
#define KLITE_CFG_DEFAULT_PRIO 4
#define KLITE_CFG_HEAP_USE_BUILTIN 1
#define KLITE_CFG_HEAP_USE_BESTFIT 1
#define KLITE_CFG_HEAP_STORAGE_PREV_NODE 1
#define KLITE_CFG_HEAP_TRACE_OWNER 1
#define KLITE_CFG_HEAP_ALIGN_BYTE 4
#define KLITE_CFG_DEFAULT_STACK_SIZE 1024
#define KLITE_CFG_IDLE_THREAD_STACK_SIZE 256
#define KLITE_CFG_STACK_OVERFLOW_DETECT 1
#define KLITE_CFG_STACKOF_BEHAVIOR_CALLBACK 1
#define KLITE_CFG_STACKOF_DETECT_ON_TASK_SWITCH 1
#define KLITE_CFG_STACKOF_SIZE 8
#define KLITE_CFG_WAIT_LIST_ORDER_BY_PRIO 1
#define KLITE_CFG_64BIT_TICK 1
#define KLITE_CFG_OPT_ENABLE 1
#define KLITE_CFG_OPT_MUTEX 1
#define KLITE_CFG_OPT_SEM 1
#define KLITE_CFG_OPT_COND 1
#define KLITE_CFG_OPT_BARRIER 1
#define KLITE_CFG_OPT_RWLOCK 1
#define KLITE_CFG_OPT_EVENT 1
#define KLITE_CFG_OPT_EVENT_FLAGS 1
#define KLITE_CFG_OPT_MAILBOX 1
#define KLITE_CFG_OPT_MPOOL 1
#define KLITE_CFG_OPT_MQUEUE 1
#define KLITE_CFG_OPT_TIMER 1
#define KLITE_CFG_OPT_THREAD_POOL 1
#define MOD_ENABLE_SCHEDULER 1
#define SCH_CFG_ENABLE_TASK 1
#define SCH_CFG_COMP_RANGE_US 1000
#define SCH_CFG_PRI_ORDER_ASC 1
#define SCH_CFG_ENABLE_EVENT 1
#define SCH_CFG_ENABLE_COROUTINE 1
#define SCH_CFG_ENABLE_CALLLATER 1
#define SCH_CFG_CALLLATER_MAX_ARG 12
#define SCH_CFG_ENABLE_SOFTINT 1
#define SCH_CFG_STATIC_NAME 1
#define SCH_CFG_STATIC_NAME_LEN 16
#define SCH_CFG_ENABLE_TERMINAL 1
#define MOD_ENABLE_EMBEDDED_CLI 1
#define MOD_ENABLE_LWPRINTF 1
#define LWPRINTF_CFG_SUPPORT_LONG_LONG 1
#define LWPRINTF_CFG_SUPPORT_TYPE_INT 1
#define LWPRINTF_CFG_SUPPORT_TYPE_POINTER 1
#define LWPRINTF_CFG_SUPPORT_TYPE_FLOAT 1
#define LWPRINTF_CFG_SUPPORT_TYPE_ENGINEERING 1
#define LWPRINTF_CFG_SUPPORT_TYPE_STRING 1
#define LWPRINTF_CFG_SUPPORT_TYPE_BYTE_ARRAY 1
#define LWPRINTF_CFG_FLOAT_DEFAULT_PRECISION 6
#define LWPRINTF_CFG_ENABLE_SHORTNAMES 1
#define MOD_ENABLE_MACRO 1
#define MOD_ENABLE_PERF_COUNTER 1
#define MOD_ENABLE_RYU 1
#define MOD_ENABLE_TERM_TABLE 1
#define MOD_ENABLE_TIMELIB 1
#define MOD_ENABLE_XV 1

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULES_CONFIG_H_ */