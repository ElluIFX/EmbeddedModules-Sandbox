/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define LOG_MODULE "main"
#include "log.h"

#define FLASH_HSPI_HANDLE hspi2
#include "spif_port_lfs.h"

//

#include "TimeLib.h"
#include "cm_backtrace.h"
#include "embedded_cli.h"
#include "key.h"
#include "klite.h"
#include "led.h"
#include "lfs.h"
#include "lfs_stdlib.h"
#include "lfs_utils.h"
#include "ll_i2c.h"
#include "lwprintf.h"
#include "mf.h"
#include "scheduler.h"
#include "sds.h"
#include "spif.h"
#include "sys_utils.h"
#include "xv_extend.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Initialize(void);
static void MPU_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
key_read_e read_key(uint8_t idx) {
    switch (idx) {
        case 0:
            return (HAL_GPIO_ReadPin(KEY_A_GPIO_Port, KEY_A_Pin) ==
                    GPIO_PIN_SET);
        case 1:
            return (HAL_GPIO_ReadPin(KEY_B_GPIO_Port, KEY_B_Pin) ==
                    GPIO_PIN_RESET);
    }
    return KEY_READ_UP;
}

void key_callback(uint8_t idx, uint8_t event) {
    LOG_TRACE("Key %d event %s - %d", idx, key_get_event_name(event),
              key_get_multi_event_num(event));
}

void task_alive(void* arg) {
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}

EmbeddedCli* cli = NULL;

void uart_callback(uint8_t* buf, size_t len) {
    static bool cli_enabled = false;
    if (cli == NULL)
        return;
    if (!cli_enabled) {
        cli_enabled = true;
        sch_task_set_enabled("cli", true);
    }
    embeddedCliReceiveBuffer(cli, (const char*)buf, len);
}

void init_all(void) {
    init_module_timebase();
    LOG_CUSTOM("INIT", T_LMAGENTA, "System Initializing...");

    sch_task_create("alive", task_alive, 2, 1, 3, NULL);

    // uart_fifo_tx_init(&huart1, NULL, 2048);
    uart_dma_rx_init(&huart1, NULL, 2048, uart_callback, 1);
    LOG_PASS("UART Initialized");

    sch_task_create("keytick", (sch_task_func_t)key_tick, 100, 1, 2,
                    key_init(NULL, read_key, 2, key_callback));
    LOG_PASS("Key Initialized");

    spif_init_lfs();
    LOG_PASS("File System Initialized");

    mf_init();
    LOG_INFO("MiniFlashDB Initialized");

    cli = embeddedCliNewDefault();
    sch_add_command_to_cli(cli);
    system_utils_add_command_to_cli(cli);
    lfs_utils_add_command_to_cli(cli, &lfs);
    xv_ex_add_command_to_cli(cli);
    sch_task_create("cli", (sch_task_func_t)embeddedCliProcess, 100, 0, 1, cli);
    LOG_PASS("CLI Initialized");

    LOG_PASS("System Initialized");
}

void test_thread(void* arg) {
    typedef struct {
        uint32_t boot;
    } data_t;

    data_t data = {0};

    LOG_INFO("Test thread started");
    mf_sync_key("data", &data, sizeof(data_t));
    LOG_INFO("Boot count: %d", data.boot++);
    mf_set_key("data", &data, sizeof(data_t));
    mf_save();
}

void main_thread(void* arg) {
    init_all();

    kl_thread_create(test_thread, 0, 10240, 0);

    scheduler_run(1);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */
    cm_backtrace_init("H7B0_Test", "1.0.0", "1.0.0");
    // SCB_EnableICache();
    // SCB_EnableDCache();
    SCB->CACR |= 1 << 2;  // Enable DCache Write-through

    /* USER CODE END 1 */

    /* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();

    /* Enable D-Cache---------------------------------------------------------*/
    // SCB_EnableDCache();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
    HAL_Init();

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_SPI2_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */

    static uint8_t __ALIGNED(32) heap[256UL * 1024UL] = {0};

    kl_kernel_init(heap, sizeof(heap));            /* 系统初始化 */
    kl_thread_create(main_thread, NULL, 10240, 5); /* 创建main线程 */
    kl_kernel_start();                             /* 启动系统 */

    // init_module_heap(heap, sizeof(heap));
    // init_all();
    // scheduler_run(1);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /*AXI clock gating */
    RCC->CKGAENR = 0xFFFFFFFF;

    /** Supply configuration update enable
   */
    HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY);

    /** Configure the main internal regulator output voltage
   */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 112;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
   */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                  RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
   */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x90000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
   */
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.BaseAddress = 0x20000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_2MB;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
   */
    MPU_InitStruct.Number = MPU_REGION_NUMBER2;
    MPU_InitStruct.BaseAddress = 0x24000000;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
