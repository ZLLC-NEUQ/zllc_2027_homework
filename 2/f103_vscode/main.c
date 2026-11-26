/**
 * @file    main.c
 * @brief   STM32F103C8T6 + GM6020 最小示例（VS Code / EIDE 工程）
 *
 * 功能：
 *   1. 手写配置 72 MHz 时钟（HSE 8 MHz × 9，APB1 = 36 MHz）；
 *   2. 调用 CAN_BSP_Init() 初始化 CAN1（1 Mbps，PA11/PA12，FIFO0 中断接收）；
 *   3. 每 2 ms 发送一次 GM6020 电压控制帧；
 *   4. 电机反馈由中断自动解析进 g_gm6020[] 结构体。
 *
 * 说明：
 *   - 本文件不使用 CubeMX 生成的代码；
 *   - 若开发板没有 8 MHz 外部晶振，需要改写 SystemClock_Config()；
 *   - 电机 ID 默认 1，电压从 ±3000 左右开始试，确认方向后再加大。
 */

#include "stm32f1xx_hal.h"
#include "can_bsp.h"
#include "gm6020.h"

static void SystemClock_Config(void);

int main(void)
{
    uint32_t last_tx;

    HAL_Init();

    /* 72 MHz：HSE 8 MHz × 9 = 72 MHz，APB1 = 36 MHz（CAN 时钟） */
    SystemClock_Config();

    /* CAN1：GPIO、过滤器、NVIC、启动、接收中断一次配好 */
    if (CAN_BSP_Init() != HAL_OK)
    {
        Error_Handler();
    }

    /* 清空电机结构体，并注册反馈解析回调 */
    GM6020_Init();

    last_tx = HAL_GetTick();

    while (1)
    {
        /* 每 2 ms 发一帧控制指令，电机长时间收不到指令会停机 */
        if ((HAL_GetTick() - last_tx) >= 2u)
        {
            last_tx = HAL_GetTick();

            /* 先只让 1 号电机小电压转动；2~7 号保持 0 */
            GM6020_SetVoltage(1, 3000);

            /* 一次把 0x1FF（ID1~4）和 0x2FF（ID5~7）两帧发出去 */
            (void)GM6020_SendVoltage();

            /*
             * 到这里，g_gm6020[1] 中的角度/转速/电流/温度
             * 已经由 CAN1 RX0 中断自动更新，可直接读取：
             *   g_gm6020[1].angle_deg
             *   g_gm6020[1].speed_rpm
             *   g_gm6020[1].torque_raw
             *   g_gm6020[1].temperature
             *   GM6020_IsOnline(1, HAL_GetTick())
             */
        }
    }
}

/**
 * @brief 72 MHz 系统时钟：HSE(8 MHz) → PLL ×9 → SYSCLK 72 MHz
 *        AHB = 72 MHz，APB1 = 36 MHz，APB2 = 72 MHz
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        Error_Handler();
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 错误处理：关中断并停住，方便调试器定位
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
