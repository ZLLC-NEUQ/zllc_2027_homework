/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fdcan.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define C620_ID_BASE  0x200U
#define C620_ID_MIN		0x201U
#define C620_ID_MAX		0x208U
#define	C620_FRAME_DLC	8U//帧固定8字节

//反馈数据
typedef struct
{
	uint8_t esc_id;
	uint8_t raw[8];
	uint16_t	angle_raw;//角度初始值
	int16_t speed_rpm;
	int16_t cur_raw;
	uint8_t temp_c;
}c620_feedback_t;

c620_feedback_t c620_fb;
volatile uint32_t valid_frame_count;//有效反馈计数
volatile uint32_t reject_count;

//加载缓冲区
static uint8_t rx_data[C620_FRAME_DLC];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
	FDCAN_FilterTypeDef filter_config;//过滤器过滤ID
	filter_config.IdType=FDCAN_STANDARD_ID;
	filter_config.FilterIndex=0U;
	filter_config.FilterType=FDCAN_FILTER_RANGE;
	filter_config.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
	filter_config.FilterID1=C620_ID_MIN;
	filter_config.FilterID2=C620_ID_MAX;
	if(HAL_FDCAN_ConfigFilter(&hfdcan1,&filter_config)!=HAL_OK)
	{
		Error_Handler();
	}
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	if(HAL_FDCAN_Start(&hfdcan1)!=HAL_OK)
	{
		Error_Handler();
	}
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0U);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/* 每200ms把最新C620反馈通过USART1发给串口调试助手（阻塞发送，期间CAN中断照常抢占接收） */
	static uint32_t last_print_tick = 0U;
	static char uart_buf[96];
	if (HAL_GetTick() - last_print_tick >= 200U)
	{
		int len;
		last_print_tick = HAL_GetTick();
		len = snprintf(uart_buf, sizeof(uart_buf),
			"id=%u ang=%u spd=%d cur=%d temp=%u valid=%lu reject=%lu\r\n",
			c620_fb.esc_id,
			c620_fb.angle_raw,
			c620_fb.speed_rpm,
			c620_fb.cur_raw,
			c620_fb.temp_c,
			(unsigned long)valid_frame_count,
			(unsigned long)reject_count);
		
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static uint16_t unpack_be_u16(const uint8_t *p)//大端拼无符号16位（角度）
{
	uint16_t v=((uint16_t)p[0]<<8)|(uint16_t)p[1];//高字节在DATA[n]，推向高8位
	return v;
}
static int16_t unpack_be_s16(const uint8_t *p)//大端拼有符号16位（转速/电流）
{
	uint16_t v=((uint16_t)p[0]<<8)|(uint16_t)p[1];
	return (int16_t)v;
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t it_flags)
{//回调函数
	FDCAN_RxHeaderTypeDef rx_header;
	if((it_flags&FDCAN_IT_RX_FIFO0_NEW_MESSAGE)==0U)
	{
		return;
	}
	while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan,FDCAN_RX_FIFO0)>0U)
	{
		if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&rx_header,rx_data)!=HAL_OK)
		{
			break;
		}
		uint8_t ok=1U;//对返回报文数据帧进行检查
		if((rx_header.Identifier<C620_ID_MIN)||(rx_header.Identifier>C620_ID_MAX))
		{
			ok=0U;
		}
		if(rx_header.FDFormat!=FDCAN_CLASSIC_CAN)//数据格式必须是CAN数据帧
		{
			ok=0U;
		}
		if(rx_header.RxFrameType!=FDCAN_DATA_FRAME)
		{
			ok=0U;
		}
		if(rx_header.DataLength!=FDCAN_DLC_BYTES_8)
		{
			ok=0U;
		}
		if(ok==0U)
		{
			reject_count++;
			continue;
		}
		//进行解包
		c620_fb.esc_id=(uint8_t)(rx_header.Identifier-C620_ID_BASE);
		memcpy(c620_fb.raw,rx_data,C620_FRAME_DLC);//目的  原地址  长度
		
		c620_fb.angle_raw=unpack_be_u16(&rx_data[0]);
		c620_fb.speed_rpm=unpack_be_s16(&rx_data[2]);
		c620_fb.cur_raw=unpack_be_s16(&rx_data[4]);
		c620_fb.temp_c=rx_data[6];
		
		valid_frame_count++;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
