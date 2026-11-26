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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
typedef struct {
  int16_t voltage;
  int16_t angle;
  int16_t speed;
  int16_t current;
  int16_t tempereture;

}GM6020_struct;
GM6020_struct GM6020;
/**
 * @brief  16位列表模式过滤器配置
 * @param  hcan   CAN的句柄
 * @param  StdId1~StdId4  4个标准ID
 */
void CANFilterConfig_Scale16_IdList(CAN_HandleTypeDef * hcan,uint32_t StdId1,uint32_t StdId2,uint32_t StdId3,uint32_t StdId4,uint8_t Filter_Nunber)  
{  
  CAN_FilterTypeDef  sFilterConfig;  
  sFilterConfig.FilterBank = Filter_Nunber;         //使用过滤器1  
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;   //设为列表模式  
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  //位宽设置为16位  
  sFilterConfig.FilterIdHigh = StdId1<<5;  //4个标准CAN ID分别放入到4个存储中  
  sFilterConfig.FilterIdLow = StdId2<<5;  
  sFilterConfig.FilterMaskIdHigh = StdId3<<5;  
  sFilterConfig.FilterMaskIdLow = StdId4<<5;  
  sFilterConfig.FilterFIFOAssignment = 0;   //接收到的报文放入到FIFO0中  
  sFilterConfig.FilterActivation = ENABLE;    
  HAL_CAN_ConfigFilter(hcan, &sFilterConfig); 
}

/**
 * @brief  发送标准ID的数据帧
 * @param  hcan     CAN的句柄
 * @param  ID       数据帧ID
 * @param  pData    数组指针
 * @param  Len      字节数0~8
 */
uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len)
{
  static CAN_TxHeaderTypeDef   Tx_Header;
	
	Tx_Header.StdId=ID;
	Tx_Header.ExtId=0;
	Tx_Header.IDE=0;
	Tx_Header.RTR=0;
	Tx_Header.DLC=Len;
        /*找到空的发送邮箱，把数据发送出去*/
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX2);
    }
  }
}
/**
 * @brief  CAN FIFO0的中断回调函数，在里面完成数据的接收
 * @param  hcan     CAN的句柄
 */
 
uint8_t date[8];//设为全局变量，用于接收CAN1数据

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance ==CAN1)
	{
    uint8_t data[8];
	  CAN_RxHeaderTypeDef RxHeader;  //接受句柄
	  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data); //接收，CAN邮箱为0
    GM6020.angle    = (int16_t)((data[0] << 8) | data[1]);
    GM6020.speed    = (int16_t)((data[2] << 8) | data[3]);
    GM6020.current  = (int16_t)((data[4] << 8) | data[5]);
    GM6020.tempereture = data[6];
	}
	
}
void CAN_Start(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_ActivateNotification(hcan ,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(hcan);
}


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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  CANFilterConfig_Scale16_IdList(&hcan,0x205,0x206,0x207,0x208,0);
  CAN_Start(&hcan);
  uint32_t t = 0;
  uint8_t tx[8]={0};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (HAL_GetTick() - t >= 2)    /* 每 2 ms 发一帧 */
    {
        t = HAL_GetTick();
        GM6020.voltage = 3000;
        tx[0] = (uint8_t)(GM6020.voltage >> 8);       /* 高字节 */
        tx[1] = (uint8_t)(GM6020.voltage & 0xFF);     /* 低字节 */
        CANx_SendStdData(&hcan,0x1FF,tx,8);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
