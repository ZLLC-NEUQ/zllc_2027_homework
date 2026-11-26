/**
 * @file    can_bsp.c
 * @brief   STM32F103 CAN1 手写底层驱动实现（HAL 库）
 *
 * 时钟：SYSCLK = 72 MHz，APB1 = 36 MHz（CAN 时钟 = APB1）
 * 波特率：36 MHz / 4 / (1 + 7 + 1) = 1 Mbps，采样点 88.9%
 */

#include "can_bsp.h"
#include <string.h>

/* main.c 中实现 */
extern void Error_Handler(void);

CAN_HandleTypeDef hcan1;

static CAN_RxCallback_t s_rx_callback = NULL;

/* =====================================================================
 * 一、GPIO / 时钟 / NVIC（HAL 弱函数，HAL_CAN_Init 内部自动调用）
 * ===================================================================== */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    GPIO_InitTypeDef gpio = {0};

    if (hcan->Instance != CAN1)
    {
        return;
    }

    /* 外设时钟：CAN1 在 APB1，GPIOA / AFIO 在 APB2 */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* PA11 = CAN1_RX：输入浮空 */
    gpio.Pin  = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA12 = CAN1_TX：复用推挽输出 */
    gpio.Pin   = GPIO_PIN_12;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* CAN1 RX0 中断（与 USB 低优先级中断共用向量） */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1)
    {
        return;
    }

    __HAL_RCC_CAN1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
}

/* =====================================================================
 * 二、中断入口
 * 注意：如果你的工程里 stm32f1xx_it.c 已经写了同名函数，只保留一处！
 * ===================================================================== */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

/* =====================================================================
 * 三、过滤器：只接收 0x200 ~ 0x20F 的标准帧（覆盖 GM6020 反馈 0x205~0x20B）
 * ===================================================================== */
static HAL_StatusTypeDef CAN_BSP_ConfigFilter(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;

    /*
     * 32 位掩码过滤器的 ID 寄存器位置：
     *   标准 ID 位于 STID[10:0] << 21，
     *   HAL 的 FilterIdHigh 是 32 位值的高 16 位，所以写入 “ID << 5”。
     * 掩码 0x7F0 只比较高 7 位，因此 0x200 ~ 0x20F 全部通过。
     */
    filter.FilterIdHigh     = (uint16_t)(0x200u << 5);   /* 0x4000 */
    filter.FilterIdLow      = 0x0000u;
    filter.FilterMaskIdHigh = (uint16_t)(0x7F0u << 5);   /* 0xFE00 */
    filter.FilterMaskIdLow  = 0x0000u;

    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;

    return HAL_CAN_ConfigFilter(&hcan1, &filter);
}

/* =====================================================================
 * 四、初始化 CAN1
 * ===================================================================== */
HAL_StatusTypeDef CAN_BSP_Init(void)
{
    hcan1.Instance = CAN1;

    /* 1 Mbps：APB1 36 MHz / Prescaler 4 = 9 MHz（Tq ≈ 111.1 ns）
     * 1 位 = (1 + 7 + 1) Tq = 9 Tq → 1 µs，采样点 (1+7)/9 = 88.9% */
    hcan1.Init.Prescaler         = 4;
    hcan1.Init.Mode              = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth     = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1          = CAN_BS1_7TQ;
    hcan1.Init.TimeSeg2          = CAN_BS2_1TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff        = DISABLE;
    hcan1.Init.AutoWakeUp        = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (CAN_BSP_ConfigFilter() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 只打开 FIFO0 “收到新报文”中断，需要时再加错误中断 */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* =====================================================================
 * 五、发送
 * ===================================================================== */
uint8_t CAN_BSP_SendStd(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox;
    uint8_t buf[8] = {0};

    if (data == NULL || len > 8u)
    {
        return 1;
    }

    memcpy(buf, data, len);

    tx.StdId   = std_id & 0x7FFu;
    tx.ExtId   = 0;
    tx.IDE     = CAN_ID_STD;
    tx.RTR     = CAN_RTR_DATA;
    tx.DLC     = len;
    tx.TransmitGlobalTime = DISABLE;

    /* 三个发送邮箱都占满时直接返回，交给调用者下个周期重发 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0u)
    {
        return 2;
    }

    if (HAL_CAN_AddTxMessage(&hcan1, &tx, buf, &mailbox) != HAL_OK)
    {
        return 3;
    }

    return 0;
}

uint32_t CAN_BSP_GetFreeMailbox(void)
{
    return HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
}

/* =====================================================================
 * 六、接收：FIFO0 非空中断 → 读出所有帧 → 回调业务层
 * ===================================================================== */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    if (hcan->Instance != CAN1)
    {
        return;
    }

    /* 一次把 FIFO0 里积压的帧全部取完（HAL 内部会自动释放 RFOM） */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0u)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) == HAL_OK)
        {
            if (s_rx_callback != NULL)
            {
                s_rx_callback(&hdr, data);
            }
        }
    }
}

void CAN_BSP_SetRxCallback(CAN_RxCallback_t cb)
{
    s_rx_callback = cb;
}
