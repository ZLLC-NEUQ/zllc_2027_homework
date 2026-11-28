#include "bsp_fdcan.h"
#include <string.h>

/* ===== C620 反馈协议常量 ===== */
#define C620_ID_BASE      0x200U   /* 反馈帧基地址 */
#define C620_ID_MIN       0x201U   /* 电调ID=1，反馈帧ID下限 */
#define C620_ID_MAX       0x208U   /* 电调ID=8，反馈帧ID上限 */
#define C620_FRAME_DLC    8U       /* 每帧固定8字节 */

/* 单个C620的反馈数据（本题只接1个电调） */
typedef struct
{
    uint8_t  esc_id;            /* 电调ID，1~8（由帧ID推算：ID-0x200） */
    uint8_t  raw[8];            /* 数据域原始8字节 */
    uint16_t angle_raw;         /* 角度原始值 0~8191，无符号，大端 */
    int16_t  speed_rpm;         /* 转速，有符号，RPM，大端 */
    int16_t  current_raw;       /* 实际转矩电流原始值，有符号，大端 */
    uint8_t  temp_c;            /* 温度 ℃ */
} c620_feedback_t;

c620_feedback_t c620_fb;               /* 最新有效反馈（Watch窗口观察这个） */
volatile uint32_t valid_frame_count;   /* 有效帧计数 */
volatile uint32_t reject_count;        /* 被报文检查拒绝的帧数 */

/* 接收临时缓冲（中断中GetRxMessage把数据搬到这里） */
static uint8_t rx_data[C620_FRAME_DLC];

/**
************************************************************************
* @brief:  can_filter_init
* @note:   FDCAN1硬件过滤器：只放行标准ID 0x201~0x208 到FIFO0
************************************************************************
**/
static void can_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType      = FDCAN_STANDARD_ID;       /* 11位标准帧 */
    fdcan_filter.FilterIndex = 0U;
    fdcan_filter.FilterType  = FDCAN_FILTER_RANGE;      /* ID区间过滤 */
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;/* 命中送FIFO0 */
    fdcan_filter.FilterID1   = C620_ID_MIN;             /* 区间下限 0x201 */
    fdcan_filter.FilterID2   = C620_ID_MAX;             /* 区间上限 0x208 */

    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

    /* 全局过滤：未命中过滤器的标准/扩展帧一律拒绝；远程帧也拒绝 */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1U);
}

/**
************************************************************************
* @brief:  bsp_can_init
* @note:   配置过滤器、启动FDCAN1、使能FIFO0新报文中断
************************************************************************
**/
void bsp_can_init(void)
{
    can_filter_init();
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
}

/* ===== 大端解包辅助函数 ===== */

/* 大端拼无符号16位：高字节在前 */
static uint16_t unpack_be_u16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

/* 大端拼有符号16位：拼成uint16后强转int16_t，负数（如0xFF9C）才会变成-100 */
static int16_t unpack_be_s16(const uint8_t *p)
{
    uint16_t v = ((uint16_t)p[0] << 8) | (uint16_t)p[1];
    return (int16_t)v;
}

/**
************************************************************************
* @brief:  HAL_FDCAN_RxFifo0Callback
* @note:   FIFO0收到新报文的中断回调。
*          while循环把FIFO取空（一次中断可能压入多帧），
*          先做四样报文检查，全部合法才解包更新。
************************************************************************
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        /* ===== 报文检查：任一不满足都不得更新有效反馈 ===== */
        uint8_t ok = 1U;

        /* 1) ID必须在 0x201~0x208 */
        if ((rx_header.Identifier < C620_ID_MIN) ||
            (rx_header.Identifier > C620_ID_MAX))
        {
            ok = 0U;
        }
        /* 2) 必须是经典CAN帧（不是CAN-FD） */
        if (rx_header.FDFormat != FDCAN_CLASSIC_CAN)
        {
            ok = 0U;
        }
        /* 3) 必须是数据帧（不是远程帧） */
        if (rx_header.RxFrameType != FDCAN_DATA_FRAME)
        {
            ok = 0U;
        }
        /* 4) DLC必须是8字节 */
        if (rx_header.DataLength != FDCAN_DLC_BYTES_8)
        {
            ok = 0U;
        }

        if (ok == 0U)
        {
            reject_count++;
            continue;
        }

        /* ===== 全部合法，按大端解包 ===== */
        c620_fb.esc_id = (uint8_t)(rx_header.Identifier - C620_ID_BASE);
        memcpy(c620_fb.raw, rx_data, C620_FRAME_DLC);

        c620_fb.angle_raw   = unpack_be_u16(&rx_data[0]);  /* DATA[0..1]，0~8191 */
        c620_fb.speed_rpm   = unpack_be_s16(&rx_data[2]);  /* DATA[2..3]，有符号RPM */
        c620_fb.current_raw = unpack_be_s16(&rx_data[4]);  /* DATA[4..5]，有符号原始值 */
        c620_fb.temp_c      = rx_data[6];                  /* DATA[6]，温度℃ */
        /* DATA[7] 为Null/保留，不赋予任何含义 */

        valid_frame_count++;
    }
}
