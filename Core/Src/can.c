#include "can.h"
#include "main.h"
extern FDCAN_HandleTypeDef hfdcan1;






CAN_Status CAN_AppInit(void)
{
FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    /* ID = 0，Mask = 0，先接收所有标准帧 */
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;

    filter.RxBufferIndex = 0;
    filter.IsCalibrationMsg = 0;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        return CAN_STATUS_ERROR;
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        return CAN_STATUS_ERROR;
    }

    return CAN_STATUS_OK;
}

static uint32_t CAN_LengthToDataLength(uint8_t length)
{
    switch (length)
    {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        default: return 0xFFFFFFFFU;
    }
}




CAN_Status CAN_SendStd(uint16_t std_id,
                       const uint8_t *data,
                       uint8_t length)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint32_t fdcan_length;

    if (std_id > 0x07FFU)
    {
        return CAN_STATUS_BAD_ARGUMENT;
    }

    if (data == 0)
    {
        return CAN_STATUS_BAD_ARGUMENT;
    }

    fdcan_length = CAN_LengthToDataLength(length);

    if (fdcan_length == 0xFFFFFFFFU)
    {
        return CAN_STATUS_BAD_ARGUMENT;
    }

    tx_header.Identifier = std_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = fdcan_length;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
                                      &tx_header,
                                      (uint8_t *)data) != HAL_OK)
    {
        return CAN_STATUS_ERROR;
    }

    return CAN_STATUS_OK;
}

static uint8_t CAN_DataLengthToLength(uint32_t data_length)
{
    switch (data_length)
    {
        case FDCAN_DLC_BYTES_0: return 0U;
        case FDCAN_DLC_BYTES_1: return 1U;
        case FDCAN_DLC_BYTES_2: return 2U;
        case FDCAN_DLC_BYTES_3: return 3U;
        case FDCAN_DLC_BYTES_4: return 4U;
        case FDCAN_DLC_BYTES_5: return 5U;
        case FDCAN_DLC_BYTES_6: return 6U;
        case FDCAN_DLC_BYTES_7: return 7U;
        case FDCAN_DLC_BYTES_8: return 8U;
        default:                return 0xFFU;
    }
}



CAN_Status CAN_ReadStd(uint32_t *std_id,
                       uint8_t *data,
                       uint8_t *length)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t received_length;

    if (std_id == 0 || data == 0 || length == 0)
    {
        return CAN_STATUS_BAD_ARGUMENT;
    }

    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1,
                                     FDCAN_RX_FIFO0) == 0U)
    {
        return CAN_STATUS_NO_DATA;
    }

    do
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1,
                                   FDCAN_RX_FIFO0,
                                   &rx_header,
                                   data) != HAL_OK)
        {
            return CAN_STATUS_ERROR;
        }
    }
    while ((rx_header.IdType != FDCAN_STANDARD_ID ||
            rx_header.RxFrameType != FDCAN_DATA_FRAME ||
            rx_header.FDFormat != FDCAN_CLASSIC_CAN) &&
           HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1,
                                        FDCAN_RX_FIFO0) > 0U);

    if (rx_header.IdType != FDCAN_STANDARD_ID ||
        rx_header.RxFrameType != FDCAN_DATA_FRAME ||
        rx_header.FDFormat != FDCAN_CLASSIC_CAN)
    {
        return CAN_STATUS_NO_DATA;
    }

    received_length = CAN_DataLengthToLength(rx_header.DataLength);

    if (received_length == 0xFFU)
    {
        return CAN_STATUS_ERROR;
    }

    *std_id = rx_header.Identifier;
    *length = received_length;

    return CAN_STATUS_OK;
}

