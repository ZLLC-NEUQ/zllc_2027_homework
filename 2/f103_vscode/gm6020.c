/**
 * @file    gm6020.c
 * @brief   GM6020 控制帧发送与反馈帧解析实现
 *
 * 反馈帧格式（8 字节）：
 *   [0..1] 机械角度 0~8191（大端）
 *   [2..3] 转速 rpm（int16，大端）
 *   [4..5] 转矩电流原始值（int16，大端）
 *   [6]    温度 ℃
 *   [7]    保留
 */

#include "gm6020.h"
#include <string.h>

GM6020_Motor_t g_gm6020[GM6020_MOTOR_NUM + 1];

/* ---------------- 内部工具 ---------------- */

static int16_t GM6020_Clamp(int32_t value, int16_t max)
{
    if (value >  (int32_t)max) return  max;
    if (value < -(int32_t)max) return -max;
    return (int16_t)value;
}

static void GM6020_PutInt16(uint8_t *buf, int16_t value)
{
    uint16_t u = (uint16_t)value;
    buf[0] = (uint8_t)(u >> 8);   /* 高字节在前 */
    buf[1] = (uint8_t)(u & 0xFFu);
}

static int16_t GM6020_GetInt16(const uint8_t *buf)
{
    return (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

/* 一帧 4 个 int16，发给 0x1FF / 0x2FF */
static uint8_t GM6020_SendInt16x4(uint32_t std_id, const int16_t value[4])
{
    uint8_t data[8];
    uint8_t i;

    for (i = 0; i < 4u; i++)
    {
        GM6020_PutInt16(&data[2u * i], value[i]);
    }

    return CAN_BSP_SendStd(std_id, data, 8u);
}

/* ---------------- 反馈解析（注册为 CAN 接收回调） ---------------- */

static void GM6020_OnRx(const CAN_RxHeaderTypeDef *hdr, const uint8_t *data)
{
    uint8_t id;
    GM6020_Motor_t *motor;

    if (hdr->DLC < 8u || hdr->RTR != CAN_RTR_DATA)
    {
        return;
    }

    /* 只处理 GM6020 的反馈 ID：0x205 ~ 0x20B */
    if (hdr->StdId < (GM6020_FB_BASE + 1u) ||
        hdr->StdId > (GM6020_FB_BASE + GM6020_MOTOR_NUM))
    {
        return;
    }

    id    = (uint8_t)(hdr->StdId - GM6020_FB_BASE);
    motor = &g_gm6020[id];

    motor->angle_raw   = GM6020_GetInt16(&data[0]);
    motor->speed_rpm   = GM6020_GetInt16(&data[2]);
    motor->torque_raw  = GM6020_GetInt16(&data[4]);
    motor->temperature = data[6];

    /* 机械角度 0~8191 → 0~360° */
    motor->angle_deg = (float)motor->angle_raw * 360.0f / 8192.0f;

    motor->rx_count++;
    motor->last_rx_ms = HAL_GetTick();
    motor->online     = 1;
}

/* ---------------- 对外接口 ---------------- */

void GM6020_Init(void)
{
    uint8_t i;

    memset(g_gm6020, 0, sizeof(g_gm6020));

    for (i = 0; i <= GM6020_MOTOR_NUM; i++)
    {
        g_gm6020[i].id = i;
    }

    CAN_BSP_SetRxCallback(GM6020_OnRx);
}

void GM6020_SetVoltage(uint8_t id, int16_t voltage)
{
    if (id < 1u || id > GM6020_MOTOR_NUM)
    {
        return;
    }
    g_gm6020[id].set_voltage = GM6020_Clamp(voltage, GM6020_VOLTAGE_MAX);
}

uint8_t GM6020_SendVoltage(void)
{
    int16_t value[4];
    uint8_t ret;
    uint8_t i;

    /* 0x1FF：电机 ID 1~4 */
    for (i = 1; i <= 4u; i++)
    {
        value[i - 1] = g_gm6020[i].set_voltage;
    }
    ret = GM6020_SendInt16x4(GM6020_CMD_ID_1_4, value);
    if (ret != 0u)
    {
        return ret;
    }

    /* 0x2FF：电机 ID 5~7，第 4 个槽位填 0 */
    value[0] = g_gm6020[5].set_voltage;
    value[1] = g_gm6020[6].set_voltage;
    value[2] = g_gm6020[7].set_voltage;
    value[3] = 0;

    return GM6020_SendInt16x4(GM6020_CMD_ID_5_7, value);
}

GM6020_Motor_t *GM6020_Get(uint8_t id)
{
    if (id < 1u || id > GM6020_MOTOR_NUM)
    {
        return NULL;
    }
    return &g_gm6020[id];
}

uint8_t GM6020_IsOnline(uint8_t id, uint32_t now_ms)
{
    GM6020_Motor_t *motor = GM6020_Get(id);

    if (motor == NULL || motor->online == 0u)
    {
        return 0;
    }

    if ((now_ms - motor->last_rx_ms) > GM6020_ONLINE_TIMEOUT)
    {
        motor->online = 0;
    }

    return motor->online;
}
