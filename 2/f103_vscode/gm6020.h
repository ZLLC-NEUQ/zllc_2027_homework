/**
 * @file    gm6020.h
 * @brief   GM6020 云台电机：控制帧发送 + 反馈帧解析（结果存进结构体）
 *
 * 依赖：can_bsp.h 提供的 CAN1 底层
 */

#ifndef __GM6020_H__
#define __GM6020_H__

#include "can_bsp.h"
#include <stdint.h>

#define GM6020_MOTOR_NUM     7       /* 电机 ID 1~7 */
#define GM6020_VOLTAGE_MAX   30000   /* 控制量范围 ±30000 */
#define GM6020_ONLINE_TIMEOUT 100u   /* 超过 100 ms 无反馈判离线 */

/* 控制帧 ID */
#define GM6020_CMD_ID_1_4    0x1FFu  /* 电压控制：ID 1~4 */
#define GM6020_CMD_ID_5_7    0x2FFu  /* 电压控制：ID 5~7 */

/* 反馈帧 ID = 0x204 + 电机 ID，即 0x205 ~ 0x20B */
#define GM6020_FB_BASE       0x204u

/* ---------------- 电机数据结构体 ---------------- */
typedef struct
{
    uint8_t  id;            /* 电机 ID 1~7 */

    /* 控制量 */
    int16_t  set_voltage;   /* 下发的电压指令 -30000 ~ +30000 */

    /* 反馈量 */
    int16_t  angle_raw;     /* 机械角度原始值 0~8191 */
    float    angle_deg;     /* 换算后的机械角度 0~360° */
    int16_t  speed_rpm;     /* 转速，rpm */
    int16_t  torque_raw;    /* 转矩电流原始值 */
    uint8_t  temperature;   /* 温度，℃ */

    /* 运行状态 */
    uint32_t rx_count;      /* 累计收到的反馈帧数 */
    uint32_t last_rx_ms;    /* 最近一次收到反馈的时刻 */
    uint8_t  online;        /* 1 = 在线，0 = 离线 */
} GM6020_Motor_t;

/* 下标 1~7 对应电机 ID，下标 0 不使用 */
extern GM6020_Motor_t g_gm6020[GM6020_MOTOR_NUM + 1];

/**
 * @brief 初始化：清空结构体，并把 GM6020 的解析函数注册为 CAN 接收回调
 * @note  必须在 CAN_BSP_Init() 之后调用
 */
void GM6020_Init(void);

/** @brief 设置指定 ID 电机的电压指令（-30000 ~ +30000，内部限幅） */
void GM6020_SetVoltage(uint8_t id, int16_t voltage);

/**
 * @brief 将当前所有电机的电压指令打包发出去（两帧：0x1FF、0x2FF）
 * @retval 0 全部发送成功；其它值见 CAN_BSP_SendStd()
 * @note   建议每 1~2 ms 调用一次，电机长时间收不到指令会停机
 */
uint8_t GM6020_SendVoltage(void);

/** @brief 取指定 ID 电机的数据指针，非法 ID 返回 NULL */
GM6020_Motor_t *GM6020_Get(uint8_t id);

/** @brief 判断电机是否在线（内部会刷新超时状态） */
uint8_t GM6020_IsOnline(uint8_t id, uint32_t now_ms);

#endif /* __GM6020_H__ */
