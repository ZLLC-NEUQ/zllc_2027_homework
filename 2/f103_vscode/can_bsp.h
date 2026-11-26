/**
 * @file    can_bsp.h
 * @brief   STM32F103 CAN1 手写底层驱动（HAL 库，VS Code / EIDE 工程）
 *
 * 说明：
 *   - 不使用 CubeMX 生成的 can.c / MX_CAN1_Init()，初始化全部在本文件完成；
 *   - 时钟、GPIO、过滤器、NVIC、启动、中断接收都在这里配好；
 *   - 业务层只需要 CAN_BSP_Init() / CAN_BSP_SendStd() / CAN_BSP_SetRxCallback()。
 *
 * 引脚：PA11 = CAN1_RX，PA12 = CAN1_TX（如需 PB8/PB9 要开重映射）
 * 总线：1 Mbps，标准帧，数据帧
 */

#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 接收回调：每收到一帧标准数据帧调用一次 */
typedef void (*CAN_RxCallback_t)(const CAN_RxHeaderTypeDef *hdr,
                                 const uint8_t *data);

/* HAL 的 CAN 句柄，业务层取用 hcan1 */
extern CAN_HandleTypeDef hcan1;

/**
 * @brief  初始化 CAN1：时钟/GPIO/参数/过滤器/NVIC/启动/接收中断
 * @retval HAL_OK 成功
 * @note   需先调用 HAL_Init() 并配好 72 MHz 时钟（APB1 = 36 MHz）
 */
HAL_StatusTypeDef CAN_BSP_Init(void);

/**
 * @brief  发送一帧标准数据帧（非阻塞）
 * @param  std_id 11 位标准 ID
 * @param  data   数据指针
 * @param  len    0~8
 * @retval 0 成功入邮箱；1 参数错误；2 三个邮箱全忙；3 入队失败
 */
uint8_t CAN_BSP_SendStd(uint32_t std_id, const uint8_t *data, uint8_t len);

/** @brief 注册接收回调（在 CAN_BSP_Init 之后调用） */
void CAN_BSP_SetRxCallback(CAN_RxCallback_t cb);

/** @brief 查询当前空闲发送邮箱数量 0~3（调试用） */
uint32_t CAN_BSP_GetFreeMailbox(void);

#endif /* __CAN_BSP_H__ */
