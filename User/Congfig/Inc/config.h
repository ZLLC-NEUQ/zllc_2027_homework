/**
 * @file config.h
 * @author cjw
 * @brief 工程配置文件
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef CONFIG_H
#define CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

//底盘或云台状态
//#define CHASSIS
#define GIMBAL

//底盘标定参考正方向角度(数据来源yaw电机)
#define Reference_Angle  -1.4265281
#define Reference_Radian  (Reference_Angle * PI / 180.f)

//调试或比赛状态
#define DEBUG
//#define BATTLE


//功率控制相关
#define POWER_LIMIT_JH
//#define POWER_LIMIT_GY
#define POWER_CONTROL 1 //启用功率控制
//#define BUFFER_LOOP

#define Heat_Detect_ENABLE
//#define Heat_Detect_DISABLE

#define Forward_Power

#define SuperCap 0

//遥控器选择
//#define USE_VT13
//#define USE_DR16
#define USE_FS_i6X
/* 兵种/底盘类型选择*/
 #define AGV      //舵轮底盘
#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#define Chassis_Radius 0.46000000f // 底盘半径，单位为m

#define H7_Offset_X 0.51465f          // 旋转中心到H7的x方向偏移，单位为m，正值表示H7在旋转中心前方，底盘坐标系
#define H7_Offset_Y 0.12475f        // 旋转中心到H7的y方向偏移，单位为m，正值表示H7在旋转中心左侧，底盘坐标系



/* Exported types ------------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
