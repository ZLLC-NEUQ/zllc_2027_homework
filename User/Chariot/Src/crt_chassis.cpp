/**
 * @file crt_chassis.cpp
 * @author lez by wanghongxi
 * @brief 底盘
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/**
 * @brief 轮组编号
 * 3 2
 *  1
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"
#include "buzzer.h"
#include "drv_math.h"
#include "config.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
//
//
// chasiss是借用代码最多的【哭】
//
//
//这些就直接用了吧
float Chassis_Speed_Kalman_F[36] = {1.0f, 0.0f, 0.002f, 0.0f, 0.0f, 0.0f,
                                    0.0f, 1.0f, 0.0f, 0.002f, 0.0f, 0.0f,
                                    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                                    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};

float Chassis_Speed_Kalman_H[36] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                                    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                                    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f};

float Chassis_Speed_Kalman_P[36] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                                    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
// 过程模型噪声
float Chassis_Speed_Kalman_Q[36] = {0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,   // Vx
                                    0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f,   // Vy
                                    0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f,    // ax
                                    0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f,    // ay
                                    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,    // w
                                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.001f}; // b(陀螺仪漂移)
// 观测过程噪声
float Chassis_Speed_Kalman_R[36] = {15.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, // Vx
                                    0.0f, 15.0f, 0.0f, 0.0f, 0.0f, 0.0f, // Vy
                                    0.0f, 0.0f, 15.0f, 0.0f, 0.0f, 0.0f, // ax
                                    0.0f, 0.0f, 0.0f, 15.0f, 0.0f, 0.0f, // ay
                                    0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 0.0f,  // 逆解算出的角速度
                                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.3f}; // 陀螺仪的角速度
/**
 * @brief 底盘初始化
 *
 * @param __Chassis_Control_Type 底盘控制方式, 默认舵轮方式
 * @param __Speed 底盘速度限制最大值
 */
void Class_Steering_Wheel_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max, float __Steer_Power_Ratio)
{
    Supercap.Init(&hfdcan3,100.0f);

    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    Steer_Power_Ratio = __Steer_Power_Ratio;

    // 斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.05f,0.05f);
    // 斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.05f,0.05f);
    // 斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

#ifdef POWER_LIMIT
    // 超级电容初始化
    Supercap.Init(&hcan1, 45);
    Power_Limit.Init(400, 3500);
#ifdef POWER_LIMIT_BUFFER_LOOP
    Buffer_Loop_PID.Init(1.0f, 0, 0, 0, 60, 60);

#endif
#endif

    // 电机PID批量初始化
    // for (int i = 0; i < 4; i++)
    // {
    //     Motor_Wheel[i].PID_Omega.Init(2000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    // }
    Motor_Wheel[0].PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[0].Get_Output_Max(), Motor_Wheel[0].Get_Output_Max());
    Motor_Wheel[1].PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[1].Get_Output_Max(), Motor_Wheel[1].Get_Output_Max());
    Motor_Wheel[2].PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[2].Get_Output_Max(), Motor_Wheel[2].Get_Output_Max());
    Motor_Wheel[3].PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[3].Get_Output_Max(), Motor_Wheel[3].Get_Output_Max());

    // 轮向电机ID初始化
    Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OPENLOOP, M3508_REDUCTION_RATIO);
    Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OPENLOOP, M3508_REDUCTION_RATIO);
    Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_OPENLOOP, M3508_REDUCTION_RATIO);
    Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x207, DJI_Motor_Control_Method_OPENLOOP, M3508_REDUCTION_RATIO);

    //舵向电机ID初始化
    Motor_Steer[0].Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_AGV_MODE, 8.0f);
    Motor_Steer[1].Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_AGV_MODE, 8.0f);
    Motor_Steer[2].Init(&hfdcan1, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_AGV_MODE, 8.0f);
    Motor_Steer[3].Init(&hfdcan1, DJI_Motor_ID_0x208, DJI_Motor_Control_Method_AGV_MODE, 8.0f);

    for(int i=0;i<4;i++)
    {
        Motor_Steer[i].PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 15.0f, 15.0f);
        Motor_Steer[i].PID_Omega.Init(0.0f,0.0f, 0.0f, 0.0f, 8000, Motor_Steer[0].Get_Output_Max());
    }

    //舵向电机零点位置初始化
    Motor_Steer[0].Set_Zero_Position(1.26f);
    Motor_Steer[1].Set_Zero_Position(1.49f);
    Motor_Steer[2].Set_Zero_Position(2.22f);
    Motor_Steer[3].Set_Zero_Position(4.39f);//零点直接用了，应该不会特意拆电机再装上去吧
    //至少初始化是自己写的

    PID_Velocity_X.Init(0.0f, 0.0f, 0.0f, 0.0f, 150.0f, 500.0f, 0.002f);

    // 底盘速度yPID, 输出摩擦力
    PID_Velocity_Y.Init(0.0f, 0.0f, 0.0f, 0.0f, 150.0f, 500.0f, 0.002f);

    // 底盘角速度PID, 输出扭矩
    PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 12.0f, 0.002f);

    Kalman_Filter_Init(&Chassis_Speed_Kalman, 6, 0, 6);                               

    memcpy(Chassis_Speed_Kalman.F_data, Chassis_Speed_Kalman_F, sizeof(Chassis_Speed_Kalman_F));
    memcpy(Chassis_Speed_Kalman.H_data, Chassis_Speed_Kalman_H, sizeof(Chassis_Speed_Kalman_H));
    memcpy(Chassis_Speed_Kalman.P_data, Chassis_Speed_Kalman_P, sizeof(Chassis_Speed_Kalman_P));
    memcpy(Chassis_Speed_Kalman.Q_data, Chassis_Speed_Kalman_Q, sizeof(Chassis_Speed_Kalman_Q));
    memcpy(Chassis_Speed_Kalman.R_data, Chassis_Speed_Kalman_R, sizeof(Chassis_Speed_Kalman_R));

    //底盘控制方式初始化
    Chassis_Control_Type = Chassis_Control_Type_DISABLE;
}

// /**
//  * @brief 速度解算
//  *
//  */
// float temp_test_1, temp_test_2, temp_test_3, temp_test_4;
// void Class_Tricycle_Chassis::Speed_Resolution()
// {
//     // 获取当前速度值，用于速度解算初始值获取
//     switch (Chassis_Control_Type)
//     {
//     case (Chassis_Control_Type_DISABLE):
//     {
//         // 底盘失能 四轮子无力
//         for (int i = 0; i < 4; i++)
//         {
//             Motor_Wheel[i].Disable();
//         }
//     }
//     break;
//     case (Chassis_Control_Type_SPIN):
//     case (Chassis_Control_Type_FLLOW):
//     {
//         // 底盘四电机模式配置
//         for (int i = 0; i < 4; i++)
//         {
//             Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
//         }
//         // 底盘限速
//         if (Velocity_X_Max != 0)
//         {
//             Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
//         }
//         if (Velocity_Y_Max != 0)
//         {
//             Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
//         }
//         if (Omega_Max != 0)
//         {
//             Math_Constrain(&Target_Omega, -Omega_Max, Omega_Max);
//         }

// #ifdef SPEED_SLOPE
//         // 速度换算，正运动学分解
//         float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
//         float motor2_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
//         float motor3_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
//         float motor4_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
// #else
//         // 速度换算，正运动学分解
//         float motor1_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X + Target_Omega * (HALF_WIDTH + HALF_LENGTH);
//         float motor2_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X - Target_Omega * (HALF_WIDTH + HALF_LENGTH);
//         float motor3_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X + Target_Omega * (HALF_WIDTH + HALF_LENGTH);
//         float motor4_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X - Target_Omega * (HALF_WIDTH + HALF_LENGTH);
// #endif
//         // 线速度 cm/s  转角速度  RAD
//         float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
//         float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
//         float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;
//         float motor4_temp_rad = motor4_temp_linear_vel * VEL2RAD;
//         // 角速度*减速比  设定目标 直接给到电机输出轴
//         Motor_Wheel[0].Set_Target_Omega_Radian(motor2_temp_rad);
//         Motor_Wheel[1].Set_Target_Omega_Radian(-motor1_temp_rad);
//         Motor_Wheel[2].Set_Target_Omega_Radian(-motor3_temp_rad);
//         Motor_Wheel[3].Set_Target_Omega_Radian(motor4_temp_rad);
//         // 各个电机具体PID
//         for (int i = 0; i < 4; i++)
//         {
//             Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
//         }
//     }
//     break;
//     }
// }
// //Enum_Supercap_Mode test_mode = Supercap_Mode_ENABLE;
// float test_power = 58.0f;
// float compensate_max_power = 30.0f;
// /**
//  * @brief TIM定时器中断计算回调函数
//  *
//  */
// float Chassis_Buffer = 0.0;
// float Power_Limit_K = 1.0f;
// void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
// {
// #ifdef SPEED_SLOPE

//     // 斜坡函数计算用于速度解算初始值获取
//     Slope_Velocity_X.Set_Target(Target_Velocity_X);
//     Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
//     Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
//     Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
//     Slope_Omega.Set_Target(Target_Omega);
//     Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

// #endif
//     // 速度解算
//     Speed_Resolution();

//     // float Chassis_Buffer = 0.0;
//     //计算限制功率
//     //if(Referee->Get_Referee_Status() == Referee_Status_ENABLE){
//         //缓冲环限制功率
//         Chassis_Buffer = Referee->Get_Chassis_Energy_Buffer();
//         Power_Management.Buffer_Power = Referee->Get_Chassis_Energy_Buffer() - 30.0f;//(sqrt(Chassis_Buffer) - sqrt(Power_Management.Min_Buffer)) * Power_Management.Buffer_K;
//         //Math_Constrain(&Power_Management.Buffer_Power, -30.0f, 0.0f);

//         if (Supercap.Get_Supercap_Status() != Supercap_Status_DISABLE && __Sprint_Status == Sprint_Status_ENABLE)
//         {
//             Power_Management.Max_Power = Referee->Get_Chassis_Power_Max();
//         }
//         else
//         {
//             //Power_Management.Max_Power = Power_Management.Buffer_Power + Referee->Get_Chassis_Power_Max();
//             Power_Management.Max_Power = Referee->Get_Chassis_Power_Max();
//         }
// //    }
// //    else{
// //        //裁判系统离线限制功率
// //        Power_Management.Max_Power = 70.0f;
// //        Chassis_Buffer = 0.0f;
// //    }
// //    
//     Power_Management.Actual_Power = Supercap.Get_Chassis_Actual_Power();//Referee->Get_Chassis_Power();
//     Power_Management.Total_error = 0.0f;

// #ifdef AGV
//     for (int i = 0; i < 4; i++)         //数据传递处理
//     {
//         //都是计算转子的
//         Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Wheel[i].Get_Gearbox_Rate();
//         Power_Management.Motor_Data[i].feedback_torque = Motor_Wheel[i].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;     //与减速比有关
//         Power_Management.Motor_Data[i].torque = Motor_Wheel[i].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;                     //与减速比有关
//         Power_Management.Motor_Data[i].pid_output = Motor_Wheel[i].Get_Out();

//         Power_Management.Motor_Data[i + 4].feedback_omega  = Motor_Steer[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Steer[i].Get_Gearbox_Rate();
//         Power_Management.Motor_Data[i + 4].feedback_torque = Motor_Steer[i].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
//         Power_Management.Motor_Data[i + 4].torque          = Motor_Steer[i].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
//         Power_Management.Motor_Data[i + 4].pid_output      = Motor_Steer[i].Get_Out();  
//     }

//     Power_Limit.Power_Task(Power_Management);

//     for (int i = 0; i < 4; i++)
//     {
//         Motor_Wheel[i].Set_Out(Power_Management.Motor_Data[i].output);
//         Motor_Wheel[i].Output();

//         Motor_Steer[i].Set_Out(Power_Management.Motor_Data[i + 4].output);
//         Motor_Steer[i].Output();
//     }
// #else
//     for (int i = 0; i < 4; i++)         //数据传递处理
//     {
//         //都是计算转子的
//         Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Wheel[i].Get_Gearbox_Rate();
//         Power_Management.Motor_Data[i].feedback_torque = Motor_Wheel[i].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;     //与减速比有关
//         Power_Management.Motor_Data[i].torque = Motor_Wheel[i].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;                     //与减速比有关
//         Power_Management.Motor_Data[i].pid_output = Motor_Wheel[i].Get_Out();

//         Power_Management.Motor_Data[i].Target_error = fabs(Motor_Wheel[i].Get_Target_Omega_Radian() - Motor_Wheel[i].Get_Now_Omega_Radian());
        
//     }
//     Power_Management.Total_error = 0.0;
//     Power_Limit.Power_Task(Power_Management);

//     for (int i = 0; i < 4; i++)
//     {
//         Motor_Wheel[i].Set_Out(Power_Management.Motor_Data[i].output);
//         //Motor_Wheel[i].Output();
//     }
// #endif

//     //if(Referee->Get_Referee_Status() == Referee_Status_ENABLE){
//         Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() + Power_Management.Buffer_Power);


//     //Supercap.TIM_Supercap_PeriodElapsedCallback();          //向超电发送信息
//     float power = Supercap.Get_Limit_Power();
//     memcpy(CAN_Supercap_Tx_Data,&power,4);
//     uint8_t tmp = (uint8_t)SuperCap;
//     memcpy(CAN_Supercap_Tx_Data+4,&tmp,1);

// }
/**
 * @brief 速度解算
 * 
 */
//为什么我运动学解算看得明白，但就是不会写呢[哭][哭]
float True_Vx[4],True_Vy[4],True_Target_Angle_Radian[4];
float car_V,car_yaw;
void Class_Steering_Wheel_Chassis::Speed_Resolution()
{
    if (Motor_Steer[0].Get_MA600_Status() == MA600_Status_DISABLE || Motor_Steer[1].Get_MA600_Status() == MA600_Status_DISABLE ||
        Motor_Steer[2].Get_MA600_Status() == MA600_Status_DISABLE || Motor_Steer[3].Get_MA600_Status() == MA600_Status_DISABLE)
    {
       for (uint8_t i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Wheel[i].PID_Omega.Set_Integral_Error(0.0f);
            Motor_Wheel[i].Set_Out(0.0f);

            Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Steer[i].PID_Omega.Set_Integral_Error(0.0f);
            Motor_Steer[i].PID_Angle.Set_Integral_Error(0.0f);
            Motor_Steer[i].Set_Out(0.0f);
        }
        return;
    }//失能是最好写的[看]
#ifdef AGV
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Wheel[i].PID_Omega.Set_Integral_Error(0.0f);
            Motor_Wheel[i].Set_Out(0.0f);

            Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Steer[i].PID_Omega.Set_Integral_Error(0.0f);
            Motor_Steer[i].PID_Angle.Set_Integral_Error(0.0f);
            Motor_Steer[i].Set_Out(0.0f);
        }
        break;
    }
    // 舵轮运动学逆解
    case (Chassis_Control_Type_FLLOW):
    case (Chassis_Control_Type_SPIN_Positive):
    case (Chassis_Control_Type_Drive):
    {
        //原代码中用了轮组自锁，为了确保小车静止时的稳定性，每个小轮坐标系符合右手系
        //以下是我自己写的，虽然和原来的差不多。。。主要是借用了臭傻吊的思路
        static uint32_t lock_time=0;
        static uint8_t lock_flag=0;
        float delta_Angle = 0.0f, Transform_Radian = 0.0f;// 用于优化处理的变量
        if(abs(Target_Velocity_X)<0.01 && abs(Target_Velocity_Y)<0.01 && abs(Target_Omega)<0.01)
        {
            lock_time++;
            if(lock_time>500)//当前为速度，响应较快
            lock_flag=1;
            if(lock_flag)
            {
                for(int i=0;i<4;i++)
                {
                    Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE);
                    Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
                }

                Motor_Steer[0].Set_Target_Radian(-PI / 4.0f);
                Motor_Steer[1].Set_Target_Radian(PI / 4.0f);
                Motor_Steer[2].Set_Target_Radian(-PI / 4.0f);
                Motor_Steer[3].Set_Target_Radian(PI / 4.0f);

                for (int i = 0; i < 4; i++)
                {
                    Transform_Radian = Motor_Steer[i].Get_Now_Zero_Offset_Radian();

                    // 优劣弧处理
                    if ((i % 2) == 0)
                    {
                        delta_Angle = -PI / 4.0f - Transform_Radian;
                    }
                    else
                    {
                        delta_Angle = PI / 4.0f - Transform_Radian;
                    }

                    delta_Angle = Normalize_Angle_Radian_PI_to_PI(delta_Angle); // 将角度限制在-PI到PI之间

                    if (delta_Angle > PI / 2.0f)
                    {
                        delta_Angle = delta_Angle - PI;
                    }
                    else if (delta_Angle < -PI / 2.0f)
                    {
                        delta_Angle = delta_Angle + PI;
                    }
                    Motor_Steer[i].Set_Target_Radian(Transform_Radian + delta_Angle);//静止时把轮子控制在45°的位置，稳定性更高
                    Motor_Steer[i].Set_Transform_Radian(Transform_Radian);
                    Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
                    Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
                }
                break;
            }
        }
        else
        {
            lock_time = 0;
        }

        if (lock_flag)
        {
            lock_flag = 0;
        }
        //
        // 0 1 2 3 左前 右前 右后 左后 顺时针    右x前y坐标系   基于编码器0度朝前，逆时针为正角度   确保轮子正转的是朝前的速度，不然得单独加负号
        True_Vx[0] = True_Vx[1] = Slope_Velocity_X.Get_Out() - sqrt(2)/2 * Target_Omega * R_DIST;//这里以前是乘上了一个4，说是这样转的速度更快，但是正常的解算是不需要的，总之让我先试试再说
        True_Vx[0] = True_Vx[1] = Slope_Velocity_X.Get_Out() + sqrt(2)/2 * Target_Omega * R_DIST;
        True_Vy[0] = True_Vy[3] = Slope_Velocity_Y.Get_Out() + sqrt(2)/2 * Target_Omega * R_DIST;
        True_Vy[1] = True_Vy[2] = Slope_Velocity_Y.Get_Out() - sqrt(2)/2 * Target_Omega * R_DIST;

        for(int i=0;i<4;i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE); // 舵轮控制模式

            float temp_Target_Omega = 0.0f;
            arm_sqrt_f32(True_Vx[i]*True_Vx[i]+True_Vy[i]*True_Vy[i],&temp_Target_Omega);
            temp_Target_Omega = temp_Target_Omega / WHEEL_RADIUS;//转化为角速度
            
            if (fabs(temp_Target_Omega)<0.0001 && True_Vx[i]==0.0f && True_Vy[i]==0.0f)
            {
                True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian();
            }
            else
            {
                True_Target_Angle_Radian[i] = atan2f(True_Vy[i], True_Vx[i]);
            }

            // 角度优化处理
            delta_Angle = True_Target_Angle_Radian[i] - Motor_Steer[i].Get_Now_Zero_Offset_Radian(); // -2PI -- 2PI
            delta_Angle = Normalize_Angle_Radian_PI_to_PI(delta_Angle);                              // 处理重叠的角度（-20 = 340），归一化到 -PI --- PI
            if (delta_Angle > PI / 2.0f)
            {
                True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle - PI;
                temp_Target_Omega *= -1.0f;
            }
            else if (delta_Angle < -PI / 2.0f)
            {
                True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle + PI;
                temp_Target_Omega *= -1.0f;
            }
            else
            {
                // 不需要处理角度
                True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle;
            }

            // 处理-180 - 180的突变问题    同时还有优劣弧处理
            True_Target_Angle_Radian[i] = Normalize_Angle_Radian_PI_to_PI(True_Target_Angle_Radian[i]);

            Motor_Steer[i].Set_Target_Radian(True_Target_Angle_Radian[i]);
            Motor_Wheel[i].Set_Target_Omega_Radian(temp_Target_Omega);
        }

        for (int i = 0; i < 4; i++)
        {
            Transform_Radian = Motor_Steer[i].Get_Now_Zero_Offset_Radian();
            Motor_Steer[i].Set_Transform_Radian(Transform_Radian);
            Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
            Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
        }
        break;

    }
    }
#endif
}

void Class_Steering_Wheel_Chassis::Set_Chassis_Kalman_Measure(float value1, float value2, float value3, float value4, float value5, float value6)
{
    Chassis_Speed_Kalman.MeasuredVector[0] = value1;
    Chassis_Speed_Kalman.MeasuredVector[1] = value2;
    Chassis_Speed_Kalman.MeasuredVector[2] = value3;
    Chassis_Speed_Kalman.MeasuredVector[3] = value4;
    Chassis_Speed_Kalman.MeasuredVector[4] = value5;
    Chassis_Speed_Kalman.MeasuredVector[5] = value6;
}
float tmp_Velocity_Vx, tmp_Velocity_Vy, tmp_Omega;
float Ins_Accel_X_b, Ins_Accel_Y_b;
float wwx,wwy;
float H7_Offset_X = 0.0f, H7_Offset_Y = 0.0f, Distance_Offset = 0.0f;
void Class_Steering_Wheel_Chassis::Chassis_Speed_Estimate()
{
    tmp_Velocity_Vx = tmp_Velocity_Vy = tmp_Omega = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        tmp_Velocity_Vx += (Motor_Wheel[i].Get_Now_Omega_Radian() * arm_cos_f32(Motor_Steer[i].Get_Now_Zero_Offset_Radian()) * WHEEL_RADIUS) / 4.0f;
        tmp_Velocity_Vy += (Motor_Wheel[i].Get_Now_Omega_Radian() * arm_sin_f32(Motor_Steer[i].Get_Now_Zero_Offset_Radian()) * WHEEL_RADIUS) / 4.0f;
        tmp_Omega += (Motor_Wheel[i].Get_Now_Omega_Radian() * arm_cos_f32(Wheel_Azimuth[i] - Motor_Steer[i].Get_Now_Zero_Offset_Radian()) * WHEEL_RADIUS / R_DIST) / 4.0f;
    }
    Ins_Accel_X_b = -IMU->Get_Accel_X_b(); // 与imu实际放置方式有关
    Ins_Accel_Y_b = -IMU->Get_Accel_Y_b();
    float offset_angle = atan2f(H7_Offset_X, H7_Offset_Y);
    arm_sqrt_f32(H7_Offset_X * H7_Offset_X + H7_Offset_Y * H7_Offset_Y, &Distance_Offset);

    Ins_Accel_X_b = Ins_Accel_X_b - Distance_Offset * IMU->Get_Gyro_Yaw() * IMU->Get_Gyro_Yaw() * arm_cos_f32(offset_angle);
    Ins_Accel_Y_b = Ins_Accel_Y_b - Distance_Offset * IMU->Get_Gyro_Yaw() * IMU->Get_Gyro_Yaw() * arm_sin_f32(offset_angle);//这啥？消除喵板斜放向心力的影响？

    float delta_angle;
    float Chassis_Angle;
    Chassis_Angle = Motor_Yaw->Get_Now_Radian();
    delta_angle = (Reference_Radian - Chassis_Angle);
    if (Chassis_Control_Type== Chassis_Control_Type_Drive)
    {
        delta_angle = (delta_angle + 20.5f * PI / 180.0f);//咋测出20.5的，不知道
    }
    delta_angle = delta_angle < 0 ? (delta_angle + 2 * PI) : delta_angle;

    float tmp_ax = Ins_Accel_X_b;
    float tmp_ay = Ins_Accel_Y_b;

    Ins_Accel_X_b = tmp_ax * arm_cos_f32(delta_angle) - tmp_ay * arm_sin_f32(delta_angle);
    Ins_Accel_Y_b = tmp_ax * arm_sin_f32(delta_angle) + tmp_ay * arm_cos_f32(delta_angle);

    wwx = Ins_Accel_X_b;
    wwy = Ins_Accel_Y_b;

    float tmp_vx = tmp_Velocity_Vx;
    float tmp_vy = tmp_Velocity_Vy;

    tmp_Velocity_Vx = tmp_vx * arm_cos_f32(delta_angle) - tmp_vy * arm_sin_f32(delta_angle);
    tmp_Velocity_Vy = tmp_vx * arm_sin_f32(delta_angle) + tmp_vy * arm_cos_f32(delta_angle);
    //注意数据单位
    Set_Chassis_Kalman_Measure(tmp_Velocity_Vx, tmp_Velocity_Vy, 0.0f, 0.0f, tmp_Omega, IMU->Get_Gyro_Yaw());
    
    Kalman_Filter_Update(&Chassis_Speed_Kalman, NULL);

    Now_Velocity_X = Chassis_Speed_Kalman.FilteredValue[0];
    Now_Velocity_Y = Chassis_Speed_Kalman.FilteredValue[1];
    Now_Omega      = Chassis_Speed_Kalman.FilteredValue[4];
}

void Class_Steering_Wheel_Chassis::Stree_Angle_Resolution()
{
    //自锁和速度解算一样
    static uint32_t Lock_Time = 0;
    static uint8_t Lock_Flag = 0;
    float delta_Angle = 0.0f, Transform_Radian = 0.0f; // 用于优化处理的变量
    if (fabs(Target_Velocity_X) < 0.01 && fabs(Target_Velocity_Y) < 0.01 && fabs(Target_Omega) < 0.01)
    {
        Lock_Time++;
        if (Lock_Time > 3000)
            Lock_Flag = 1;
        if (Lock_Flag)
        {
            for (int i = 0; i < 4; i++)
            {
                Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE); // 舵轮控制模式
            }

            Motor_Steer[0].Set_Target_Radian(-PI / 4.0f);
            Motor_Steer[1].Set_Target_Radian(PI / 4.0f);
            Motor_Steer[2].Set_Target_Radian(-PI / 4.0f);
            Motor_Steer[3].Set_Target_Radian(PI / 4.0f);
            for (int i = 0; i < 4; i++)
            {
                Transform_Radian = Motor_Steer[i].Get_Now_Zero_Offset_Radian();

                // 优劣弧处理
                if ((i % 2) == 0)
                {
                    delta_Angle = -PI / 4.0f - Transform_Radian;
                }
                else
                {
                    delta_Angle = PI / 4.0f - Transform_Radian;
                }

                delta_Angle = Normalize_Angle_Radian_PI_to_PI(delta_Angle); // 将角度限制在-PI到PI之间

                if (delta_Angle > PI / 2.0f)
                {
                    delta_Angle = delta_Angle - PI;
                }
                else if (delta_Angle < -PI / 2.0f)
                {
                    delta_Angle = delta_Angle + PI;
                }
                Motor_Steer[i].Set_Target_Radian(Transform_Radian + delta_Angle);
                Motor_Steer[i].Set_Transform_Radian(Transform_Radian);
                Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
                Target_Wheel_Omega[i] = 0.0f;
            }
            return;
        }
    }
    else
    {
        Lock_Time = 0;
    }

    if (Lock_Flag)
    {
        Lock_Flag = 0;
    }
    float tmp_target_angle[4];
    float True_Vx[4], True_Vy[4], True_Target_Angle_Radian[4];

    float delta_angle;
    float Chassis_Angle;
    Chassis_Angle = Motor_Yaw->Get_Now_Radian();
    delta_angle = -(Reference_Radian - Chassis_Angle);
    if (Chassis_Control_Type== Chassis_Control_Type_Drive)
    {
        delta_angle = (delta_angle - 20.5f * PI / 180.0f);
    }
    delta_angle = delta_angle < 0 ? (delta_angle + 2 * PI) : delta_angle;

    float tmp_tx,tmp_ty;

    tmp_tx = Target_Velocity_X * arm_cos_f32(delta_angle) - Target_Velocity_Y * arm_sin_f32(delta_angle);
    tmp_ty = Target_Velocity_X * arm_sin_f32(delta_angle) + Target_Velocity_Y * arm_cos_f32(delta_angle);

    if (Chassis_Control_Type == Chassis_Control_Type_Drive)
    {
        True_Vx[0] = True_Vx[1] = tmp_tx - cosf(PI / 4.f) * Target_Drive_Omega * R_DIST;
        True_Vx[2] = True_Vx[3] = tmp_tx + cosf(PI / 4.f) * Target_Drive_Omega * R_DIST;

        True_Vy[0] = True_Vy[3] = tmp_ty + sinf(PI / 4.f) * Target_Drive_Omega * R_DIST;
        True_Vy[1] = True_Vy[2] = tmp_ty - sinf(PI / 4.f) * Target_Drive_Omega * R_DIST;
    }
    else
    {
        True_Vx[0] = True_Vx[1] = tmp_tx - cosf(PI / 4.f) * Target_Omega * R_DIST;
        True_Vx[2] = True_Vx[3] = tmp_tx + cosf(PI / 4.f) * Target_Omega * R_DIST;

        True_Vy[0] = True_Vy[3] = tmp_ty + sinf(PI / 4.f) * Target_Omega * R_DIST;
        True_Vy[1] = True_Vy[2] = tmp_ty - sinf(PI / 4.f) * Target_Omega * R_DIST;
    }

    // 舵轮转动角度的优化处理
    for (int i = 0; i < 4; i++)
    {
        Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE); // 舵轮控制模式

        // 计算速度
        float temp_Target_Omega = 0.0f;
        arm_sqrt_f32(True_Vx[i] * True_Vx[i] + True_Vy[i] * True_Vy[i], &temp_Target_Omega);
        temp_Target_Omega = temp_Target_Omega / WHEEL_RADIUS;

        // 计算目标角度
        if (fabs(temp_Target_Omega) < 0.0001f)
        {
            True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian();
        }
        else
        {
            True_Target_Angle_Radian[i] = atan2f(True_Vy[i], True_Vx[i]); //-PI -- PI   会自动处理Vx = 0;
            tmp_target_angle[i] = True_Target_Angle_Radian[i];
        }

        float delta_Angle;
        // 角度优化处理         180度内最短路径选择   不是选优劣弧   已经顺便处理了跳变点
        delta_Angle = True_Target_Angle_Radian[i] - Motor_Steer[i].Get_Now_Zero_Offset_Radian(); // -2PI -- 2PI
        delta_Angle = Normalize_Angle_Radian_PI_to_PI(delta_Angle);                              // 处理重叠的角度（-20 = 340），归一化到 -PI --- PI
        if (delta_Angle > PI / 2.0f)
        {
            delta_Angle = delta_Angle - PI;
            temp_Target_Omega *= -1.0f;
        }
        else if (delta_Angle < -PI / 2.0f)
        {
            delta_Angle = delta_Angle + PI;
            temp_Target_Omega *= -1.0f;
        }

        True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle;
        //True_Target_Angle_Radian[i] = Normalize_Angle_Radian_PI_to_PI(delta_Angle + Motor_Steer[i].Get_Now_Zero_Offset_Radian()); // 归一化到 -PI --- PI
        Motor_Steer[i].Set_Target_Radian(True_Target_Angle_Radian[i]);
        Motor_Steer[i].Set_Transform_Radian(Motor_Steer[i].Get_Now_Zero_Offset_Radian());
        Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
        Target_Wheel_Omega[i] = temp_Target_Omega;
    }
}

float Fx,Fy,bbb;
void Class_Steering_Wheel_Chassis::Force_Speed_Resolution()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        // 底盘失能
        for (int i = 0; i < 4; i++)
        {
            PID_Velocity_X.Set_Integral_Error(0.0f);
            PID_Velocity_Y.Set_Integral_Error(0.0f);
            PID_Omega.Set_Integral_Error(0.0f);
        }

        for(int i = 0; i < 4;i++){
            Motor_Wheel[i].Disable();
            Motor_Steer[i].Disable();
        }

        break;
    }
    case (Chassis_Control_Type_FLLOW):
    case (Chassis_Control_Type_SPIN_Positive):
    {

        PID_Velocity_X.Set_Target(Slope_Velocity_X.Get_Out());
        PID_Velocity_X.Set_Now(Now_Velocity_X);
        PID_Velocity_X.TIM_Adjust_PeriodElapsedCallback();

        PID_Velocity_Y.Set_Target(Slope_Velocity_Y.Get_Out());
        PID_Velocity_Y.Set_Now(Now_Velocity_Y);
        PID_Velocity_Y.TIM_Adjust_PeriodElapsedCallback();

        PID_Omega.Set_Target(Target_Omega);
        PID_Omega.Set_Now(Now_Omega);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        float force_x, force_y, torque_omega;

        force_x = PID_Velocity_X.Get_Out();
        force_y = PID_Velocity_Y.Get_Out();
        torque_omega = PID_Omega.Get_Out();

        float delta_angle;
        float Chassis_Angle;
        Chassis_Angle = Motor_Yaw->Get_Now_Radian();
        delta_angle = -(Reference_Radian - Chassis_Angle);
        if (Chassis_Control_Type == Chassis_Control_Type_Drive)
        {
            delta_angle = (delta_angle - 20.5f * PI / 180.0f);
        }
        delta_angle = delta_angle < 0 ? (delta_angle + 2 * PI) : delta_angle;

        float tmp_force_x = force_x;
        float tmp_force_y = force_y;

        force_x = tmp_force_x * arm_cos_f32(delta_angle) - tmp_force_y * arm_sin_f32(delta_angle);
        force_y = tmp_force_x * arm_sin_f32(delta_angle) + tmp_force_y * arm_cos_f32(delta_angle);

        Fx = force_x;
        Fy = force_y;
        bbb = delta_angle;

        // 每个轮的扭力
        float tmp_force[4];
        for (int i = 0; i < 4; i++)
        {
            // 解算到每个轮组的具体摩擦力
            tmp_force[i] = force_x * arm_cos_f32(Motor_Steer[i].Get_Now_Zero_Offset_Radian()) + force_y * arm_sin_f32(Motor_Steer[i].Get_Now_Zero_Offset_Radian()) + torque_omega / R_DIST * arm_cos_f32(Wheel_Azimuth[i] - Motor_Steer[i].Get_Now_Zero_Offset_Radian());
        }
        //改了一下防打滑的部分，不知道对不对。。。
        // 单轮打滑检测与处理
        float chassis_slip_ratio = 0.0f;
        for (int i = 0; i < 4; i++)
        {
            float target_omega = fabs(Target_Wheel_Omega[i]);
            float actual_omega = fabs(Motor_Wheel[i].Get_Now_Omega_Radian());
            
            if (target_omega > 0.1f)
            {
                float slip_ratio = actual_omega / target_omega;
                
                if (slip_ratio > Slip_Detection_Threshold)
                {
                    Slip_Time[i]++;
                    if (Slip_Time[i] > Slip_Confirm_Time)
                    {
                        Slip_Flag[i] = 1;
                        Slip_Factor[i] = Slip_Factor[i] * Slip_Damping_Factor + 0.0f;//感觉可能太小了，加个东西
                        Slip_Factor[i] = (Slip_Factor[i] > Slip_Factor_Max) ? Slip_Factor_Max : Slip_Factor[i];
                    }
                }
                else
                {
                    Slip_Time[i] = 0;
                    Slip_Factor[i] = Slip_Factor[i] * Slip_Factor_Decay;
                    Slip_Factor[i] = (Slip_Factor[i] < Slip_Factor_Min) ? Slip_Factor_Min : Slip_Factor[i];
                    if (Slip_Factor[i] < 0.1f)
                        Slip_Flag[i] = 0;
                }
                
                chassis_slip_ratio += slip_ratio;
            }
            else
            {
                Slip_Time[i] = 0;
                Slip_Factor[i] = Slip_Factor[i] * Slip_Factor_Decay;
                Slip_Factor[i] = (Slip_Factor[i] < Slip_Factor_Min) ? Slip_Factor_Min : Slip_Factor[i];
            }
        }

        // 底盘打滑检测
        chassis_slip_ratio /= 4.0f;
        float chassis_slip_damping = 1.0f;
        if (chassis_slip_ratio > Chassis_Slip_Threshold)
        {
            chassis_slip_damping = 1.0f / (1.0f + (chassis_slip_ratio - Chassis_Slip_Threshold) * Chassis_Slip_Damping);
        }

        for (int i = 0; i < 4; i++)
        {
            // 摩擦力转换至扭矩 + 打滑阻尼
            float slip_damping = (Slip_Flag[i]) ? Slip_Factor[i] : Wheel_Speed_Limit_Factor;
            Target_Wheel_Torque[i] = tmp_force[i] * WHEEL_RADIUS * chassis_slip_damping + slip_damping * (Target_Wheel_Omega[i] - Motor_Wheel[i].Get_Now_Omega_Radian());
            // 动摩擦阻力前馈
            if (Target_Wheel_Omega[i] > Wheel_Resistance_Omega_Threshold)
            {
                Target_Wheel_Torque[i] += Dynamic_Resistance_Wheel_Current[i];
            }
            else if (Target_Wheel_Omega[i] < -Wheel_Resistance_Omega_Threshold)
            {
                Target_Wheel_Torque[i] -= Dynamic_Resistance_Wheel_Current[i];
            }
            else
            {
                Target_Wheel_Torque[i] += Motor_Wheel[i].Get_Now_Omega_Radian() / Wheel_Resistance_Omega_Threshold * Dynamic_Resistance_Wheel_Current[i];
            }
            
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
            Motor_Wheel[i].Set_Target_Torque(Target_Wheel_Torque[i]);
            //Motor_Wheel[i].Set_Target_Torque(0.0f);
            Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
        }
        break;
    }
    case (Chassis_Control_Type_Drive):
    {
        PID_Velocity_X.Set_Target(Slope_Velocity_X.Get_Out());
        PID_Velocity_X.Set_Now(Now_Velocity_X);
        PID_Velocity_X.TIM_Adjust_PeriodElapsedCallback();

        PID_Velocity_Y.Set_Target(Slope_Velocity_Y.Get_Out());
        PID_Velocity_Y.Set_Now(Now_Velocity_Y);
        PID_Velocity_Y.TIM_Adjust_PeriodElapsedCallback();

        PID_Omega.Set_Target(Target_Drive_Omega);
        PID_Omega.Set_Now(Now_Omega);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        float force_x, force_y, torque_omega;

        force_x = PID_Velocity_X.Get_Out();
        force_y = PID_Velocity_Y.Get_Out();
        torque_omega = PID_Omega.Get_Out();

        float derta_angle;
        float Chassis_Angle;
        Chassis_Angle = Motor_Yaw->Get_Now_Radian();
        derta_angle = -(Reference_Radian - Chassis_Angle);
        if (Chassis_Control_Type == Chassis_Control_Type_Drive)
        {
            derta_angle = (derta_angle - 20.5f * PI / 180.0f);
        }
        derta_angle = derta_angle < 0 ? (derta_angle + 2 * PI) : derta_angle;

        float tmp_force_x = force_x;
        float tmp_force_y = force_y;

        force_x = tmp_force_x * arm_cos_f32(derta_angle) - tmp_force_y * arm_sin_f32(derta_angle);
        force_y = tmp_force_x * arm_sin_f32(derta_angle) + tmp_force_y * arm_cos_f32(derta_angle);

        Fx = force_x;
        Fy = force_y;
        bbb = derta_angle;

        // 每个轮的扭力
        float tmp_force[4];
        for (int i = 0; i < 4; i++)
        {
            // 解算到每个轮组的具体摩擦力
            tmp_force[i] = force_x * arm_cos_f32(Motor_Steer[i].Get_Now_Zero_Offset_Radian()) + force_y * arm_sin_f32(Motor_Steer[i].Get_Now_Zero_Offset_Radian()) + torque_omega / R_DIST * arm_cos_f32(Wheel_Azimuth[i] - Motor_Steer[i].Get_Now_Zero_Offset_Radian());
        }

        // 单轮打滑检测与处理
        //改了一下防打滑的代码，不知道对不对。。。
        float chassis_slip_ratio = 0.0f;
        for (int i = 0; i < 4; i++)
        {
            float target_omega = fabs(Target_Wheel_Omega[i]);
            float actual_omega = fabs(Motor_Wheel[i].Get_Now_Omega_Radian());
            
            if (target_omega > 0.1f)
            {
                float slip_ratio = actual_omega / target_omega;
                
                if (slip_ratio > Slip_Detection_Threshold)
                {
                    Slip_Time[i]++;
                    if (Slip_Time[i] > Slip_Confirm_Time)
                    {
                        Slip_Flag[i] = 1;
                        Slip_Factor[i] = Slip_Factor[i] * Slip_Damping_Factor + 10.0f;
                        Slip_Factor[i] = (Slip_Factor[i] > Slip_Factor_Max) ? Slip_Factor_Max : Slip_Factor[i];
                    }
                }
                else
                {
                    Slip_Time[i] = 0;
                    Slip_Factor[i] = Slip_Factor[i] * Slip_Factor_Decay;
                    Slip_Factor[i] = (Slip_Factor[i] < Slip_Factor_Min) ? Slip_Factor_Min : Slip_Factor[i];
                    if (Slip_Factor[i] < 0.1f)
                        Slip_Flag[i] = 0;
                }
                
                chassis_slip_ratio += slip_ratio;
            }
            else
            {
                Slip_Time[i] = 0;
                Slip_Factor[i] = Slip_Factor[i] * Slip_Factor_Decay;
                Slip_Factor[i] = (Slip_Factor[i] < Slip_Factor_Min) ? Slip_Factor_Min : Slip_Factor[i];
            }
        }

        // 底盘打滑检测
        chassis_slip_ratio /= 4.0f;
        float chassis_slip_damping = 1.0f;
        if (chassis_slip_ratio > Chassis_Slip_Threshold)
        {
            chassis_slip_damping = 1.0f / (1.0f + (chassis_slip_ratio - Chassis_Slip_Threshold) * Chassis_Slip_Damping);
        }

        for (int i = 0; i < 4; i++)
        {
            // 摩擦力转换至扭矩 + 打滑阻尼
            float slip_damping = (Slip_Flag[i]) ? Slip_Factor[i] : Wheel_Speed_Limit_Factor;
            Target_Wheel_Torque[i] = tmp_force[i] * WHEEL_RADIUS * chassis_slip_damping + slip_damping * (Target_Wheel_Omega[i] - Motor_Wheel[i].Get_Now_Omega_Radian());
            // 动摩擦阻力前馈
            if (Target_Wheel_Omega[i] > Wheel_Resistance_Omega_Threshold)
            {
                Target_Wheel_Torque[i] += Dynamic_Resistance_Wheel_Current[i];
            }
            else if (Target_Wheel_Omega[i] < -Wheel_Resistance_Omega_Threshold)
            {
                Target_Wheel_Torque[i] -= Dynamic_Resistance_Wheel_Current[i];
            }
            else
            {
                Target_Wheel_Torque[i] += Motor_Wheel[i].Get_Now_Omega_Radian() / Wheel_Resistance_Omega_Threshold * Dynamic_Resistance_Wheel_Current[i];
            }
            
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
            Motor_Wheel[i].Set_Target_Torque(Target_Wheel_Torque[i]);
            //Motor_Wheel[i].Set_Target_Torque(0.0f);
            Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
        }
        break;
    }
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
uint32_t _cntm = 0;
float aaa = 0;
float Max_Power_test = 70.0f;
float Chassis_Buffer = 0.0;
float a,b,c;
void Class_Steering_Wheel_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
    //斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
    
    //速度逆解算
    //Speed_Resolution();

    DWT_GetDeltaT(&_cntm);
    Chassis_Speed_Estimate();
    aaa = DWT_GetDeltaT(&_cntm);
    Stree_Angle_Resolution();
    Force_Speed_Resolution();

    /***************************超级电容*********************************/

    //超点直接用的原来的代码
#ifdef POWER_LIMIT_JH
    static uint8_t supercap_flag = 0;                   //超电能量低于50J的标志位
    
    //计算限制功率
    if (Referee->Get_Referee_Status() == Referee_Status_ENABLE)
    {
        // 缓冲环限制功率
        Power_Management.Buffer_Power = 0.0f;    //Referee->Get_Chassis_Energy_Buffer() - 30.0f; 
        // Power_Management.Buffer_Power = (Referee->Get_Chassis_Energy_Buffer() - 30.0f) * 1.5f;
        Math_Constrain(&Power_Management.Buffer_Power, -50.0f, 30.0f);

        if (Supercap.Get_Supercap_Status() != Supercap_Status_DISABLE && __Sprint_Status == Sprint_Status_ENABLE)
        {
            Power_Management.Max_Power = Supercap.Get_Buffer_Power() * 0.9f + Power_Management.Buffer_Power + Referee->Get_Chassis_Power_Max();
            if(Supercap.Get_Buffer_Power() <= 60.f)
            {
                Power_Management.Max_Power = Referee->Get_Chassis_Power_Max(); 
            }
        }
        else
        {
            // Power_Management.Max_Power = Power_Management.Buffer_Power + Referee->Get_Chassis_Power_Max();
            supercap_flag = 0;
            Power_Management.Max_Power = Referee->Get_Chassis_Power_Max();              //不吃缓冲能量
        }
    }
    else{
        //裁判系统离线限制功率
        Power_Management.Max_Power = Max_Power_test;
        Chassis_Buffer = 0.0f;
    }

    #ifdef AGV
    for (int i = 0; i < 4; i++)         //数据传递处理
    {
        //都是计算转子的
        // Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Wheel[i].Get_Gearbox_Rate();
        Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[i].Get_Now_Omega_Radian() * RAD_TO_RPM * M3508_REDUCTION_RATIO;
        Power_Management.Motor_Data[i].feedback_torque = Motor_Wheel[i].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;     //与减速比有关
        Power_Management.Motor_Data[i].torque = Motor_Wheel[i].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;                     //与减速比有关
        Power_Management.Motor_Data[i].pid_output = Motor_Wheel[i].Get_Out();

        Power_Management.Motor_Data[i + 4].feedback_omega  = Motor_Steer[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Steer[i].Get_Gearbox_Rate();
        Power_Management.Motor_Data[i + 4].feedback_torque = Motor_Steer[i].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
        Power_Management.Motor_Data[i + 4].torque          = Motor_Steer[i].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
        Power_Management.Motor_Data[i + 4].pid_output      = Motor_Steer[i].Get_Out();  
    }

    Power_Limit.Power_Task(Power_Management);

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].Set_Out(Power_Management.Motor_Data[i].output);
        // //Motor_Wheel[i].Output();

        Motor_Steer[i].Set_Out(Power_Management.Motor_Data[i + 4].output);//set_out已经有output输出
        //Motor_Steer[i].Output();
    }

    // if(Referee->Get_Referee_Status() == Referee_Status_ENABLE){
    //     Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() + Power_Management.Buffer_Power);
    // }
    // else{
    //     Supercap.Set_Limit_Power(90.0f);
    // }

    Supercap.Set_Supercap_Mode(Supercap_ENABLE);
    //Supercap.Set_Limit_Power(Power_Management.Max_Power);               //这样子是优先使用的缓冲功率
    Supercap.Set_Limit_Power((float)Referee->Get_Chassis_Power_Max());
    Supercap.Set_Referee_Limit_Power((uint8_t)Referee->Get_Chassis_Power_Max());
    Supercap.Set_Referee_Buffer_Power(Referee->Get_Chassis_Energy_Buffer());
    Supercap.TIM_Supercap_PeriodElapsedCallback();          //向超电发送信息
    #endif
    if (Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
    {
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Wheel[i].PID_Omega.Set_Integral_Error(0.0f);
            Motor_Wheel[i].Set_Out(0.0f);

            Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Steer[i].PID_Omega.Set_Integral_Error(0.0f);
            Motor_Steer[i].PID_Angle.Set_Integral_Error(0.0f);
            Motor_Steer[i].Set_Out(0.0f);
        }
    }
#endif	
		
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
