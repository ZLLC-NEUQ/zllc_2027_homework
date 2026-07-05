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

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
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
void Class_Tricycle_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max, float __Steer_Power_Ratio)
{
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    Steer_Power_Ratio = __Steer_Power_Ratio;

    // 斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.064f, 0.128f);
    // 斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.064f, 0.128f);
    // 斜坡函数加减速角速度
    Slope_Omega.Init(0.1f, 0.1f);

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
    Motor_Wheel[0].PID_Omega.Init(2000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[0].Get_Output_Max(), Motor_Wheel[0].Get_Output_Max());
    Motor_Wheel[1].PID_Omega.Init(2000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[1].Get_Output_Max(), Motor_Wheel[1].Get_Output_Max());
    Motor_Wheel[2].PID_Omega.Init(2000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[2].Get_Output_Max(), Motor_Wheel[2].Get_Output_Max());
    Motor_Wheel[3].PID_Omega.Init(2000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[3].Get_Output_Max(), Motor_Wheel[3].Get_Output_Max());

    // 轮向电机ID初始化
    Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OPENLOOP);
    Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OPENLOOP);
    Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OPENLOOP);
    Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_OPENLOOP);

    Motor_Steer[0].PID_Angle.Init(10.0f, 0.0f, 0.0f, 0.0f, Motor_Steer[0].Get_Output_Max(), Motor_Steer[0].Get_Output_Max());
    Motor_Steer[0].PID_Omega.Init(1000.0f,0.0f, 0.0f, 0.0f, 8000, Motor_Steer[0].Get_Output_Max());
    
    Motor_Steer[1].PID_Angle.Init(10.0f, 0.0f, 0.0f, 0.0f, Motor_Steer[1].Get_Output_Max(), Motor_Steer[1].Get_Output_Max());
    Motor_Steer[1].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[1].Get_Output_Max());

    Motor_Steer[2].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[2].Get_Output_Max(), Motor_Steer[2].Get_Output_Max());
    Motor_Steer[2].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[2].Get_Output_Max());

    Motor_Steer[3].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[3].Get_Output_Max(), Motor_Steer[3].Get_Output_Max());
    Motor_Steer[3].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[3].Get_Output_Max());


    //舵向电机ID初始化
    Motor_Steer[0].Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_AGV_MODE, 8.0f);
    Motor_Steer[1].Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_AGV_MODE, 8.0f);
    Motor_Steer[2].Init(&hfdcan1, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_AGV_MODE, 8.0f);
    Motor_Steer[3].Init(&hfdcan1, DJI_Motor_ID_0x208, DJI_Motor_Control_Method_AGV_MODE, 8.0f);

    Motor_Steer[0].Set_Zero_Position(-1.88f+3.14f);
    Motor_Steer[1].Set_Zero_Position(-1.65f+3.14f);
    Motor_Steer[2].Set_Zero_Position(-0.92f+3.14f);
    Motor_Steer[3].Set_Zero_Position(1.25f+3.14f);

    Kalman_Filter_Init(&Chassis_Speed_Kalman, 6, 0, 6);                               

    memcpy(Chassis_Speed_Kalman.F_data, Chassis_Speed_Kalman_F, sizeof(Chassis_Speed_Kalman_F));
    memcpy(Chassis_Speed_Kalman.H_data, Chassis_Speed_Kalman_H, sizeof(Chassis_Speed_Kalman_H));
    memcpy(Chassis_Speed_Kalman.P_data, Chassis_Speed_Kalman_P, sizeof(Chassis_Speed_Kalman_P));
    memcpy(Chassis_Speed_Kalman.Q_data, Chassis_Speed_Kalman_Q, sizeof(Chassis_Speed_Kalman_Q));
    memcpy(Chassis_Speed_Kalman.R_data, Chassis_Speed_Kalman_R, sizeof(Chassis_Speed_Kalman_R));

    //底盘控制方式初始化
    Chassis_Control_Type = Chassis_Control_Type_DISABLE;
}

/**
 * @brief 速度解算
 *
 */
float temp_test_1, temp_test_2, temp_test_3, temp_test_4;
float True_Vx[4], True_Vy[4];
void Class_Tricycle_Chassis::Speed_Resolution()
{
    if (Motor_Steer[0].Get_MA600_Status() == MA600_Status_DISABLE || Motor_Steer[1].Get_MA600_Status() == MA600_Status_DISABLE ||
        Motor_Steer[2].Get_MA600_Status() == MA600_Status_DISABLE || Motor_Steer[3].Get_MA600_Status() == MA600_Status_DISABLE)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
             Motor_Wheel[i].Disable();
            Motor_Steer[i].Disable();
        }
        return;
    }
    // 获取当前速度值，用于速度解算初始值获取
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        // 底盘失能 四轮子无力
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Disable();
            Motor_Steer[i].Disable();
        }
    }
    break;
    case (Chassis_Control_Type_SPIN):
    case (Chassis_Control_Type_FLLOW):
    case (Chassis_Control_Type_ANTI_SPIN):
    {
        // 底盘四电机模式配置
        static uint32_t Lock_Time = 0;
        static uint8_t Lock_Flag = 0;
        float delta_Angle = 0.0f, Transform_Radian = 0.0f; // 用于优化处理的变量
        if (fabs(Target_Velocity_X) < 0.01 && fabs(Target_Velocity_Y) < 0.01 && fabs(Target_Omega) < 0.01)
        {
            Lock_Time++;
            if (Lock_Time > 500)
                Lock_Flag = 1;
            if (Lock_Flag)
            {
                for (int i = 0; i < 4; i++)
                {
                    Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE); // 舵轮控制模式

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
                    Motor_Steer[i].Set_Target_Radian(Transform_Radian + delta_Angle);
                    Motor_Steer[i].Set_Transform_Radian(Transform_Radian);
                    Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
                    Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
                }
                break;
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

        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        }
        // 底盘限速
        if (Velocity_X_Max != 0)
        {
            Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
        }
        if (Velocity_Y_Max != 0)
        {
            Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
        }
        if (Omega_Max != 0)
        {
            Math_Constrain(&Target_Omega, -Omega_Max, Omega_Max);
        }

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
        True_Vx[0] = True_Vx[1] = Slope_Velocity_X.Get_Out() - Target_Omega * half_length;
        True_Vx[2] = True_Vx[3] = Slope_Velocity_X.Get_Out() + Target_Omega * half_length;

        True_Vy[0] = True_Vy[3] = Slope_Velocity_Y.Get_Out() + Target_Omega * half_length;
        True_Vy[1] = True_Vy[2] = Slope_Velocity_Y.Get_Out() - Target_Omega * half_length;
        // 各个电机具体PID
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
            Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
        }
    }
    break;
    }
}
//Enum_Supercap_Mode test_mode = Supercap_Mode_ENABLE;
float test_power = 58.0f;
float compensate_max_power = 30.0f;
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float Chassis_Buffer = 0.0;
float Power_Limit_K = 1.0f;
void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{


    // 斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

    // 速度解算
    Speed_Resolution();

    // float Chassis_Buffer = 0.0;
    //计算限制功率
    //if(Referee->Get_Referee_Status() == Referee_Status_ENABLE){
        //缓冲环限制功率
        Chassis_Buffer = Referee->Get_Chassis_Energy_Buffer();
        Power_Management.Buffer_Power = Referee->Get_Chassis_Energy_Buffer() - 30.0f;//(sqrt(Chassis_Buffer) - sqrt(Power_Management.Min_Buffer)) * Power_Management.Buffer_K;
        //Math_Constrain(&Power_Management.Buffer_Power, -30.0f, 0.0f);

        if (Supercap.Get_Supercap_Status() != Supercap_Status_DISABLE && __Sprint_Status == Sprint_Status_ENABLE)
        {
            Power_Management.Max_Power = Referee->Get_Chassis_Power_Max();
        }
        else
        {
            //Power_Management.Max_Power = Power_Management.Buffer_Power + Referee->Get_Chassis_Power_Max();
            Power_Management.Max_Power = Referee->Get_Chassis_Power_Max();
        }
//    }
//    else{
//        //裁判系统离线限制功率
//        Power_Management.Max_Power = 70.0f;
//        Chassis_Buffer = 0.0f;
//    }
//    
    Power_Management.Actual_Power = Supercap.Get_Chassis_Actual_Power();//Referee->Get_Chassis_Power();
    Power_Management.Total_error = 0.0f;

#ifdef AGV
    for (int i = 0; i < 4; i++)         //数据传递处理
    {
        //都是计算转子的
        Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Wheel[i].Get_Gearbox_Rate();
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
        Motor_Wheel[i].Output();

        Motor_Steer[i].Set_Out(Power_Management.Motor_Data[i + 4].output);
        Motor_Steer[i].Output();
    }
#else
    for (int i = 0; i < 4; i++)         //数据传递处理
    {
        //都是计算转子的
        Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Wheel[i].Get_Gearbox_Rate();
        Power_Management.Motor_Data[i].feedback_torque = Motor_Wheel[i].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;     //与减速比有关
        Power_Management.Motor_Data[i].torque = Motor_Wheel[i].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;                     //与减速比有关
        Power_Management.Motor_Data[i].pid_output = Motor_Wheel[i].Get_Out();

        Power_Management.Motor_Data[i].Target_error = fabs(Motor_Wheel[i].Get_Target_Omega_Radian() - Motor_Wheel[i].Get_Now_Omega_Radian());
        
    }
    Power_Management.Total_error = 0.0;
    Power_Limit.Power_Task(Power_Management);

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].Set_Out(Power_Management.Motor_Data[i].output);
        //Motor_Wheel[i].Output();
    }
#endif

    //if(Referee->Get_Referee_Status() == Referee_Status_ENABLE){
        Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() + Power_Management.Buffer_Power);


    //Supercap.TIM_Supercap_PeriodElapsedCallback();          //向超电发送信息
    float power = Supercap.Get_Limit_Power();
    memcpy(CAN_Supercap_Tx_Data,&power,4);
    uint8_t tmp = (uint8_t)SuperCap;
    memcpy(CAN_Supercap_Tx_Data+4,&tmp,1);

}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/