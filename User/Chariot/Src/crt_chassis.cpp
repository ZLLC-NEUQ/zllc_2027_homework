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
    Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x204);
}

/**
 * @brief 底盘初始化
 *
 * @param __Speed 底盘速度限制最大值
 */
void Class_HybridTrackLeg_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max)
{
    // 超级电容初始化

    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;

    // 斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.005f, 0.01f);
    // 斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.005f, 0.01f);
    //  斜坡函数加减速角度
    Slope_Position.Init(0.0014f, 0.0028f);
    // 斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

    // imu初始化
    // BoardDM_BMI.Init();
    // 过热检测状态机初始化
    //FSM_OverHeated_Detect.Chassis = this;
    //FSM_OverHeated_Detect.Init(2, 0);

    // 轮向电机PID初始化
    Motor_Wheel[0].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[0].Get_Output_Max(), Motor_Wheel[0].Get_Output_Max());
    Motor_Wheel[1].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[1].Get_Output_Max(), Motor_Wheel[1].Get_Output_Max());
    Motor_Wheel[2].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[2].Get_Output_Max(), Motor_Wheel[2].Get_Output_Max());
    Motor_Wheel[3].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[3].Get_Output_Max(), Motor_Wheel[3].Get_Output_Max());
    // 轮向电机ID初始化
    Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x204);

    // 关节电机PID初始化
    // DM速度位置控制模式Kp、Ki参数需要用上位机调节
    //Motor_Leg[0].PID_Posture.Init(1.0f, 0.0f, 0.0f, 0.0f);
    //Motor_Leg[1].PID_Posture.Init(1.0f, 0.0f, 0.0f, 0.0f);
    // 关节电机ID初始化
    //Motor_Joint[0].Init(&hfdcan2, DM_Motor_ID_0xA1, DM_Motor_Control_Method_POSITION_OMEGA, 0, PI, 20.94359f, 20.0f);
    //Motor_Joint[1].Init(&hfdcan2, DM_Motor_ID_0xA2, DM_Motor_Control_Method_POSITION_OMEGA, 0, PI, 20.94359f, 20.0f);
    //Motor_Leg[0].Init(&hfdcan2, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_POSITION, 0, PI, 20.94359f, 20.0f);
    //Motor_Leg[1].Init(&hfdcan2, DM_Motor_ID_0xA2, DM_Motor_Control_Method_MIT_POSITION, 0, PI, 20.94359f, 20.0f);

    // 履带驱动电机PID初始化
    // Motor_Track[0].PID_Omega.Init(1500.0f, 50.0f, 0.0f, 0.0f, Motor_Track[0].Get_Output_Max(), Motor_Track[0].Get_Output_Max());
    // Motor_Track[1].PID_Omega.Init(1250.0f, 50.0f, 0.0f, 0.0f, Motor_Track[1].Get_Output_Max(), Motor_Track[1].Get_Output_Max()); // 需调参
    // // 履带电机ID初始化
    // Motor_Track[0].Init(&hfdcan2, DJI_Motor_ID_0x201);
    // Motor_Track[1].Init(&hfdcan2, DJI_Motor_ID_0x202);

    // // 底部导轮电机PID初始化
    // Motor_Guider[0].PID_Omega.Init(650.0f, 10.0f, 0.0f, 0.0f, Motor_Guider[0].Get_Output_Max(), Motor_Guider[0].Get_Output_Max());
    // Motor_Guider[1].PID_Omega.Init(700.0f, 10.0f, 0.0f, 0.0f, Motor_Guider[1].Get_Output_Max(), Motor_Guider[1].Get_Output_Max()); // 需调参
    // // 底部导轮电机ID初始化
    // Motor_Guider[0].Init(&hfdcan2, DJI_Motor_ID_0x203);
    // Motor_Guider[1].Init(&hfdcan2, DJI_Motor_ID_0x204);

    // 底盘控制方式初始化
    Chassis_Control_Type = Chassis_Control_Type_DISABLE;
}

/**
 * @brief 速度解算
 *
 */
float temp_test_1, temp_test_2, temp_test_3, temp_test_4;
void Class_Tricycle_Chassis::Speed_Resolution()
{
    // 获取当前速度值，用于速度解算初始值获取
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        // 底盘失能 四轮子无力
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Disable();
        }
    }
    break;
    case (Chassis_Control_Type_SPIN):
    case (Chassis_Control_Type_FLLOW):
    {
        // 底盘四电机模式配置
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

#ifdef SPEED_SLOPE
        // 速度换算，正运动学分解
        float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        float motor2_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        float motor3_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        float motor4_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
#else
        // 速度换算，正运动学分解
        float motor1_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X + Target_Omega * (HALF_WIDTH + HALF_LENGTH);
        float motor2_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X - Target_Omega * (HALF_WIDTH + HALF_LENGTH);
        float motor3_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X + Target_Omega * (HALF_WIDTH + HALF_LENGTH);
        float motor4_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X - Target_Omega * (HALF_WIDTH + HALF_LENGTH);
#endif
        // 线速度 cm/s  转角速度  RAD
        float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
        float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
        float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;
        float motor4_temp_rad = motor4_temp_linear_vel * VEL2RAD;
        // 角速度*减速比  设定目标 直接给到电机输出轴
        Motor_Wheel[0].Set_Target_Omega_Radian(motor2_temp_rad);
        Motor_Wheel[1].Set_Target_Omega_Radian(-motor1_temp_rad);
        Motor_Wheel[2].Set_Target_Omega_Radian(-motor3_temp_rad);
        Motor_Wheel[3].Set_Target_Omega_Radian(motor4_temp_rad);
        // 各个电机具体PID
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
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
#ifdef SPEED_SLOPE

    // 斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

#endif
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
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_HybridTrackLeg_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
#ifdef SPEED_SLOPE
    // 斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

#endif
    // 过温保护
    // FSM_OverHeated_Detect.Reload_TIM_Status_PeriodElapsedCallback();
    // Chassis_SglRound_Angle = Get_Chassis_Coordinate_System_Angle_Rad();
    // // 底盘给云台发消息
    // CAN_Chassis_Tx_Gimbal_Callback();
    // CAN_Chassis_Tx_Gimbal_Callback_1();
    // // 底盘Omega控制
    // Control_Chassis_Omega_TIM_PeriodElapsedCallback();
    Speed_Resolution();
    // 位姿切换
    //Switch_Pose();

#ifdef POWER_CONTROL
    Power_Management.Max_Power = Supercap.Get_Chassis_Device_LimitPower();

    //Power_Limit.Power_Task(Power_Management);

    for (int i = 0, j = 0; i < 4; i += 2, ++j)
    {
        Power_Management.Motor_Data[i].feedback_omega = Motor_Track[j].Get_Now_Omega_Radian() * M3508_REDUATION * RAD_TO_RPM;
        Power_Management.Motor_Data[i].feedback_torque = Motor_Track[j].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
        Power_Management.Motor_Data[i].pid_output = Motor_Track[j].Get_Out();
        Power_Management.Motor_Data[i].torque = Motor_Track[j].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
        //Motor_Track[j].Reset_Out_And_Output(Power_Management.Motor_Data[i].output);
    }
#endif
}

void Class_HybridTrackLeg_Chassis::Speed_Resolution()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Wheel[i].PID_Omega.Set_Integral_Error(0.0f);
            Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
            Motor_Wheel[i].Set_Out(0.0f);
        }
        break;
    }
    case (Chassis_Control_Type_SPIN_Positive):
    case (Chassis_Control_Type_SPIN_Negative):
    case (Chassis_Control_Type_FLLOW):
    {
        // 电机模式配置
        // 轮向电机
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
#ifdef SPEED_SLOPE
        // 速度换算，正运动学分解
        float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        float motor2_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        float motor3_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        float motor4_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
#else
        // 速度换算，正运动学分解
        float motor1_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X + Target_Omega * (HALF_WIDTH + HALF_LENGTH);
        float motor2_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X - Target_Omega * (HALF_WIDTH + HALF_LENGTH);
        float motor3_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X + Target_Omega * (HALF_WIDTH + HALF_LENGTH);
        float motor4_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X - Target_Omega * (HALF_WIDTH + HALF_LENGTH);
#endif
        // 线速度 cm/s  转角速度  RAD
        float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
        float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
        float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;
        float motor4_temp_rad = motor4_temp_linear_vel * VEL2RAD;
        // 角速度*减速比  设定目标 直接给到电机输出轴
        Motor_Wheel[0].Set_Target_Omega_Radian(motor2_temp_rad);
        Motor_Wheel[1].Set_Target_Omega_Radian(-motor1_temp_rad);
        Motor_Wheel[2].Set_Target_Omega_Radian(-motor3_temp_rad);
        Motor_Wheel[3].Set_Target_Omega_Radian(motor4_temp_rad);

        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
        }
        break;
    }
    }
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
