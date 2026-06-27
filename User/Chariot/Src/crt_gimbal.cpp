/**
 * @file crt_gimbal.cpp
 * @author lez by wanghongxi
 * @brief 云台
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float test_angle = 0;
float Test_Target_Omega = 0;
float last_angle = 0;
void Class_Gimbal_Yaw_Motor_LK::TIM_PID_PeriodElapsedCallback()
{
    switch (LK_Motor_Control_Method)
    {
        case (LK_Motor_Control_Method_TORQUE):
        {        
            PID_Torque.Set_Target(Target_Torque);
            PID_Torque.Set_Now(Data.Now_Current);
            PID_Torque.TIM_Adjust_PeriodElapsedCallback();

            Out = PID_Torque.Get_Out();
        }
        break;
        case (LK_Motor_Control_Method_OMEGA):
        {
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Get_Transform_Omega());
            PID_Omega.TIM_Adjust_PeriodElapsedCallback();

            Target_Current = PID_Omega.Get_Out();

            Out = Target_Current;
        }
        break;
        case (LK_Motor_Control_Method_ANGLE):
        {
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Get_Transform_Angle());
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Transform_Omega);
            PID_Omega.TIM_Adjust_PeriodElapsedCallback();

            Out = PID_Omega.Get_Out();         
        }
        break;
        default:
        {
            Out = 0.0f;
        }
        break;
    }    

    Output();
}

void Class_Gimbal_Yaw_Motor_LK::Disable()
{
    Set_LK_Motor_Control_Method(LK_Motor_Control_Method_OpenLoop);
    Set_Out(0.0f);
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Yaw_Motor_LK::Transform_Angle()
{
    True_Rad_Yaw = IMU->Get_Rad_Yaw();
    True_Gyro_Yaw = IMU->Get_Gyro_Yaw();
    True_Angle_Yaw = IMU->Get_Angle_Yaw();
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float test_omega = 1.0f;
float m_angle = 0.0f;
void Class_Gimbal_Pitch_Motor_J4310::TIM_PID_PeriodElapsedCallback()
{
     switch (DM_Motor_Control_Method)
    {
        case (DM_Motor_Control_Method_MIT_TORQUE):
        {
            if(DM_Motor_Control_Alg == DM_PID_Omega){
                PID_Omega.Set_Now(Transform_Omega);
                PID_Omega.Set_Target(Target_Omega);
                PID_Omega.TIM_Adjust_PeriodElapsedCallback();

                Out = PID_Omega.Get_Out();
                Output_Torque = Torque_Max / 2048.0f * Out;
            }
            else if(DM_Motor_Control_Alg == DM_PID_Angle){
                PID_Angle.Set_Now(Get_Transform_Angle());
                PID_Angle.Set_Target(Target_Angle);
                PID_Angle.TIM_Adjust_PeriodElapsedCallback();
                Target_Omega = PID_Angle.Get_Out();

                PID_Omega.Set_Now(Get_Transform_Omega());
                PID_Omega.Set_Target(Target_Omega);
                PID_Omega.TIM_Adjust_PeriodElapsedCallback();

                Out = PID_Omega.Get_Out();
                float tmp_Torque = J * Transform_Target_Acc + B * Transform_Target_Vel;  //简单的动力学补偿，参数需要根据实际负载测量后赋值
                Out += tmp_Torque / (Torque_Max / 2048.0f);

                Output_Torque = Torque_Max / 2048.0f * Out;
            }
            else if(DM_Motor_Control_Alg == DM_Motor_DISANLE){
                Out           = 0.0f;
                Output_Angle  = 0.0f;
                Output_Omega  = 0.0f;
                Output_Torque = 0.0f;
            }
            break;
        }
        default:
        {
            Out           = 0.0f;
            Output_Angle  = 0.0f;
            Output_Omega  = 0.0f;
            Output_Torque = 0.0f;
            break;
        }
    }
    Output();
}

void Class_Gimbal_Pitch_Motor_GM6020::Disable()
{
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
    Set_Out(0.0f);
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_GM6020::Transform_Angle()
{
    True_Rad_Pitch = -IMU->Get_Rad_Roll();
    True_Gyro_Pitch = -IMU->Get_Gyro_Roll();
    True_Angle_Pitch = -IMU->Get_Angle_Roll();
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Pitch_Motor_LK6010::TIM_PID_PeriodElapsedCallback()
{
    switch (LK_Motor_Control_Method)
    {
    case (LK_Motor_Control_Method_TORQUE):
    {
        Out = Target_Torque * Torque_Current / Current_Max * Current_Max_Cmd;
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_IMU_OMEGA):
    {
        // 角速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Pitch * 180.f / PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Out = PID_Omega.Get_Out();
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Pitch * 180.f / PI);
        }
        else
        {
            // 角度环
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out() + Gravity_Compensate;
        Set_Out(Out);
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_LK6010::Transform_Angle()
{
    True_Rad_Pitch = 1 * IMU->Get_Rad_Pitch();
    True_Gyro_Pitch = 1 * IMU->Get_Gyro_Pitch();
    True_Angle_Pitch = 1 * IMU->Get_Angle_Pitch();
}

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    // yaw轴电机
   Motor_Main_Yaw.PID_Angle.Init(0.28f, 0.0f, 0.0004f, 0.0f, 3, 15);
    Motor_Main_Yaw.PID_Omega.Init(2200.0f, 0.0f, 0.0f, 0.0f, 400.0f, 2048.0f);
    Motor_Main_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Main_Yaw.Get_Output_Max(), Motor_Main_Yaw.Get_Output_Max());            //这样一直输出的都是0，失能
    Motor_Main_Yaw.Init(&hfdcan2, LK_Motor_ID_0x141, LK_Motor_Control_Method_ANGLE,2048);

    // pitch轴电机
    Motor_Yaw.PID_Angle.Init(40.0f, 0.0f, 0.18f, 0.0f, 10000000, 10000000,0.0f, 0.0f, 0, 0.001f, 0.0f, PID_D_First_ENABLE);
    Motor_Yaw.PID_Omega.Init(60.0f, 1500.0f, 0.0f, 0, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max(), 0.0f, 0.0f, 0.0f, 0.001f, 0.8f);
    Motor_Yaw.PID_Torque.Init(0.8f, 100.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.IMU = &Boardc_BMI;
#ifdef DEBUG_PITCH_SPEED_LOOP
    Motor_Pitch.Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_IMU_OMEGA, 3413);
#else
    Motor_Yaw.Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_IMU_ANGLE, 3413);

#endif
}

/**
 * @brief 输出到电机
 *
 */
float temp_err = 0.0f;
float temp_target_angle = 0.0f;
void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        // 云台失能
        Motor_Pitch.Disable();
        Motor_Main_Yaw.Disable();

        Motor_Main_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Torque.Set_Integral_Error(0.0f);
    }
    else // 非失能模式
    {
        Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);

#ifdef DEBUG_PITCH_SPEED_LOOP
        Motor_Pitch.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_OMEGA);
#else
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);

#endif

        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            // 设置目标角度
            Motor_Main_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
        }
        else if ((Gimbal_Control_Type == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            Target_Pitch_Angle = MiniPC->Get_Rx_Pitch_Angle();
            Target_Yaw_Angle = MiniPC->Get_Rx_Yaw_Angle();
        }

        // 限制角度范围 处理yaw轴180度问题
        while ((Target_Yaw_Angle - Motor_Main_Yaw.Get_True_Angle_Yaw()) > Max_Yaw_Angle)
        {
            Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
        }
        while ((Target_Yaw_Angle - Motor_Main_Yaw.Get_True_Angle_Yaw()) < -Max_Yaw_Angle)
        {
            Target_Yaw_Angle += (2 * Max_Yaw_Angle);
        }

        // 新处理yaw轴180度问题
        //  1. 角度优化

        //        float temp_min;

        //        // 计算误差，考虑当前电机状态
        //        temp_err = Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw();

        //        // 标准化到[0, 360)范围
        //        while (temp_err > 360.0f)
        //            temp_err -= 360.0f;
        //        while (temp_err < 0.0f)
        //            temp_err += 360.0f;

        //        // 比较路径长度
        //        if (fabs(temp_err) < (360.0f - fabs(temp_err)))
        //            temp_min = fabs(temp_err);
        //        else
        //            temp_min = 360.0f - fabs(temp_err);

        //        // 判断是否需要切换方向
        //        // if (temp_min > 90.0f)
        //        // {
        //        //     steering_wheel->invert_flag = !steering_wheel->invert_flag;
        //        //     // 重新计算误差
        //        //     temp_err = steering_wheel->Target_Angle - steering_wheel->Now_Angle - steering_wheel->invert_flag * 180.0f;
        //        // }
        //        // 2. 优劣弧优化，实际上角度优化那里已经完成了
        //        if (temp_err > 180.0f)
        //        {
        //            temp_err -= 360.0f;
        //        }
        //        else if (temp_err < -180.0f)
        //        {
        //            temp_err += 360.0f;
        //        }

        //        temp_target_angle = Motor_Yaw.Get_True_Angle_Yaw() + temp_err;
        //        Target_Yaw_Angle = temp_target_angle;

        // pitch限位
        Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

        // 设置目标角度
        Motor_Main_Yaw.Set_Target_Angle(Target_Yaw_Angle);
        Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    Output();

    // 根据不同c板的放置方式来修改这几个函数
    Motor_Main_Yaw.Transform_Angle();
    Motor_Pitch.Transform_Angle();

    Motor_Main_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
