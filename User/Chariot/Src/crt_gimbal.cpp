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

void Class_Gimbal_Yaw_Motor_LK7025::TIM_PID_PeriodElapsedCallback()
{
    switch (LK_Motor_Control_Method)
    {
    case LK_Motor_Control_Method_IMU_ANGLE:
    {
        PID_Angle.Set_Target(Target_Angle);

        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 外环：IMU角度
            PID_Angle.Set_Now(True_Angle_Yaw);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 内环：IMU角速度
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Yaw);
        }
        else
        {
            // IMU离线时，改用电机自身反馈
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }

        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
        Set_Out(Out);
    }
    break;

    case LK_Motor_Control_Method_ANGLE:
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Data.Now_Angle);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega_Angle = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega_Angle);
        PID_Omega.Set_Now(Data.Now_Omega_Angle);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
        Set_Out(Out);
    }
    break;

    case LK_Motor_Control_Method_TORQUE:
    {
        Out = Target_Torque * Torque_Current
              / Current_Max * Current_Max_Cmd;

        Set_Out(Out);
    }
    break;

    case LK_Motor_Control_Method_OpenLoop:
    {
        // 保持当前Out，失能函数会将其清零
    }
    break;

    default:
    {
        Set_Out(0.0f);
    }
    break;
    }

    // 按LK协议把控制量写入CAN发送缓冲区
    Output();
}

void Class_Gimbal_Yaw_Motor_LK7025::Disable()
{
    Set_LK_Motor_Control_Method(
        LK_Motor_Control_Method_OpenLoop
    );

    Set_Out(0.0f);
    Output();
}

void Class_Gimbal_Yaw_Motor_LK7025::Transform_Angle()
{
    True_Rad_Yaw   = IMU->Get_Rad_Yaw();
    True_Gyro_Yaw  = IMU->Get_Gyro_Yaw();
    True_Angle_Yaw = IMU->Get_Angle_Yaw();
}

float DebugMo = 0.0f;

/**
 * @brief 大 Pitch 的 PID 计算
 */
void Class_Gimbal_Pitch_Motor_DM4310::TIM_PID_PeriodElapsedCallback()
{
    switch (DM_Motor_Control_Method)
    {
    case DM_Motor_Control_Method_MIT_IMU_Angle:
    {
        PID_Angle.Set_Target(Target_Angle);

        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 外环：使用 IMU 的 Pitch 角度
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega = PID_Angle.Get_Out();

            // 内环：使用电机自身反馈速度
            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);
        }
        else
        {
            // IMU 离线时，使用电机自身角度
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega * RAD_TO_DEG);
        }

        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        Set_Out(
            Target_Torque +
            abs(cosf(True_Rad_Pitch) * DebugMo)
        );
    }
    break;

    case DM_Motor_Control_Method_MIT_OPENLOOP:
    {
        Out = Out;
    }
    break;

    default:
    {
        Set_Out(0.0f);
    }
    break;
    }

    // 调用达妙父类的发送打包函数
    Output();
}

/**
 * @brief 失能大 Pitch
 */
void Class_Gimbal_Pitch_Motor_DM4310::Disable()
{
    Set_DM_Motor_Control_Method(
        DM_Motor_Control_Method_MIT_OPENLOOP
    );

    Set_Out(0.0f);
    Output();
}

/**
 * @brief 从 IMU 获取大 Pitch 的真实姿态
 */
void Class_Gimbal_Pitch_Motor_DM4310::Transform_Angle()
{
    True_Rad_Pitch   = -IMU->Get_Rad_Roll();
    True_Gyro_Pitch  = -IMU->Get_Gyro_Roll();
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

    // Yaw轴 LK7025
    Motor_Yaw.PID_Angle.Init(
        0.4f, 0.0f, 0.0f,
        0.0f, 200.0f, 4000.0f,
        0.0f, 0.0f, 0.0f,
        0.001f, 0.0f
    );

    Motor_Yaw.PID_Omega.Init(
        170.0f, 210.0f, 0.0f,
        0.0f, 200.0f, 4000.0f,
        0.0f, 0.0f, 0.0f,
        0.001f, 0.0f
    );

    Motor_Yaw.IMU = &Boardc_BMI;

    Motor_Yaw.Init(
        &hfdcan2,
        LK_Motor_ID_0x141,
        220.0f,
        0,
        33.0f,
        LK_Motor_Control_Method_IMU_ANGLE,
        LK_Motor_Control_Torque
    );

        // 大 Pitch：DM-J4340P
    Motor_Pitch.PID_Angle.Init(
        0.4f, 0.0f, 0.0f,
        0.0f, 200.0f, 4090.0f
    );

    Motor_Pitch.PID_Omega.Init(
        220.0f, 80.0f, 0.0f,
        0.0f, 2000.0f, 4090.0f
    );

    Motor_Pitch.IMU = &Boardc_BMI;

    Motor_Pitch.Init(
        &hfdcan2,
        DM_Motor_ID_0xA1,
        DM_Motor_Control_Method_MIT_OPENLOOP
    );


    // 小 Pitch
    Motor_Pitch_2.Init(
        &hfdcan1,
        DM_Motor_ID_0xA2,
        DM_Motor_Control_Method_POSITION_OMEGA,
        PI
    );

    Motor_Pitch_2.Set_Target_Omega(0.0f);
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
            // 立即失能大 Pitch 和 Yaw，并发送零输出
            Motor_Pitch.Disable();
            Motor_Yaw.Disable();

            // 清除 Yaw 三个 PID 的历史积分
            Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
            Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);

            // 达妙大 Pitch 只使用角度环和速度环
            Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);

            // 将目标扭矩和最终输出再次清零
            Motor_Yaw.Set_Target_Torque(0.0f);
            Motor_Pitch.Set_Target_Torque(0.0f);

            Motor_Yaw.Set_Out(0.0f);
            Motor_Pitch.Set_Out(0.0f);

            // 小 Pitch 使用位置-速度模式，失能时把目标速度设为 0
            Motor_Pitch_2.Set_Target_Omega(0.0f);
        }
    
    else // 非失能模式
    {
        Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);

        Motor_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_IMU_Angle);

        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
            Motor_Pitch_2.Set_Target_Omega(200.0f);
            Motor_Pitch_2.Set_Target_Angle(0.0f);
        }
        else if ((Gimbal_Control_Type == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            Target_Pitch_Angle = MiniPC->Get_Rx_Pitch_Angle();
            Target_Yaw_Angle = MiniPC->Get_Rx_Yaw_Angle();
        }

        // 限制角度范围 处理yaw轴180度问题
        while ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) > Max_Yaw_Angle)
        {
            Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
        }
        while ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) < -Max_Yaw_Angle)
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
        Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
        Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    // 根据当前云台模式设置各电机目标值
    Output();

    // 更新姿态角度
    Motor_Yaw.Transform_Angle();
    Motor_Pitch.Transform_Angle();
    // Yaw 和大 Pitch 执行 PID
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
    // 小 Pitch 打包并发送位置、速度控制指令
    Motor_Pitch_2.TIM_Process_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
