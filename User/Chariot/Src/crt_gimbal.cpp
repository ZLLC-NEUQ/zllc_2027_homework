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
void Class_Gimbal_Pitch_Motor_DM4310::TIM_PID_PeriodElapsedCallback()
{
    switch (DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_MIT_IMU_Angle):
    {
        PID_Angle.Set_Target(Target_Angle_Deg);

        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {

            // 角度环
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_DEG = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_DEG);
            // PID_Omega.Set_Now(True_Gyro_Pitch);
            PID_Omega.Set_Now(Kf_Gyro_Pitch.x);
        }
        else
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_DEG = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_DEG);
            PID_Omega.Set_Now(Data.Now_Omega_after_kalman);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(Target_Torque + Gravity_Compensate);
    }
    break;
    case (DM_Motor_Control_Method_MIT_OPENLOOP):
    {
        Out = Out;
    }
    break;
    case (DM_Motor_Control_Method_MIT_Encoder_Position):
    {
        // 角度环
        PID_Angle.Set_Target(Target_Angle_Deg);
        PID_Angle.Set_Now(EmcoderAngle_To_TrueAngle);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega_DEG = PID_Angle.Get_Out();

        // 速度环
        PID_Omega.Set_Target(Target_Omega_DEG);
        // PID_Omega.Set_Now(True_Gyro_Pitch);
        // PID_Omega.Set_Now(Data.Now_Omega_after_kalman);
        PID_Omega.Set_Now(Kf_Gyro_Pitch.x);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(Target_Torque + Gravity_Compensate);
    }
    break;
    default:
    {
        Set_Out(0.0);
    }
    break;
    }
    Output(); // 进入父类中进行输出
}

void Class_Gimbal_Yaw_Motor_DM4310::Disable()
{

    Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_OPENLOOP);
    Set_Out(0.0f);
    Output();
}


void Class_Gimbal_Pitch_Motor_DM4310::Disable()
{

    Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_OPENLOOP);
    Set_Out(0.0f);
    Output();
}
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Yaw_Motor_DM4310::TIM_PID_PeriodElapsedCallback()
{
    switch (DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_MIT_IMU_Angle):
    {
        PID_Angle.Set_Target(Target_Angle_Deg);
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Yaw);
            if (Target_Angle_Deg - True_Angle_Yaw > 180.0f)
            {
                PID_Angle.Set_Target(Target_Angle_Deg - 360.0f);
            }
            else if (Target_Angle_Deg - True_Angle_Yaw < -180.0f)
            {
                PID_Angle.Set_Target(Target_Angle_Deg + 360.0f);
            }
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_DEG = PID_Angle.Get_Out();
            // 速度环
            PID_Omega.Set_Target(Target_Omega_DEG);
            // PID_Omega.Set_Now(True_Gyro_Yaw);
            PID_Omega.Set_Now(Kf_Gyro_Yaw.x);
        }
        else
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_DEG = PID_Angle.Get_Out();

            //     //速度环
            PID_Omega.Set_Target(Target_Omega_DEG);
            PID_Omega.Set_Now(Data.Now_Omega_after_kalman);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Target_Torque = PID_Omega.Get_Out();
        Set_Out(Target_Torque);
    }
    break;
    case (DM_Motor_Control_Method_MIT_OPENLOOP):
    {
        Out = Out;
    }
    break;
    case (DM_Motor_Control_Method_MIT_Encoder_Position):
    {
        // 角度环
        PID_Angle.Set_Target(Target_Angle_Deg);
        PID_Angle.Set_Now(EmcoderAngle_To_TrueAngle);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega_DEG = PID_Angle.Get_Out();

        // 速度环
        PID_Omega.Set_Target(Target_Omega_DEG);
        // PID_Omega.Set_Now(True_Gyro_Yaw);
        PID_Omega.Set_Now(Kf_Gyro_Yaw.x);
        // PID_Omega.Set_Now(Data.Now_Omega_after_kalman);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(Target_Torque);
    }
    break;
    default:
    {
        Set_Out(0.0);
    }
    break;
    }
    Output(); // 进入父类中进行输出
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float test_angle = 0;
float Test_Target_Omega = 0;
float last_angle = 0;
void Class_Gimbal_Yaw_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        // 默认开环速度控制
        Out = Out;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        // 力矩环
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        // 角速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Yaw * 180.f / PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        // PID_Angle.Set_Target(Target_Angle);
        //  Target_Angle=test_angle;
        if (last_angle != Target_Angle)
        {
            PID_Angle.Set_Target(Target_Angle);
        }
        last_angle = Target_Angle;
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Yaw);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Radian = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Radian);
            PID_Omega.Set_Now(True_Gyro_Yaw * 57.3f);
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

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(-PID_Omega.Get_Out());
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

void Class_Gimbal_Yaw_Motor_GM6020::Disable()
{
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
    Set_Out(0.0f);
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Yaw_Motor_GM6020::Transform_Angle()
{
    True_Rad_Yaw = IMU->Get_Rad_Yaw();
    True_Gyro_Yaw = IMU->Get_Gyro_Yaw();
    True_Angle_Yaw = IMU->Get_Angle_Yaw();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_DM4310::Transform_Angle()
{
    // True_Angle_Pitch = -1 * IMU ->Get_DMIMU_Pitch();// 角度
    // True_Gyro_Pitch = -1 * IMU->Get_DMIMU_Gyro_Pitch() * 57.29; // 角速度
    True_Rad_Pitch = 1 * IMU->Get_Rad_Pitch();
    True_Gyro_Pitch = 1 * IMU->Get_Gyro_Pitch() * 57.29;
    True_Angle_Pitch = 1 * IMU->Get_Angle_Pitch();

    kalman_update(&Kf_Gyro_Pitch, True_Gyro_Pitch);
}


/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Yaw_Motor_DM4310::Transform_Angle()
{
    // 粗略修正YAW轴漂移
    //  Service_time = DWT_GetTimeline_us();
    True_Rad_Yaw = IMU->Get_Rad_Yaw();
    True_Gyro_Yaw = IMU->Get_Gyro_Yaw() * 57.29;
    True_Angle_Yaw = IMU->Get_Angle_Yaw();
    // True_Gyro_Yaw = IMU->Get_DMIMU_Gyro_Yaw() * 57.29;
    // True_Angle_Yaw = IMU->Get_DMIMU_Yaw() - Service_time * K;

    kalman_update(&Kf_Gyro_Yaw, True_Gyro_Yaw);
}


/**
 * @brief 将编码器角度转换为真实角度
 *
 */
void Class_Gimbal_Pitch_Motor_DM4310::Transform_EmcoderAngle_To_TrueAngle()
{
    const uint16_t ENC_AT_MIN = 31746; // 实测：角度 -15° 时的编码器值
    const uint16_t ENC_AT_MAX = 38307; // 实测：角度 +40° 时的编码器值
    const float MIN_ANGLE_DEG = -21.0f;
    const float MAX_ANGLE_DEG = 43.7f;

    uint16_t encoder_raw = Get_Now_Encoder_Position();

    // 线性插值
    float angle_deg = MIN_ANGLE_DEG + (float)(encoder_raw - ENC_AT_MIN) / (ENC_AT_MAX - ENC_AT_MIN) * (MAX_ANGLE_DEG - MIN_ANGLE_DEG);

    // 限幅（防止因编码器噪声或超出量程）
    if (angle_deg < MIN_ANGLE_DEG)
        angle_deg = MIN_ANGLE_DEG;
    if (angle_deg > MAX_ANGLE_DEG)
        angle_deg = MAX_ANGLE_DEG;

    EmcoderAngle_To_TrueAngle = angle_deg;
}

/**
 * @brief 将编码器角度转换为真实角度
 *
 */
void Class_Gimbal_Yaw_Motor_DM4310::Transform_EmcoderAngle_To_TrueAngle()
{
    const float Reference_Angle = 1.2372514f;  // 零位校准值（弧度）
    const float ENCODER_RESOLUTION = 65535.0f; // 单圈分辨率

    // 原始机械角度 [0, 2π)
    float raw_angle_rad = (float)(Get_Now_Encoder_Position() / ENCODER_RESOLUTION) * 2.0f * PI;

    // 减去参考角度实现校准
    float calibrated_angle = raw_angle_rad - Reference_Angle;

    // 归一化到 [-π, π] 范围
    calibrated_angle = fmodf(calibrated_angle, 2.0f * PI);
    if (calibrated_angle > PI)
        calibrated_angle -= 2.0f * PI;
    else if (calibrated_angle < -PI)
        calibrated_angle += 2.0f * PI;

    // 转换为度数，并根据安装方向决定是否取反
    EmcoderAngle_To_TrueAngle = calibrated_angle / PI * 180.0f;
}


/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float test_omega = 1.0f;
float m_angle = 0.0f;
void Class_Gimbal_Pitch_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        // 默认开环
        Out = Out;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        // 力矩环
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        // 角速度环

        //			if(True_Angle_Pitch>=15){
        //			Target_Omega_Angle=-test_omega;
        //			}
        //			if(True_Angle_Pitch<=-15){
        //				Target_Omega_Angle=test_omega;
        //			}

        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Pitch * 180.f / PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
//        // PID_Angle.Set_Target(-m_angle);
//        PID_Angle.Set_Target(-Target_Angle);
//        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
//        {
//            // 角度环
//            PID_Angle.Set_Now(True_Angle_Pitch);
//            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

//            Target_Omega_Angle = PID_Angle.Get_Out();

//            // 速度环
//            PID_Omega.Set_Target(Target_Omega_Angle);
//            PID_Omega.Set_Now(True_Gyro_Pitch * 57.3);
//        }
//        else
//        {
//            // 角度环
//            PID_Angle.Set_Now(Data.Now_Angle);
//            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

//            Target_Omega_Angle = PID_Angle.Get_Out();

//            // 速度环
//            PID_Omega.Set_Target(Target_Omega_Angle);
//            PID_Omega.Set_Now(Data.Now_Omega_Angle);
//        }
//        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

//        Target_Torque = -PID_Omega.Get_Out();
//        Set_Out(-PID_Omega.Get_Out() + Gravity_Compensate);
        // PID_Angle.Set_Target(-m_angle);
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Pitch * 57.3);
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

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(Target_Torque + Gravity_Compensate);
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
    Motor_Yaw.filtered_target_angle.Init(-30, 40, Filter_Fourier_Type_LOWPASS, 20, 0, 1000, 4);
    // 250 300
    Motor_Yaw.PID_Angle.Init(40.0f, 0.0f, 0.3f, 10.0f, 100, 1000, 0.0f, 0.0f, 0, 0.001f, 0.0f, PID_D_First_ENABLE);
    Motor_Yaw.PID_Omega.Init(65.0f, 1200.0f, 0.0f, 0.0f, 10000.0f, 20000.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, PID_D_First_ENABLE);
    Motor_Yaw.PID_Torque.Init(0.78f, 100.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
//    Motor_Yaw.PID_Angle.Init(0.0f, 0.0f, 0.3f, 10.0f, 100, 1000, 0.0f, 0.0f, 0, 0.001f, 0.0f, PID_D_First_ENABLE);
//    Motor_Yaw.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 20000.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, PID_D_First_ENABLE);
//    Motor_Yaw.PID_Torque.Init(0.0f, 0.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.IMU = &Boardc_BMI;
    Motor_Yaw.Init(&hfdcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_IMU_ANGLE, 2048);

    // pitch轴电机
    Motor_Pitch.PID_Angle.Init(40.0f, 0.0f, 0.18f, 0.0f, 10000000, 10000000,0.0f, 0.0f, 0, 0.001f, 0.0f, PID_D_First_ENABLE);
    Motor_Pitch.PID_Omega.Init(60.0f, 1500.0f, 0.0f, 0, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max(), 0.0f, 0.0f, 0.0f, 0.001f, 0.8f);
    Motor_Pitch.PID_Torque.Init(0.8f, 100.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());
    Motor_Pitch.IMU = &Boardc_BMI;
#ifdef DEBUG_PITCH_SPEED_LOOP
    Motor_Pitch.Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_IMU_OMEGA, 3413);
#else
    Motor_Pitch.Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_IMU_ANGLE, 3413);

#endif
}

/**
 * @brief 输出到电机
 *
 */
float temp_err = 0.0f;
float temp_target_angle = 0.0f;
float Tmp_Now_Pitch_Angle = 0.0f, Tmp_Now_Yaw_Angle = 0.0f;
void Class_Gimbal::Output()
{
   if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE) // 云台失能
    {
        // 云台失能
        Motor_Pitch_DM4310.Disable();
        Motor_Yaw_DM4310.Disable();

        // PID积分清零
        Motor_Yaw_DM4310.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw_DM4310.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch_DM4310.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch_DM4310.PID_Omega.Set_Integral_Error(0.0f);
        // 设定输出力矩清零
        Motor_Pitch_DM4310.Set_Target_Torque(0.0f);
        Motor_Yaw_DM4310.Set_Target_Torque(0.0f);
    }
    else // 非失能模式
    {
        switch (Gimbal_Launch_Mode)
        {
        case Launch_Disable:
        {
            // 处理禁用状态
            Motor_Yaw_DM4310.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_IMU_Angle);
            Motor_Pitch_DM4310.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_IMU_Angle);
            Tmp_Now_Pitch_Angle = Motor_Pitch_DM4310.Get_True_Angle_Pitch();
            Tmp_Now_Yaw_Angle = Motor_Yaw_DM4310.Get_True_Angle_Yaw();
            Motor_Yaw_DM4310.PID_Angle.Set_PID_Constants(20.0f, 0.0f, 0.0f);
            Motor_Yaw_DM4310.PID_Omega.Set_PID_Constants(120.0f, 1.5f, 0.0f);
            Motor_Pitch_DM4310.PID_Angle.Set_PID_Constants(22.0f, 0.0f, 0.0f);
            Motor_Pitch_DM4310.PID_Omega.Set_PID_Constants(130.0f, 5.0f, 1.0f);

            break;
        }
        case Launch_Enable:
        {
            // // 处理使能状态
            Motor_Yaw_DM4310.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_Encoder_Position);
            Motor_Pitch_DM4310.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_Encoder_Position);
            Tmp_Now_Pitch_Angle = Motor_Pitch_DM4310.Get_True_Angle_Pitch_From_Encoder();
            Tmp_Now_Yaw_Angle = Motor_Yaw_DM4310.Get_True_Angle_Yaw_From_Encoder();
            Motor_Yaw_DM4310.PID_Angle.Set_PID_Constants(10.0f, 0.0f, 0.2f);
            Motor_Yaw_DM4310.PID_Omega.Set_PID_Constants(120.0f, 1.5f, 0.0f);
            Motor_Pitch_DM4310.PID_Angle.Set_PID_Constants(18.0f, 0.0f, 0.0f);
            Motor_Pitch_DM4310.PID_Omega.Set_PID_Constants(120.0f, 5.0f, 1.0f);
            break;
        }
        }

        // PITCH限制角度
        Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

        // 限制角度范围 处理yaw轴180度问题
        while ((Target_Yaw_Angle - Tmp_Now_Yaw_Angle) > Max_Yaw_Angle)
        {
            Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
        }
        while ((Target_Yaw_Angle - Tmp_Now_Yaw_Angle) < -Max_Yaw_Angle)
        {
            Target_Yaw_Angle += (2 * Max_Yaw_Angle);
        }

        // 设置目标角度
        Motor_Yaw_DM4310.Set_Target_Angle_DEG(Target_Yaw_Angle);
        Motor_Pitch_DM4310.Set_Target_Angle_DEG(Target_Pitch_Angle);
    }
    // 设定达妙电机始终在线
    Motor_Pitch_DM4310.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Motor_Yaw_DM4310.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    // 控制模式
    Output();

    // 根据不同c板的放置方式来修改这几个函数
    Motor_Pitch_DM4310.Transform_Angle();
    Motor_Pitch_DM4310.Transform_EmcoderAngle_To_TrueAngle();
    Motor_Yaw_DM4310.Transform_Angle();
    Motor_Yaw_DM4310.Transform_EmcoderAngle_To_TrueAngle();

    // PID输出
    static uint8_t mod2 = 0;
    mod2++;
    if(mod2 == 2)
    {
        Motor_Yaw_DM4310.TIM_PID_PeriodElapsedCallback();
        mod2 = 0;
    }
    Motor_Pitch_DM4310.TIM_PID_PeriodElapsedCallback();
    // // 增加上位机MPC解算前馈
    // if (Gimbal_Control_Type != Gimbal_Control_Type_DISABLE)
    // {
    //     float tmp_torque_yaw = Motor_Yaw_DM4310.Get_Target_Torque() + J * MiniPC->Get_Rx_Yaw_Acc();
    //     float tmp_torque_pitch = Motor_Pitch_DM4310.Get_Target_Torque() + J * MiniPC->Get_Rx_Pitch_Acc();
    //     Motor_Yaw_DM4310.Reset_Set_Out_And_Output(tmp_torque_yaw);
    //     Motor_Pitch_DM4310.Reset_Set_Out_And_Output(tmp_torque_pitch);
    // }

}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
