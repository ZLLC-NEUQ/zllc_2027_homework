/**
 * @file dvc_minipc.cpp
 * @author cjw by yssickjgd
 * @brief 迷你主机
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* includes ------------------------------------------------------------------*/

#include "dvc_minipc.h"
volatile uint32_t debug_can_id = 0;
volatile uint8_t debug_rx0 = 0;
volatile uint8_t debug_rx1 = 0;

/* private macros ------------------------------------------------------------*/

/* private types -------------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/

/* private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 迷你主机初始化,can
 *
 */
void Class_MiniPC::Init(FDCAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == FDCAN1)
  {
    CAN_Manage_Object = &CAN1_Manage_Object;
    CAN_Tx_Data = CAN1_MiniPc_Tx_Data;
  }
  else if (hcan->Instance == FDCAN2)
  {
    CAN_Manage_Object = &CAN2_Manage_Object;
    CAN_Tx_Data = CAN2_MiniPc_Tx_Data;
  }
}

float camera_distance = 0.036;
/**
 * @brief 数据处理过程
 *
 */
void Class_MiniPC::Data_Process()
{
    const float scale = 1.0f / 10000.0f;
    const float rad_to_deg = 180.0f / PI;

    Rx_Angle_Yaw =
        Yaw_Rx_Cache.angle * scale * rad_to_deg;

    Rx_Angle_Pitch =
        Pitch_Rx_Cache.angle * scale * rad_to_deg;

    Rx_Yaw_Velocity =
        Yaw_Rx_Cache.velocity * scale * rad_to_deg;

    Rx_Pitch_Velocity =
        Pitch_Rx_Cache.velocity * scale * rad_to_deg;

    Rx_Yaw_Acceleration =
        Yaw_Rx_Cache.acceleration * scale * rad_to_deg;

    Rx_Pitch_Acceleration =
        Pitch_Rx_Cache.acceleration * scale * rad_to_deg;

    Fire = (Yaw_Rx_Cache.mode >> 1) & 0x01;
    Control = (Yaw_Rx_Cache.mode >> 2) & 0x01;

    alive = Control;

    Math_Constrain(&Rx_Angle_Pitch, -20.0f, 25.0f);
}

/**
 * @brief 迷你主机发送数据输出
 *
 */
float Dttttt;
uint32_t last_cnttt = 0;
void Class_MiniPC::Output()
{
  // 设置发送数据

  float Yaw_rad = Tx_Angle_Yaw * PI / 180.0f;
  float Pitch_rad = Tx_Angle_Pitch * PI / 180.0f;
  float Roll_rad = Tx_Angle_Roll * PI / 180.0f;

  // Pack_Tx_CAN.q[0] = (int16_t)((arm_sin_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f) - arm_cos_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f)) * 10000.f);
  // Pack_Tx_CAN.q[1] = (int16_t)((arm_cos_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f) + arm_sin_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f)) * 10000.f);
  // Pack_Tx_CAN.q[2] = (int16_t)((arm_cos_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f) - arm_sin_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f)) * 10000.f);
  // Pack_Tx_CAN.q[3] = (int16_t)((arm_cos_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f) + arm_sin_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f)) * 10000.f);

  Pack_Tx_CAN.q[3] = (int16_t)((arm_cos_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f) + arm_sin_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f)) * 10000.f);
  Pack_Tx_CAN.q[0] = (int16_t)((arm_sin_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f) - arm_cos_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f)) * 10000.f);
  Pack_Tx_CAN.q[1] = (int16_t)((arm_cos_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f) + arm_sin_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f)) * 10000.f);
  Pack_Tx_CAN.q[2] = (int16_t)((arm_cos_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f) - arm_sin_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f)) * 10000.f);
    memcpy(CAN_Tx_Data, &Pack_Tx_CAN, sizeof(Pack_tx_t));
    Dttttt = DWT_GetDeltaT(&last_cnttt);

}

/**
 * @brief tim定时器中断增加数据到发送缓冲区
 *
 */
void Class_MiniPC::TIM_Write_PeriodElapsedCallback()
{
  Transform_Angle_Tx();
  Output();
}

/**
 * @brief tim定时器中断定期检测迷你主机是否存活
 *
 */
void Class_MiniPC::TIM1msMod50_Alive_PeriodElapsedCallback()
{
  // 判断该时间段内是否接收过迷你主机数据
  if (Flag == Pre_Flag)
  {
    // 迷你主机断开连接
    MiniPC_Status = MiniPC_Status_DISABLE;
    // Buzzer.Set_NowTask(BUZZER_DEVICE_OFFLINE_PRIORITY);
  }
  else
  {
    // 迷你主机保持连接
    MiniPC_Status = MiniPC_Status_ENABLE;
  }

  Pre_Flag = Flag;
}

/**
 * @brief CRC16 Caculation function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
 * @return : CRC16 checksum
 */
uint16_t Class_MiniPC::Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;

  if (pchMessage == NULL)
    return 0xFFFF;
  while (dwLength--)
  {
    ch_data = *pchMessage++;
    wCRC = (wCRC >> 8) ^ W_CRC_TABLE[(wCRC ^ ch_data) & 0x00ff];
  }

  return wCRC;
}

/**
 * @brief CRC16 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */

bool Class_MiniPC::Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  if ((pchMessage == NULL) || (dwLength <= 2))
    return false;

  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
  return (
      (w_expected & 0xff) == pchMessage[dwLength - 2] &&
      ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**

@brief Append CRC16 value to the end of the buffer
@param[in] pchMessage : Data to Verify,
@param[in] dwLength : Stream length = Data + checksum
@return none
*/
void Class_MiniPC::Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
  uint16_t w_crc = 0;

  if ((pchMessage == NULL) || (dwLength <= 2))
    return;

  w_crc = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);

  pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}

/**
 * 计算给定向量的偏航角（yaw）。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量（未使用）
 * @return 计算得到的偏航角（以角度制表示）
 */
float Class_MiniPC::calc_yaw(float x, float y, float z)
{
  // 使用 atan2f 函数计算反正切值，得到弧度制的偏航角
  float yaw = atan2f(y, x);

  // 将弧度制的偏航角转换为角度制
  yaw = (yaw * 180 / PI); // 向左为正，向右为负

  return yaw;
}

/**
 * 计算给定向量的欧几里德距离。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的欧几里德距离
 */
float Class_MiniPC::calc_distance(float x, float y, float z)
{
  // 计算各分量的平方和，并取其平方根得到欧几里德距离
  float distance = sqrtf(x * x + y * y + z * z);

  return distance;
}

/**
 * 计算给定向量的俯仰角（pitch）。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的俯仰角（以角度制表示）
 */
float dist;
float Class_MiniPC::calc_pitch(float x, float y, float z)
{
    float d = calc_distance(x, y, z);
    if (d < a_d)
    {
        // return 当前pitch，因为无解1
		return IMU->Get_Angle_Pitch();
		
    }

    float v0 =
        Referee->Get_Referee_Status() == Referee_Status_ENABLE &&
                Referee->Get_Shoot_Speed()
            ? Referee->Get_Shoot_Speed()
            : bullet_v;
    
    // 初始估值一定偏小一点点
    float t = (d - a_d) / v0;
    const float t1 = 2.0f * a_d * v0, t2 = v0 * v0 - z * g;
    
    // 牛顿迭代法，可省略，最好两次
    t -= (d * d - a_d * a_d + t * (-t1 + t * (-t2 + 0.25f * g * g * t * t))) / (-t1 + t * (-2.0f * t2 + g * g * t * t));
    t -= (d * d - a_d * a_d + t * (-t1 + t * (-t2 + 0.25f * g * g * t * t))) / (-t1 + t * (-2.0f * t2 + g * g * t * t));
    
    // pitch向下为正，加负号
    return 180.0f * atanf((z + 0.5 * g * t * t) / sqrtf(x * x + y * y)) / PI;
}

/**
 * 计算计算yaw，pitch
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的目标角（以角度制表示）
 */

void Class_MiniPC::Self_aim(float x, float y, float z, float *yaw, float *pitch, float *distance)
{

  *yaw = calc_yaw(x, y, z);
  *pitch = calc_pitch(x, y, z);
  *distance = calc_distance(x, y, z);
}

float Class_MiniPC::meanFilter(float input)
{
  static float buffer[5] = {0};
  static uint64_t index = 0;
  float sum = 0;

  // Replace the oldest value with the new input value
  buffer[index] = input;

  // Increment the index, wrapping around to the start of the array if necessary
  index = (index + 1) % 5;

  // Calculate the sum of the buffer's values
  for (int i = 0; i < 5; i++)
  {
    sum += buffer[i];
  }

  // Return the mean of the buffer's values
  return sum / 5.0;
}

/************************ copyright(c) ustc-robowalker **************************/
/**
 * @brief CAN通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
static int16_t MiniPC_Read_Int16_LE(const uint8_t *data)
{
    uint16_t value;

    value = (uint16_t)data[0];
    value |= ((uint16_t)data[1] << 8);

    return (int16_t)value;
}
void Class_MiniPC::CAN_RxCpltCallback(uint32_t can_id,const uint8_t *rx_data)
{
    if (rx_data == NULL)
  {
      return;
  }
      debug_can_id = can_id;
      debug_rx0 = rx_data[0];
      debug_rx1 = rx_data[1];
  uint8_t axis = rx_data[0] & 0x01;//axis=0：yaw , axis=1：pitch
  Struct_MiniPC_Axis_Rx_Cache *cache = NULL;
    if ((can_id == 0xA3) && (axis == 0)) 
  {
      cache = &Yaw_Rx_Cache;//0xA3 必须对应 axis=0
  }
  else if ((can_id == 0xA4) && (axis == 1))
  {
      cache = &Pitch_Rx_Cache;//0xA4 必须对应 axis=1
  }
  else
  {
      return;//如果算法发送错了，直接丢弃这一帧
  }
  //解析8字节数据
  cache->mode = rx_data[0];
  cache->seq = rx_data[1];

  cache->angle = MiniPC_Read_Int16_LE(&rx_data[2]);

  cache->velocity = MiniPC_Read_Int16_LE(&rx_data[4]);

  cache->acceleration = MiniPC_Read_Int16_LE(&rx_data[6]);

  cache->new_data = 1;
//  //等待 yaw 和 pitch 都到达
//    if ((Yaw_Rx_Cache.new_data == 0) || (Pitch_Rx_Cache.new_data == 0))
//  {
//      return;
//  }
//  //比较序列号，只有两个包的 seq 一致，才认为是同一组数据
//    if (Yaw_Rx_Cache.seq != Pitch_Rx_Cache.seq)
//  {
//      return;
//  }
//  //比较 shoot 和 control，mode 的 bit1 和 bit2 应该在两个包中相同
//    if ((Yaw_Rx_Cache.mode & 0x06) !=(Pitch_Rx_Cache.mode & 0x06))
//  {
//      return;
//  }
  //处理完整的一组数据
  Data_Process();
  //Flag 不能在每收到一个包时增加，应该在 完整收到一组匹配的yaw和pitch 之后，才增加一次
  //否则只收到 yaw、不收到 pitch，也会被误判为算法板在线
  Flag += 1;

  Yaw_Rx_Cache.new_data = 0;
  Pitch_Rx_Cache.new_data = 0;
}
/*函数逻辑顺序：检查指针->读取axis->根据CAN ID选缓存->解析8字节->等待两个包->比较seq
->比较shoot/control->更新角度、速度、加速度->Flag+1*/