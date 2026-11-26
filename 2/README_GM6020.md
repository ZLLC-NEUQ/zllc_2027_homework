# GM6020 + STM32 HAL CAN 驱动说明

适用于 STM32F405（bxCAN）+ STM32CubeMX 生成的 HAL 工程，控制大疆 GM6020 云台电机。

---

## 1. 硬件接线

| STM32 引脚 | 接到 |
|---|---|
| PA11（CAN1_RX） | CAN 收发器 RXD（如 TJA1050 / SN65HVD230） |
| PA12（CAN1_TX） | CAN 收发器 TXD |
| — | 收发器 CANH / CANL → 电机 CANH / CANL |
| — | 总线两端各接 **120 Ω** 终端电阻 |
| — | GM6020 供电 **24 V**，CAN 地与电源地共地 |

> 电机 ID 用大疆 RoboMaster Assistant 设置（默认 ID=1）。ID 1~4 走 0x1FF 帧，ID 5~7 走 0x2FF 帧。

## 2. CubeMX 配置（CAN 1 Mbps）

GM6020 固定 **1 Mbps**。以 APB1 = 42 MHz（F405 主频 168 MHz）为例：

| Prescaler | BS1 | BS2 | SJW | 位时间 | 采样点 |
|---|---|---|---|---|---|
| **6** | **5** | **1** | 1 | 1 µs | 85.7% |
| 3 | 11 | 2 | 1 | 1 µs | 85.7% |
| 3 | 9 | 4 | 1 | 1 µs | 71.4% |

步骤：

1. `Connectivity → CAN1` → 勾 `Activated`，Mode 勾 `Master Mode`；
2. Parameter Settings：按上表填 Prescaler / BS1 / BS2 / SJW；
3. `NVIC Settings` → 勾 `CAN1 RX0 interrupts`；
4. 过滤器二选一：
   - **CubeMX 配**：Filter Settings 里加一条 32 位掩码过滤，ID=0x200、Mask=0x7F0、FIFO0、Enable（推荐）；
   - **代码配**：不配，交给 `GM6020_Init()` 里的 `HAL_CAN_ConfigFilter()`；
5. 生成代码。

## 3. 把驱动加入工程

把 `gm6020.h`、`gm6020.c` 放进工程（如 `Core/Src`、`Core/Inc`），Keil/CubeIDE 里添加编译。

## 4. main.c 集成

```c
/* USER CODE BEGIN Includes */
#include "gm6020.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
if (GM6020_Init(&hcan1) != HAL_OK)   /* 若 CubeMX 已配过滤器，可去掉此句 */
{
    Error_Handler();
}
if (HAL_CAN_Start(&hcan1) != HAL_OK)
{
    Error_Handler();
}
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
/* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
uint32_t last_tx = 0;
while (1)
{
    /* 每 2 ms 发一次控制帧（必须周期发送，否则电机自动停机） */
    if (HAL_GetTick() - last_tx >= 2u)
    {
        last_tx = HAL_GetTick();

        GM6020_SetVoltage(1, 3000);   /* ID1，约 2.4V，小电压试转 */
        GM6020_SendVoltage(&hcan1);   /* 同时把 2~7 号电机的指令一起发出 */
    }
    /* USER CODE END WHILE */
}
```

## 5. 接收回调

放在 `main.c` 的 `/* USER CODE BEGIN 4 */` 区域或任意用户文件：

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0u)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) == HAL_OK)
        {
            GM6020_ParseFeedback(&hdr, data);
        }
    }
}
```

之后直接读：

```c
g_gm6020[1].angle_deg;
g_gm6020[1].speed_rpm;
g_gm6020[1].torque_raw;
g_gm6020[1].temperature;
GM6020_IsOnline(1, HAL_GetTick());
```

## 6. 协议速查

| 项目 | 内容 |
|---|---|
| 波特率 | 1 Mbps |
| 电压控制帧 | 0x1FF（ID 1~4）、0x2FF（ID 5~7），4 × int16 大端，±30000 |
| 电流控制帧 | 0x1FE（ID 1~4）、0x2FE（ID 5~8），±16384 ≈ ±3A |
| 反馈帧 | 0x205 ~ 0x20B（ID 1~7） |
| 反馈数据 | [0..1] 角度 0~8191；[2..3] 转速 rpm；[4..5] 转矩电流；[6] 温度；[7] 保留 |

## 7. 注意事项

1. **必须周期发送**：电机长时间收不到控制帧会自动停机，建议 1~2 ms 一帧。
2. **电压/电流控制二选一**：只发 0x1FF/0x2FF 或只发 0x1FE/0x2FE，不要混发。
3. **先小电压**：初次调试用 ±3000 以内，确认转向、角度、转速反馈正确再加。
4. **反馈 ID 从帧头取**：`StdId - 0x204` 即电机 ID，不要用 `data[7]`。
5. **三个发送邮箱**：`GM6020_SendVoltage()` 一次发两帧（0x1FF、0x2FF），若邮箱紧张会返回 `HAL_BUSY`，下周期重发即可。
6. **角度是机械角**：0~8191 对应 0~360°，跨圈会回绕；需要多圈位置要自己做累计。
7. **CAN 速率必须一致**：所有 GM6020 和主控都要 1 Mbps，终端电阻 120 Ω。

## 8. 下一步

需要速度环/位置环时，可以在 `GM6020_SendVoltage()` 之前加入 PID：

```c
float pid_calc(float target, float feedback);   /* 自己实现 */
g_gm6020[1].set_voltage = (int16_t)pid_calc(target_rpm, g_gm6020[1].speed_rpm);
```

调试顺序建议：**开环小电压 → 速度环 → 位置环**。
