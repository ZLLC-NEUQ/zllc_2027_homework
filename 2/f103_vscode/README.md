# STM32F103C8T6 + GM6020（VS Code / EIDE 手写 CAN）

不使用 CubeMX 生成的代码，CAN 初始化、过滤器、中断接收全部手写。

## 文件

| 文件 | 作用 |
|---|---|
| `can_bsp.h/.c` | CAN1 底层：时钟/GPIO/过滤器/NVIC/启动/中断收发 |
| `gm6020.h/.c` | GM6020 控制帧发送 + 反馈解析进结构体 `g_gm6020[]` |
| `main.c` | 72 MHz 时钟配置、初始化、2 ms 周期发控制帧 |

## 硬件

- 芯片：STM32F103C8T6，8 MHz 外部晶振
- CAN：PA11 = CAN1_RX，PA12 = CAN1_TX → 收发器（TJA1050/SN65HVD230）→ GM6020
- 总线两端各 120 Ω 终端电阻，电机 24 V 供电，共地
- 若用 PB8/PB9，需要 AFIO 重映射并改 GPIO 配置

## 关键参数（手写配置）

| 项目 | 值 |
|---|---|
| SYSCLK | 72 MHz（HSE 8 MHz × PLL9） |
| APB1 | 36 MHz（CAN 时钟） |
| Prescaler | 4 |
| BS1 / BS2 | 7 Tq / 1 Tq |
| SJW | 1 Tq |
| 位时间 | (1+7+1) × 4/36 MHz = 1 µs → **1 Mbps** |
| 采样点 | (1+7)/9 = 88.9% |
| 过滤器 | 32 位掩码，ID 0x200、Mask 0x7F0 → 收 0x200~0x20F |

## EIDE 工程配置步骤

1. **文件加入工程**：把 `can_bsp.c`、`gm6020.c`、`main.c` 加入 EIDE 的源文件列表；
2. **HAL 库**：工程需包含 STM32F1 HAL 驱动（可从 STM32CubeF1 固件包拷 `Drivers/` 目录）：
   - `stm32f1xx_hal.c`、`stm32f1xx_hal_can.c`、`stm32f1xx_hal_gpio.c`、
     `stm32f1xx_hal_rcc.c`、`stm32f1xx_hal_cortex.c`
   - `system_stm32f1xx.c`
   - 启动文件 `startup_stm32f103xb.s`
3. **宏定义**：`USE_HAL_DRIVER`、`STM32F103xB`
4. **头文件路径**：
   - `Drivers/STM32F1xx_HAL_Driver/Inc`
   - `Drivers/CMSIS/Device/ST/STM32F1xx/Include`
   - `Drivers/CMSIS/Include`
5. **`stm32f1xx_hal_conf.h`**：确保打开
   `#define HAL_CAN_MODULE_ENABLED`（另外 GPIO/RCC/CORTEX 为默认开启）
6. **中断**：`USB_LP_CAN1_RX0_IRQHandler` 已在 `can_bsp.c` 里实现。
   如果工程的 `stm32f1xx_it.c` 里也有同名函数，**删掉一处**，否则链接报重复定义。
7. **启动文件**：CubeMX/Keil 工程里的 `startup_stm32f103xb.s` 已包含
   `USB_LP_CAN1_RX0_IRQHandler` 的向量表项，无需改。
8. **下载/调试**：EIDE 里选 ST-Link / DAP-Link，配好 `launch.json` 即可。

> 如果你用的是旧版 F1 HAL（没有 `HAL_CAN_AddTxMessage`），需要升级到
> STM32Cube F1 固件包 V1.8 及以上的 HAL。

## 电机协议速查（GM6020）

| 项目 | 内容 |
|---|---|
| 控制帧 | 0x1FF（ID1~4）、0x2FF（ID5~7），4×int16 大端，±30000 |
| 反馈帧 | 0x205 ~ 0x20B（ID = 0x204 + 电机号） |
| 反馈字节 | [0..1] 角度 0~8191；[2..3] 转速 rpm；[4..5] 转矩电流；[6] 温度；[7] 保留 |

## 使用方式

```c
GM6020_SetVoltage(1, 3000);   /* 1 号电机，约 2.4V */
GM6020_SendVoltage();         /* 发 0x1FF + 0x2FF */

/* 中断里已自动更新，直接读： */
float  deg  = g_gm6020[1].angle_deg;
int16_t rpm = g_gm6020[1].speed_rpm;
uint8_t t   = g_gm6020[1].temperature;
if (!GM6020_IsOnline(1, HAL_GetTick())) { /* 掉线处理 */ }
```

## 调试顺序建议

1. 先用万用表/示波器确认 PA12 有波形、PA11 有隐性电平；
2. 用 CAN 分析仪看 0x1FF 帧是否发出、ID/数据是否正确；
3. 上电后看是否收到 0x205 反馈（`g_gm6020[1].rx_count` 增加）；
4. 小电压试转，确认正负方向、角度变化方向；
5. 再加 PID（速度环 → 位置环）。

## 常见问题

| 现象 | 排查 |
|---|---|
| 编译报 `CAN_FilterTypeDef` 无 `SlaveStartFilterBank` | F1 没有该字段，本代码已按 F1 写 |
| 链接报 `USB_LP_CAN1_RX0_IRQHandler` 重复 | it.c 与 can_bsp.c 留一个 |
| 收不到反馈 | 波特率不是 1 Mbps / 终端电阻 / 过滤器配置 / 收发器接线 |
| 电机不转 | 控制帧没周期发送、电压太小、ID 不对（0x1FF 只对 ID1~4） |
| 一上电就停机保护 | 电压指令超范围或供电不足，先 ±3000 以内 |
