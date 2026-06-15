# 征龙凌沧27赛季嵌入式培训


## 学习记录 1：M65 步兵底盘（舵轮）功能恢复

> **任务**：参考学长 26 赛季代码，把训练仓库的底盘运动功能恢复到 M65 步兵（舵轮底盘）
>
> **硬件**：达妙 MC02（STM32H723）。底盘实车接线：CAN1 = 8 个 C620（4 驱动轮 + 4 转向舵）、CAN2 = 超级电容、CAN3 = 自研磁编（舵角）、UART10 = 裁判系统。

### 一、超级电容 CAN 总线适配（CAN3 → CAN2）

**遇到的问题：**
训练仓库代码里超电初始化是 `Supercap.Init(&hfdcan3, 75.f)`，即挂在 CAN3；但 M65 实车的超电接在 **CAN2**。不改的话超电收发会落在错误的总线上。

**排查 / 确认：**
查达妙 MC02 官方板图的引脚定义，确认丝印 CAN 接口与芯片 FDCAN 外设的对应关系：

| 丝印接口 | 引脚（RX/TX） | 对应外设 |
| --- | --- | --- |
| CAN1 | PD0 / PD1 | FDCAN1 (`hfdcan1`) |
| CAN2 | PB5 / PB6 | FDCAN2 (`hfdcan2`) |
| CAN3 | PD12 / PD13 | FDCAN3 (`hfdcan3`) |

![达妙 MC02 板图：CAN 接口引脚定义](photos/MC02_pinout.png)

即三个接口干净 1:1 对应 `hfdcan1/2/3`，确认超电应使用 `hfdcan2`。

**解决办法（发送 + 接收两侧都要改）：**

1. `crt_chassis.cpp` 的 `Init` 里（发送 / 绑定侧）：
   ```cpp
   // 改前
   Supercap.Init(&hfdcan3, 75.f);
   // 改后
   Supercap.Init(&hfdcan2, 75.f);
   ```
2. `tsk_config_and_callback.cpp` 里（接收侧）：超电接收 `case 0x67` 原本在 CAN3 回调里，M65 在 CAN2，于是移到 `Chassis_Device_CAN2_Callback`：
   ```cpp
   case (0x67): // 超电接收
       chariot.Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
       break;
   ```

**收获：** 代码里一行"挂哪条总线"的配置，背后对应的就是板子上一个具体接口、一组引脚。查板图把 **软件配置 ↔ 芯片外设 ↔ 板上接口** 对应起来，才能确认改得对——读软件反而进一步加深了我对硬件电气连接的理解。

### 二、舵角磁编（MA600）派发补全

**遇到的问题：**
舵轮底盘要靠磁编读舵角做闭环。但训练仓库里 `Chassis_Device_CAN3_Callback`（CAN3 接收回调）是个**空函数 `{}`**——磁编的 CAN 帧收进来后没有任何派发，`MA600_Data_Process` 永远不会被调用，`MA600_Status` 一直是 DISABLE，**舵角反馈是死的**。

**代码链路梳理（理解磁编数据怎么变成舵角）：**

1. 磁编在 **CAN3** 发帧，每个舵一个 ID：`0xD1 ~ 0xD4`；
2. `drv_can.cpp` 的 `HAL_FDCAN_RxFifo0Callback` 按总线收下，调用 CAN3 注册的回调；
3. CAN3 回调按 CAN ID 派发，调 `Motor_Steer[i].MA600_Data_Process(帧)`；
4. `MA600_Data_Process` 解析单圈 / 多圈 / 角速度（`Data[1..6]`，每个 `×(-1)/100`），算出
   `Zero_Offset_Radian = Normalize(单圈角 − 零点)`；
5. 底盘每周期用 `Get_Now_Zero_Offset_Radian()` 作为**权威舵角**（估速、优劣弧解算、力解算都用它），并 `Set_Transform_Radian()` 喂给舵电机角度环；
6. 舵电机 `AGV_MODE` 串级：**角度环反馈用磁编角，速度环反馈用电机自身转速**（避免磁编差分测速不准）；
7. `ita_chariot.cpp` 周期调用 `TIM_Alive_PeriodElapsedCallback_MA600()` 做磁编掉线检测。

**解决办法：** 把空的 CAN3 回调补全，将 4 个磁编 ID 派发到对应舵电机：
```cpp
void Chassis_Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
        case (0xD1): chariot.Chassis.Motor_Steer[0].MA600_Data_Process(CAN_RxMessage); break;
        case (0xD2): chariot.Chassis.Motor_Steer[1].MA600_Data_Process(CAN_RxMessage); break;
        case (0xD3): chariot.Chassis.Motor_Steer[2].MA600_Data_Process(CAN_RxMessage); break;
        case (0xD4): chariot.Chassis.Motor_Steer[3].MA600_Data_Process(CAN_RxMessage); break;
    }
}
```

**收获：** 一条 sensor → 控制的完整数据通路会横跨 **驱动层(drv_can) → 设备层(dvc_djimotor) → 应用层(crt_chassis) → 交互层(ita_chariot)**。只看单个文件会误以为"这函数没东西调用"，但其实要顺着 CAN 的注册（`CAN_Init`）和按 ID 派发，才能把链路接通。

### 三、其余移植改动

- `crt_chassis.h / .cpp`：由四轮 `Class_Tricycle_Chassis` 换为舵轮 `Class_Steering_Wheel_Chassis`；
- `ita_chariot.h`：底盘对象类型改为 `Class_Steering_Wheel_Chassis`；
- `ita_chariot.cpp`：`Chassis_Control_Type_SPIN` → `Chassis_Control_Type_SPIN_Positive`；
- 以上改动 Keil 编译 **0 Error / 0 Warning**。

![Keil 编译 0 Error / 0 Warning](photos/build_ok.png)

### 四、待上车验证（TODO）

- 舵电机零点标定 `Set_Zero_Position`（需实车逐个标定）

---

