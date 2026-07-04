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

## 学习记录 2：底盘核心解算流程理解

> 这次不是改代码，而是把学长舵轮底盘的运动解算流程读懂，为后续上车标定、调参打基础。对底盘主循环整体的理解：**把操作手的速度指令，一步步解算成每个电机的输出。**

### 一、总体流程（`TIM_Calculate_PeriodElapsedCallback`）

底盘主循环由定时器按固定周期（约 1ms）反复调用，每次按顺序跑完整套控制：

```
斜坡平滑目标速度 -> 卡尔曼估真实车速 -> 舵角解算 -> 力解算 -> 功率限制 -> 下发电机
```

- **斜坡平滑**：把目标速度过一遍斜坡函数，避免突变导致打滑、抖动；
- **卡尔曼估速** `Chassis_Speed_Estimate`：估出底盘当前真实速度，作为后面 PID 的反馈；
- 失能（DISABLE）模式下所有电机置零、清积分，安全停车。

### 二、舵角解算（`Stree_Angle_Resolution`）

决定每个舵轮**朝哪个方向、目标转多快**。核心思想：每个轮的速度 = **整车平移** + **整车自转**（ω 乘以 r）两部分的矢量和。

- 把这两部分在该轮位置叠加成一个速度矢量；
- 矢量的**方向**就是舵该转到的角度（`atan2`），**模长**就是驱动轮的目标速度；
- **优劣弧优化**：如果算出来要转的角度超过 90 度，就让舵转到**相反方向**、同时把轮速**取反**——结果一样，但舵少转一大截、响应更快；
- 怠速时把四个轮摆成 X 形，起到锁车作用。

### 三、力解算（`Force_Speed_Resolution`）

决定每个**驱动轮出多大力 / 扭矩**。这套代码用的是**力控（扭矩控制）**，不是各轮独立速度环。分四步：

1. **整车速度 PID -> 期望合力**：对 Vx、Vy、ω 各跑一个 PID，反馈用卡尔曼估出的真实车速，输出当作“整车要施加的合力 Fx、Fy 与力矩 τ”；
2. **坐标变换（跟随云台）**：把合力按“底盘朝向与参考方向之差 `derta_angle`”旋转到底盘自己的坐标系——这就是底盘跟随云台的来源；
3. **分配到每个轮**：每个轮只能沿当前舵向出力，于是把合力投影到各轮舵向：
   ```
   tmp_force[i] = Fx*cos(θ_i) + Fy*sin(θ_i) + (τ / R_DIST)*cos(Azimuth_i - θ_i)
   ```
   （`θ_i` 为该轮当前舵角，来自磁编）前半是平移分量、后半是自转分量，四轮合起来正好等于整车想要的力 + 力矩；
4. **力 -> 扭矩 + 前馈**：`扭矩 = 力 * 轮半径`，再加“目标轮速 - 实际轮速”的跟踪项和**动摩擦前馈**（正转加、反转减、近零线性过渡），最后以扭矩模式下发。

**这么做的原因**：先算整车合力、再分配到各轮、扭矩模式下发，好处是四轮协调一致地分担同一需求，也便于紧接着**统一做功率限制**。

### 四、涉及的机械常量（M65 待核实）

力解算里用到几个机械常量，上车前要按 M65 实测 / 确认，否则出力和转向会偏：

- `WHEEL_RADIUS`：轮半径（力转扭矩用，错了出力大小不对）；
- `R_DIST`：底盘中心到轮的距离（自转力分配用）；
- `Wheel_Azimuth[i]`：每个轮的安装方位角（自转力分配用，错了车转不正 / 走偏）。

### 小结

读懂这条解算流程后，我对底盘“指令怎么变成电机输出”有了整体认识：**速度指令 -> PID 算合力 -> 坐标变换 -> 按舵向分配 -> 转扭矩下发**。也清楚了后续上车要标定 / 调的参数分别影响哪一步。

## 学习记录 3：功率限制理解（转向优先分配）

> 接着上一篇的解算流程往下读。力解算算出每个电机“想出多大扭矩”后，下发前还要过一道**功率限制**，保证整车不超裁判系统/超电给的功率预算。算法本体在 `User/Algorithm/Src/alg_new_power_limit.cpp` 的 `Power_Task`，由 `crt_chassis.cpp` 调用。

> 补充：工程里还有个旧版 `alg_power_limit.cpp`（没用上）。一开始搜 `Power_Task` 发现旧版里根本没有这个函数，才定位到真正在用的是 `alg_new_power_limit.cpp`——读代码前先确认“到底用哪一版”很重要。

### 一、单电机功率模型

预测一个电机大概消耗多少功率（`Calculate_Theoretical_Power`）：

```
cmdPower = k4*ω*τ      // 机械功率
         + k1*|ω|       // 转速相关损耗
         + k2*τ*τ       // 电流/铜损
         + k3           // 常数损耗
```

ω 是转速、τ 是扭矩。8 个电机各算一遍，得到每个的理论功率。

### 二、分配缩放（转向舵优先）

不是把 8 个电机一起按同一比例压，而是**分驱动和转向两拨，有优先级**：

- 先按“驱动 vs 转向”分别累加理论功率；
- **转向舵先分，且额度给到总预算的 80%**（`dir_power_limit = Max_Power * 0.8`）：
  - 转向需求没超 80% -> 转向**全额满足**，剩余全留给驱动轮；
  - 转向需求超了 -> 转向按比例压到 80%，驱动轮只能用剩下的 20%；
- **驱动轮再分剩余额度**：超了按比例压，没超就满额。

**为什么转向优先**：舵转不到位，车就往错误方向跑。所以宁可先保证方向对，把大头功率给转向，再把剩余分给决定“跑多快”的驱动轮。这是舵轮底盘一个关键的取舍。

### 三、反解扭矩

知道某个电机“只准用这么多功率”后，反过来求它最多能出多大扭矩（`Calculate_Toque`）：把功率模型当成一个关于扭矩的**一元二次方程**，用求根公式解出对应扭矩（判别式 < 0 就给 0），再乘 `GET_TORQUE_TO_CMD_CURRENT` 转成指令电流，最后**限幅到 ±16384**（C620 电流满量程）下发。

### 四、两个细节

- **反向放电不参与分配**：理论功率 < 0 的电机（在减速、当发电机往回充能）直接用原输出，不去压它——它不耗功率，反而是缓冲；
- 代码里还预留了“按误差分配”“RLS 在线辨识 k 参数”的逻辑，但目前都注释掉了，实际走的是上面这套固定 80/20 分配。

### 小结：整条功率线闭环

`crt_chassis` 打包 8 个电机数据 + 定预算 `Max_Power`（来自改到 CAN2 的超电）-> `Power_Task`：算理论功率 -> 转向舵优先、驱动轮吃剩余地分配 -> 反解每个电机的限制扭矩 -> 写回 output -> `crt_chassis` 再 `Set_Out` 下发。

至此底盘“速度指令 -> 解算 -> 功率限制 -> 下发”这条主线读通了

## 学习记录 4：实车联调代码更新（7.4）

> **目标**：把下板舵轮底盘、上/下板通信、MiniPC 协议、VT13 遥控器判定、Pitch 闭环方向和设备在线检测这些关键链路接起来，并加入必要的调试观测变量。


### 一、上板到下板 CAN 控制链路调试

当前下板主要通过 CAN2 接收上板发来的底盘控制帧。`tsk_config_and_callback.cpp` 中，`0x77` 被作为上板控制帧处理：

```cpp
case (0x77): // 留给上板通讯
{
    chariot.CAN_Chassis_Rx_Gimbal_Callback();
}
break;
```

在 `CAN_Chassis_Rx_Gimbal_Callback()` 中解析上板下发的控制类型、云台坐标系速度、pitch 角等信息，并把云台坐标系速度转换到底盘坐标系，再写入底盘目标速度：

```cpp
Chassis.Set_Target_Velocity_X(chassis_velocity_x);
Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
```

为了方便在 Debug Watch 窗口确认链路是否真的跑通，新增了这些下板调试变量：

```cpp
volatile uint32_t dbg_rx_0x77_cnt = 0;
volatile uint8_t dbg_rx_control_type = 0;
volatile uint8_t dbg_rx_chassis_mode = 0;

volatile float dbg_rx_gimbal_vx = 0.0f;
volatile float dbg_rx_gimbal_vy = 0.0f;
volatile float dbg_rx_chassis_vx = 0.0f;
volatile float dbg_rx_chassis_vy = 0.0f;
volatile float dbg_rx_pitch = 0.0f;
```

调试时可以按这个顺序判断问题在哪一层：

1. `dbg_rx_0x77_cnt` 是否增加：判断下板是否收到上板 CAN 帧；
2. `dbg_rx_control_type` / `dbg_rx_chassis_mode` 是否变化：判断控制类型是否解析正确；
3. `dbg_rx_gimbal_vx` / `dbg_rx_gimbal_vy` 是否变化：判断上板是否真的下发速度；
4. `dbg_rx_chassis_vx` / `dbg_rx_chassis_vy` 是否变化：判断坐标变换和目标速度写入是否正常；
5. 轮子仍不动时，再继续看底盘模式、alive 标志、电机在线状态和功率限制。

### 二、底盘随动 yaw 电机适配

之前的版本里，底盘随动使用的是 DJI GM6020 yaw 电机对象：

```cpp
Class_DJI_Motor_GM6020 Motor_Yaw;
```

当前版本根据实车改为 LK yaw 电机：

```cpp
Class_LK_Motor Motor_Yaw;
```

对应地，掉线保护也从 DJI 电机状态判断改成 LK 电机状态判断：

```cpp
if (Motor_Yaw.Get_LK_Motor_Status() == LK_Motor_Status_DISABLE ||
    Gimbal_Status == Gimbal_Status_DISABLE)
{
    buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
}
```

这一步的意义是：底盘随动所需的 yaw 编码器反馈要来自当前车上真实使用的 LK yaw 电机，否则底盘坐标转换会没有可靠的角度来源。

### 三、VT13 遥控器输入判定优化

原逻辑是只要摇杆值“不等于 0”，就认为遥控器正在控制：

```cpp
if (VT13.Get_Left_X() != 0 ||
    VT13.Get_Left_Y() != 0 ||
    VT13.Get_Right_X() != 0 ||
    VT13.Get_Right_Y() != 0)
```

实车上摇杆可能有轻微零漂，直接用 `!= 0` 容易误判，一旦遥控器与车连接，云台就剧烈振动。
当前版本增加了死区：

```cpp
const float dead = 0.05f;

if (Math_Abs(VT13.Get_Left_X()) > dead ||
    Math_Abs(VT13.Get_Left_Y()) > dead ||
    Math_Abs(VT13.Get_Right_X()) > dead ||
    Math_Abs(VT13.Get_Right_Y()) > dead)
{
    VT13_Control_Type = VT13_Control_Type_REMOTE;
}
```

这样只有摇杆偏移超过 0.05 时，才认为操作手真的在控制，能减少因为零漂导致的误触发。

### 四、Pitch 闭环方向修正与首次上电限幅

7.3日烧录时，Pitch 上电后出现了猛抽（“三” 中已提过）。当前版本在 `crt_gimbal.cpp` 中把 Pitch 输出方向取反：

```cpp
Target_Torque = -PID_Omega.Get_Out();
```

同时加了首次联调用的输出限幅：

```cpp
Math_Constrain(&Target_Torque, -3000.0f, 3000.0f);
```

这两个改动的目的不同：

- 取反是为了解决 Pitch 闭环方向问题；
- 限幅是为了第一次上电更安全，避免 PID 输出过大导致机构猛抽。

后续如果确认方向稳定、参数合适，可以再根据实际需求逐步放开限幅。

### 五、MiniPC CAN 接收协议更新

之前的版本里，MiniPC 接收结构体是把 yaw、pitch、Fire 等内容放在一个包里解析。和算法艾学长沟通后，优化成了两个轴分开发：

- `0xA3`：yaw 包，要求 `axis = 0`；
- `0xA4`：pitch 包，要求 `axis = 1`。

接收缓存结构体也改成每个轴一份：

```cpp
struct Struct_MiniPC_Axis_Rx_Cache
{
    uint8_t mode;
    uint8_t seq;

    int16_t angle;
    int16_t velocity;
    int16_t acceleration;

    uint8_t new_data;
};
```

新的回调接口带上 CAN ID，便于区分 yaw / pitch 两类包：

```cpp
void Class_MiniPC::CAN_RxCpltCallback(uint32_t can_id, const uint8_t *rx_data)
```

解析逻辑大致为：

1. 先判断 `rx_data` 是否为空；
2. 读取 `rx_data[0]` 的最低位作为 `axis`；
3. `can_id == 0xA3 && axis == 0` 时写入 `Yaw_Rx_Cache`；
4. `can_id == 0xA4 && axis == 1` 时写入 `Pitch_Rx_Cache`；
5. 从 8 字节数据中按小端格式解析角度、速度、加速度；
6. 调用 `Data_Process()` 把弧度制缩放值转换为角度制；
7. 更新 `Fire`、`Control`、`alive` 等状态。

核心解析如下：

```cpp
cache->mode = rx_data[0];
cache->seq = rx_data[1];
cache->angle = MiniPC_Read_Int16_LE(&rx_data[2]);
cache->velocity = MiniPC_Read_Int16_LE(&rx_data[4]);
cache->acceleration = MiniPC_Read_Int16_LE(&rx_data[6]);
```

`Data_Process()` 中统一用 `1 / 10000` 作为缩放，再从弧度转角度：

```cpp
const float scale = 1.0f / 10000.0f;
const float rad_to_deg = 180.0f / PI;

Rx_Angle_Yaw = Yaw_Rx_Cache.angle * scale * rad_to_deg;
Rx_Angle_Pitch = Pitch_Rx_Cache.angle * scale * rad_to_deg;
```

为了调试 MiniPC CAN 是否有进帧，还加了全局观测变量：

```cpp
volatile uint32_t debug_can_id = 0;
volatile uint8_t debug_rx0 = 0;
volatile uint8_t debug_rx1 = 0;
```

**插一嘴：**
代码里关于“等待 yaw 和 pitch 都到达”“比较 seq 是否一致”“比较 mode 中 shoot/control 是否一致”的保护逻辑目前还处于注释状态。
我的想法是，现在版本先让链路跑通（算法有点等不及了），后续还需要把同一组 yaw/pitch 数据的同步校验补回来，避免只收到单轴数据也被当成完整数据使用。

### 六、MiniPC 发送帧调整

在 `drv_can.cpp` 中，MiniPC 发送帧改为在周期发送里通过 CAN1 发 `0xA0`：

```cpp
CAN_Send_Data(&hfdcan1, 0xA0, CAN1_MiniPc_Tx_Data, 8);
```

同时保留了调试计数 / 状态变量：

```cpp
volatile uint32_t debug_a0_tx_count = 0;
volatile uint8_t debug_a0_tx_status = 0xFF;
```

后续如果怀疑 MiniPC 没收到 MCU 发出的反馈帧，可以先确认这两个变量以及 CAN1 上是否能抓到 `0xA0`。

### 七、当前版本的调试结论

这版代码的核心结论可以概括为：

```text
超电链路已按实车复核统一为 CAN3；
CAN3 中补全了超电 0x67 和 MA600 0xD1~0xD4 的派发；
上板 0x77 控制帧接收链路加入了调试变量；
底盘随动 yaw 电机对象从 GM6020 改为 LK；
VT13 遥控器输入判定加入死区；
Pitch 输出方向取反，并加入首次上电限幅；
MiniPC 接收协议改为 0xA3 yaw + 0xA4 pitch 双包解析。
```
