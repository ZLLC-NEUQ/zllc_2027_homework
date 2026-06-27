# 征龙凌沧27赛季嵌入式培训

## FreeRTOS 基础知识

### 1. 什么是 FreeRTOS
FreeRTOS 是一个专为嵌入式系统设计的实时操作系统（RTOS），具有开源、轻量级、可移植等特点。

### 2. 核心概念

#### 任务（Task）
- FreeRTOS 中的基本执行单元
- 每个任务都有自己的栈空间和优先级
- 状态：运行态、就绪态、阻塞态、挂起态

#### 任务调度器（Scheduler）
- 负责决定哪个任务在何时运行
- 基于优先级进行抢占式调度
- 相同优先级的任务采用时间片轮转

#### 队列（Queue）
- 用于任务间通信
- 支持不同类型数据的传递
- 遵循 FIFO 原则

#### 信号量（Semaphore）
- **二值信号量**：用于同步或互斥
- **计数信号量**：用于资源管理
- **互斥量**：支持优先级继承的互斥访问

### 3. 常用 API

```c
// 创建任务
xTaskCreate(TaskFunction_t pvTaskCode, const char *pcName, uint16_t usStackDepth, void *pvParameters, UBaseType_t uxPriority, TaskHandle_t *pxCreatedTask);

// 启动调度器
vTaskStartScheduler();

// 延时函数
vTaskDelay(TickType_t xTicksToDelay);
```

### 4. 内存管理
- FreeRTOS 提供多种内存分配策略
- 静态分配：任务栈由用户预先分配
- 动态分配：任务栈由堆内存自动分配

## Booster 发射机构学习记录
😴😴😴😭😭😭😴😴😴😭😭😭😴😴😴😭😭😭😴😴😴😭😭😭😴😴😴
### 1. 概述

Booster 是机器人发射机构的核心控制模块，负责管理摩擦轮（发射子弹的动力源）和拨弹盘（将子弹送入发射通道）的电机控制。系统采用双有限状态机（FSM）设计，分别处理**热量检测**和**卡弹处理**。

### 2. 文件结构

| 文件 | 作用 |
|------|------|
| [crt_booster.cpp](file:///e:/gitgit/zllc_2027_homework/User/Chariot/Src/crt_booster.cpp) | 核心实现，包含状态机、输出控制 |
| [crt_booster.h](file:///e:/gitgit/zllc_2027_homework/User/Chariot/Inc/crt_booster.h) | 类定义、控制类型枚举、阈值参数 |

### 3. 核心控制类型

#### 3.1 发射机构控制类型

```c
enum Enum_Booster_Control_Type
{
    Booster_Control_Type_DISABLE = 0,  // 失能
    Booster_Control_Type_CEASEFIRE,    // 停火
    Booster_Control_Type_SINGLE,       // 单发
    Booster_Control_Type_REPEATED,     // 连发
    Booster_Control_Type_MULTI         // 多发
};
```

#### 3.2 摩擦轮控制类型

```c
enum Enum_Friction_Control_Type
{
    Friction_Control_Type_DISABLE = 0,  // 停止
    Friction_Control_Type_ENABLE       // 开启
};
```

### 4. 电机配置

发射机构包含四个电机：

| 电机 | 型号 | CAN ID | 控制模式 | PID 参数 |
|------|------|--------|----------|----------|
| 拨弹盘 Motor_Driver | C610 | 0x201 (CAN2) | OMEGA/ANGLE | Angle: 40,0.1,0 / Omega: 6000,40,0 |
| 摩擦轮左 Motor_Friction_Left | C620 | 0x202 (CAN1) | OMEGA | 150,4,0.2 |
| 摩擦轮右 Motor_Friction_Right | C620 | 0x201 (CAN1) | OMEGA | 150,4,0.2 |
| 摩擦轮下 Motor_Friction_Right | C620 | 0x201 (CAN1) | OMEGA | 150,4,0.2 |

**关键参数**：
- `Friction_Omega = 1000 rad/s`（摩擦轮目标角速度）
- `Default_Driver_Omega = 2.5f * 2.0f * PI / 9.0f * 25 rad/s`（拨弹盘默认速度）
- `Driver_Angle` 每次增加 `2π/9` 弧度（一圈 8 发子弹）

### 5. 热量管理

#### 5.1 热量检测有限状态机（FSM_Heat_Detect）

状态机包含 4 个状态：

```
状态0(正常) → 检测到摩擦轮大扭矩 → 状态1(发射嫌疑)
状态1 → 超时(45周期) → 状态2(确认发射)
状态2 → 摩擦轮开关切换 → 状态0 + Heat+=10
状态2 → 正常发射完成 → 状态0 + actual_bullet_num++
状态0 → 停机命令 → 状态3(停机)
状态3 → 摩擦轮启动 → 状态0
```

**热量计算**：
- 每次发射增加 `Heat = 10`
- 热量冷却：`Heat -= Referee->Get_Booster_17mm_1_Heat_CD() / 1000`

#### 5.2 热量预测与控制参数

```c
float Heat_Max = 400.0f;           // 最大热量
float Cooling_Value = 10.0f;       // 冷却值
float Tau0 = 0.45f;                // 提前收缩时间
float Tau1 = 0.0965f;              // 收缩陡度
float Recover_Ratio = 0.85f;       // 恢复比例
```

### 6. 卡弹处理有限状态机（FSM_Antijamming）

状态机包含 5 个状态，处理拨弹盘卡弹情况：

```
状态0(正常) → 检测到大扭矩(Torque>6000) → 状态1(卡弹嫌疑)
状态1 → 超时(100周期) → 状态2(卡弹反应)
状态1 → 扭矩恢复正常 → 状态0
状态2 → 设置拨弹盘反转角度 → 状态3(卡弹处理)
状态3 → 扭矩正常持续200周期 → 状态0(正常)
状态3 → 扭矩异常持续400周期 → 状态4(失能)
状态4 → 扭矩恢复正常 → 状态1
```

**卡弹处理策略**：
- 拨弹盘反转 `2π/8` 弧度尝试解除卡弹
- 若持续卡死超过 400 周期，进入失能状态

### 7. 控制输出（Output）

根据 `Booster_Control_Type` 执行不同控制策略：

| 控制类型 | 摩擦轮 | 拨弹盘 | 说明 |
|----------|--------|--------|------|
| DISABLE | 停止(0) | 失能(开环) | 完全停止 |
| CEASEFIRE | 保持 | 速度归零 | 停火但保持摩擦轮转速 |
| SINGLE | 开启 | 角度增量 2π/9 | 单发一次 |
| MULTI | 开启 | 角度增量 5×2π/9 | 五连发 |
| REPEATED | 开启 | 恒定速度拨弹 | 持续连发，热量不足时停火 |

### 8. 关键阈值

```c
uint16_t Driver_Torque_Threshold = 6000;    // 拨弹盘堵转扭矩阈值
uint16_t Friction_Torque_Threshold = 1900;  // 摩擦轮判定发射阈值
float Friction_Omega_Threshold = 600;       // 摩擦轮启动判定阈值
```

### 9. 定时处理（TIM_Calculate_PeriodElapsedCallback）

每个定时器周期执行：
1. `FSM_Heat_Detect` 热量检测状态机
2. `FSM_Antijamming` 卡弹处理状态机
3. `Output()` 输出控制
4. 四个电机的 PID 计算
5. 困😒😒😒😒😒😒😒😒😒😒😒😒😒😒😒😒😒😒😒😒😒😒😒😒😒😒
