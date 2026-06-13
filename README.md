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