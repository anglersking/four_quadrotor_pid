# four_quadrotor_pid

四旋翼飞行器串级 PID 姿态控制框架，可复用于各类电机控制项目。

## 控制架构

```
┌─────────────────────────────────────────────────┐
│                  遥控器输入                       │
│            (俯仰/横滚/偏航/油门/模式)               │
├─────────────────────────────────────────────────┤
│                  模式选择器                        │
│      手动模式 │ 定高模式 │ 定点模式                 │
├─────────────────────────────────────────────────┤
│              位置环 (P)  ──► 限幅                 │
│              速度环 (PI) ──► 限幅                 │
│              高度环 (P)  ──► 限幅                 │
├─────────────────────────────────────────────────┤
│         姿态外环 (P) ──► 角度控制 ←── MPU6050      │
│              期望角速度                            │
│         姿态内环 (PID) ──► 角速度控制 ←── 陀螺仪    │
├─────────────────────────────────────────────────┤
│            电机混控矩阵 (X型四旋翼)                  │
│   M1 = T + Pitch - Roll + Yaw                   │
│   M2 = T + Pitch + Roll - Yaw                   │
│   M3 = T - Pitch + Roll + Yaw                   │
│   M4 = T - Pitch - Roll - Yaw                   │
├─────────────────────────────────────────────────┤
│          M1      M2      M3      M4              │
│        (电机1)  (电机2)  (电机3)  (电机4)           │
└─────────────────────────────────────────────────┘
```

## PID 串级控制说明

四旋翼采用**串级 PID**：外环控制角度，内环控制角速度。相比单级 PID，串级 PID 响应更快、抗干扰更强。

```
遥控器/期望 ──→ [角度环 P] ──→ [角速度环 PID] ──→ 电机混控 ──→ 四个电机
                   ↑              ↑
              MPU6050 角度    MPU6050 角速度
```

### 15 个 PID 控制器

| 控制器 | 用途 | 控制环 |
|--------|------|--------|
| `pit_angle` | 俯仰角度 | 姿态外环 |
| `rol_angle` | 横滚角度 | 姿态外环 |
| `yaw_angle` | 偏航角度 | 姿态外环 |
| `pit_gyro` | 俯仰角速度 | 姿态内环 |
| `rol_gyro` | 横滚角速度 | 姿态内环 |
| `yaw_gyro` | 偏航角速度 | 姿态内环 |
| `acc_high` | 高度加速度 | 高度外环 |
| `vel_high` | 高度速度 | 高度内环 |
| `pos_high` | 高度位置 | 高度最外环 |
| `acc_fix_x` | X轴加速度 | 定点最内环 |
| `vel_fix_x` | X轴速度 | 定点中间环 |
| `pos_fix_x` | X轴位置 | 定点最外环 |
| `acc_fix_y` | Y轴加速度 | 定点最内环 |
| `vel_fix_y` | Y轴速度 | 定点中间环 |
| `pos_fix_y` | Y轴位置 | 定点最外环 |

## PID 参数

```c
// 姿态外环 (P 控制)
pit_angle: Kp=2.55, IntegralMax=300, OutMax=800
rol_angle: Kp=2.55, IntegralMax=300, OutMax=800
yaw_angle: Kp=3.2,  IntegralMax=300, OutMax=800

// 姿态内环 (PID 控制)
pit_gyro:  Kp=0.45, Ki=0.0015, Kd=0.15, IntegralMax=300, OutMax=800
rol_gyro:  Kp=0.45, Ki=0.0015, Kd=0.15, IntegralMax=300, OutMax=800
yaw_gyro:  Kp=1.25, Ki=0.0025, Kd=0.15, IntegralMax=300, OutMax=800

// 高度控制
vel_high:  Kp=3.1,  IntegralMax=200, OutMax=800
pos_high:  Kp=1.05, IntegralMax=200, OutMax=800

// 水平定点
vel_fix_x/y: Kp=0.08, IntegralMax=200, OutMax=800
pos_fix_x/y: Kp=2.2,  IntegralMax=200, OutMax=800
```

## 飞行模式

### 手动模式
遥控器直驱，俯仰/横滚/偏航/油门由摇杆控制。

### 定高模式
油门中位自动保持高度，上下拨杆升降。高度控制为串级结构：
```
pos_high (P) → vel_high (PI) → 与油门混控输出
```

### 定点模式（需光流传感器）
在定高基础上加入水平位置闭环：
```
pos_fix (P) → vel_fix (PI) → 输出到 rol/pit 期望角度
```

## 电机混控

X型四旋翼的混控矩阵：

```
M1 = 油门 + 俯仰 - 横滚 + 偏航   (前左)
M2 = 油门 + 俯仰 + 横滚 - 偏航   (前右)
M3 = 油门 - 俯仰 + 横滚 + 偏航   (后左)
M4 = 油门 - 俯仰 - 横滚 - 偏航   (后右)
```

## 代码结构

```
pid.h / pid.cpp            # PID 控制器数据结构 + 核心算法
controller.h / controller.cpp  # 飞控逻辑 (模式选择/姿态/高度/定点)
test.cpp                   # 测试程序 (VS 控制台验证)
```

### PID 核心实现

```cpp
float pid_controller(_PID* controller) {
    controller->err = controller->expect - controller->feedback;
    controller->integral += controller->ki * controller->err;
    // 积分限幅
    if (controller->integral > controller->integral_max)
        controller->integral = controller->integral_max;
    // PID 输出 = P + I + D
    controller->out = controller->kp * controller->err
        + controller->integral
        + controller->kd * (controller->err - controller->err_last);
    // 输出限幅
    if (controller->out > controller->out_max)
        controller->out = controller->out_max;
    return controller->out;
}
```

## 构建

### Windows (Visual Studio)

```bash
# 打开 test.sln 用 VS 编译运行
# 或命令行
msbuild test.sln /p:Configuration=Release
```

### 移植到嵌入式平台

代码为平台无关的 C++，核心 `pid.cpp` / `controller.cpp` 可直接移植到：
- STM32 飞控 (Keil/IAR/GCC)
- ESP32 无人机
- Pixhawk / ArduPilot 自定义控制

仅需替换传感器读取和电机输出函数。

## 调试

```bash
test.exe
```

输出示例：

```
PID_CONTROL_FOUR_MOTOR_MODE
定高模式
unlock
fly
Motor 1:145,2:255 3:355 4:445
...
```

## 调参经验

1. **先调内环后调外环**：姿态内环 (角速度 PID) 稳定后，再调整姿态外环 (角度 P)
2. **外环只用 P**：角度外环不需要积分 (I)，飞行器有自稳特性
3. **D 项用角速度**：角速度环的 D 项代替角加速度计算，避免微分噪声
4. **输出限幅**：每个 PID 输出都有限幅，防止积分饱和和电机饱和
5. **电机不转时清积分**：未解锁或油门太低时调用 `clear_integral()` 清除所有积分项

## PID 控制器通用性

此 PID 实现是**纯 C 结构体**，与平台无关，可在以下场景复用：
- 无人机/四旋翼姿态控制
- 平衡车直立控制
- 机器人电机速度/位置控制
- 温度/压力等过程控制

核心思想：**15 个并行 PID 控制器 + 串级互联 = 四旋翼完整飞控**。

## 关联项目

- [ROS_HARDWARE](https://github.com/anglersking/ROS_HARDWARE) — 小车 STM32 底层驱动，同样使用 PID 控制
- 此框架的设计思路与小车 PID 一脉相承，可互相对照学习
