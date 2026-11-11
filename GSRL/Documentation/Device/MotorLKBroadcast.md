# 瓴控广播模式(一控四)电机驱动使用说明

作者：yuki  
版本：v1.0.0  
最后更新日期：2026-11-11  
对应协议：《瓴控科技电机 CAN 广播通讯协议 V2.35》《电机 CAN 总线通讯协议 V2.36》

---

## 1：概述

`MotorLKBroadcast` 用于控制开启了"广播模式"的瓴控 MF/MG 系列电机。广播模式下一帧 CAN 报文同时携带 4 个电机的控制量，
作用与大疆电机的一控四、达妙电机的一控四固件相同。

与单电机模式的 `MotorLKMG` 相比：

| | 单电机模式 `MotorLKMG` | 广播模式 `MotorLKBroadcast` |
| --- | --- | --- |
| 控制帧 | 每个电机一帧，ID = 0x140 + 电机ID | 4 个电机共用一帧，帧 ID 即命令类型 |
| 命令区分 | 数据段 D[0] 命令字节 | 帧 ID：0x280 / 0x281 / 0x282 / 0x288 |
| 电机 ID | 1~32 | 1~4 |
| 控制 4 个电机每周期的报文数 | 4 发 + 4 收 | 1 发 + 4 收 |
| 抱闸器命令 | 支持 | 不支持 |

## 2：使用前的电机配置

1. 在瓴控上位机中开启广播模式；
2. CAN 波特率设置为 500Kbps 或 1Mbps，并与主控一致；
3. 电机 ID 设置为 1~4，同一条总线上不能重复（拨码开关或上位机设置）。

命令最大发送频率（同时控制 4 个电机）：500Kbps 下约 600Hz，1Mbps 下约 1.2KHz。

## 3：协议要点

控制帧为标准数据帧，DLC = 8，每个电机占 2 字节，**低字节在前**：

| 电机 ID | 1 | 2 | 3 | 4 |
| --- | --- | --- | --- | --- |
| 数据槽位 | D[0:1] | D[2:3] | D[4:5] | D[6:7] |

| 帧 ID | 命令 | 槽位内容 | 本类对应方法 |
| --- | --- | --- | --- |
| 0x280 | 力矩控制 | int16，-2000~2000 | `openloopControl` 及基类所有软件闭环方法 |
| 0x281 | 速度控制 | int16，1dps/LSB，对称限幅 ±32767 | `hardwareAngularVelocityClosedloopControl` |
| 0x282 | 位置控制 | int16，0.01°/LSB，绝对位置，对称限幅 ±32767 | `hardwareAngleClosedloopControl` |
| 0x288 | 混合命令 | 低字节为命令字节，高字节为 0x00 | `setMotorCommand` |

混合命令支持：`READ_STATE_1`(0x9A)、`CLEAR_ERROR`(0x9B)、`READ_STATE_2`(0x9C)、`MOTOR_OFF`(0x80)、`MOTOR_ON`(0x88)、`MOTOR_STOP`(0x81)。

电机收到广播帧后按 ID 从低到高依次回复，回复的数据格式与单电机模式对应命令的回复相同：

| 回复命令字节 D[0] | 内容 | 更新的状态 |
| --- | --- | --- |
| 0x9C、0xA0~0xA8 | 温度、转矩电流、转速(dps)、编码器 | 角度、角速度、转矩电流、温度 |
| 0x9A、0x9B | 温度、母线电压、母线电流、电机状态、错误标志 | 温度、`getBusVoltage`、`getBusCurrent`、`isMotorOn`、`getErrorState` |
| 0x80、0x88 | 与发送相同 | `isMotorOn` |
| 0x81 | 与发送相同 | 无，只用于确认电机在线 |

**反馈帧 ID 与固件协议版本有关**：协议 V2.36 为 0x180 + ID，本库 `MotorLKMG` 使用的是 0x140 + ID。
构造函数参数 `feedbackBaseID` 用于选择，默认 `FEEDBACK_BASE_0x140`，协议 V2.36 的固件需显式传入 `FEEDBACK_BASE_0x180`。
选错时电机会执行控制命令，但所有反馈都被拒收，软件闭环会一直使用旧的反馈值。
不确定时在 CAN 接收回调中观察收到的 `StdId` 即可判断。

**编码器位数与电机型号有关**：协议允许 14、15、16 位三种，构造函数参数 `encoderResolution` 用于选择，默认 `ENCODER_16BIT`。
选错时角度和圈数都会出错，例如 14 位编码器按 16 位解析，半圈会被算成 45°。

## 4：必须了解的特性

- **广播帧没有"空槽位"。** 总线上每个在线电机都会执行帧中自己槽位的值，未写入的槽位为 0：
  力矩帧中是 0 力矩，速度帧中是 0 速度，**位置帧中是转到 0° 位置**，混合命令帧中是无命令。
  所以位置模式下必须给总线上所有电机都设置目标。
- **同一帧内的电机必须处于同一种控制模式。** 每个电机对象的控制帧 ID 由它最后调用的控制方法决定；
  合帧时以调用者的帧 ID 为准，帧 ID 不一致、ID 非法或重复的电机会被跳过，`isLastMergeComplete()` 返回 false。
  此时返回的仍是一帧可发送的部分数据，被跳过的电机槽位为 0（ID 重复时保留先写入的值），是否发送由调用者决定。
  `isLastMergeComplete()` 只表示传入的电机是否全部写入，不表示 4 个槽位齐全，也不表示发送成功。
- **控制、合帧、发送要在同一个任务里依次完成。** 合帧得到的数据和 `getMotorControlHeader()` 的帧 ID 是分两次取的，
  中间如果有别的任务切换了电机的控制模式，会把旧数据配上新的帧 ID 发出去。
- **电机 ID 只能是 1~4。** 传入其他值时对象失效：不写入控制数据、不解析反馈、不参与合帧，`isMotorIDValid()` 返回 false。
- **控制量自动限幅。** 力矩限幅到 ±2000，速度、位置对称限幅到 ±32767，NaN 按 0 处理。
  基类各闭环方法的返回值是限幅取整后实际发送的力矩控制量。
- **硬件位置闭环的范围很小。** 16 位、0.01°/LSB，电机轴角度范围只有 ±327.67°，带减速箱时输出轴范围还要除以减速比。
  需要更大范围时请使用基类的软件闭环（力矩帧）。
- **硬件位置闭环和软件角度闭环的目标互不相通。** `hardwareAngleClosedloopControl(targetAngle)` 的目标以电机自身零点为基准，
  并按减速比换算到电机轴；基类 `angleClosedloopControl` 的目标是以 `encoderOffset` 为零点的电机轴单圈角度 [0, 2π)。
  两者分开保存，切换控制方式时需要重新给出目标。
- **减速比。** `gearboxRatio` 用于换算角速度反馈和硬件闭环的目标值，角度反馈为电机轴编码器角度，不受减速比影响，与 `MotorLKMG` 一致。
  传入 0、负数、NaN 或无穷大时按 1 处理。
- **中断与任务之间没有加锁。** 反馈在 CAN 接收中断里更新，任务里读到的角度、角速度等不保证来自同一帧，
  掉线计数也可能偶尔多计一次（阈值为 10，下一帧反馈会清零）。这与本库其他电机类一致；
  需要严格一致时请使用 `decodeCanRxMessageFromQueue` 在任务中解析。
- 本类不支持 MS 系列电机（开环电压范围 ±850）。

## 5：典型用法

### 5.1 四个电机，主控做闭环，电机走力矩帧（推荐）

```cpp
#include "dvc_motor.hpp"

SimplePID::PIDParam speedParam = {20.0f, 0.0f, 0.0f, 500.0f, 0.0f};
SimplePID pid1(SimplePID::PID_POSITION, speedParam);
SimplePID pid2(SimplePID::PID_POSITION, speedParam);
SimplePID pid3(SimplePID::PID_POSITION, speedParam);
SimplePID pid4(SimplePID::PID_POSITION, speedParam);

// 参数: 电机ID, 控制器, 编码器偏移量, 减速比, 反馈基地址, 编码器位数
// 反馈基地址和编码器位数必须与实际电机一致, 协议V2.36固件使用 FEEDBACK_BASE_0x180
MotorLKBroadcast lk1(1, &pid1, 0, 1.0f, MotorLKBroadcast::FEEDBACK_BASE_0x180, MotorLKBroadcast::ENCODER_16BIT);
MotorLKBroadcast lk2(2, &pid2, 0, 1.0f, MotorLKBroadcast::FEEDBACK_BASE_0x180, MotorLKBroadcast::ENCODER_16BIT);
MotorLKBroadcast lk3(3, &pid3, 0, 1.0f, MotorLKBroadcast::FEEDBACK_BASE_0x180, MotorLKBroadcast::ENCODER_16BIT);
MotorLKBroadcast lk4(4, &pid4, 0, 1.0f, MotorLKBroadcast::FEEDBACK_BASE_0x180, MotorLKBroadcast::ENCODER_16BIT);

extern "C" void can1RxCallback(can_rx_message_t *pRxMsg)
{
    lk1.decodeCanRxMessageFromISR(pRxMsg);
    lk2.decodeCanRxMessageFromISR(pRxMsg);
    lk3.decodeCanRxMessageFromISR(pRxMsg);
    lk4.decodeCanRxMessageFromISR(pRxMsg);
}

extern "C" void chassis_task(void *argument)
{
    CAN_Init(&hcan1, can1RxCallback);
    TickType_t taskLastWakeTime = xTaskGetTickCount();
    while (1) {
        lk1.angularVelocityClosedloopControl(targetSpeed[0]);
        lk2.angularVelocityClosedloopControl(targetSpeed[1]);
        lk3.angularVelocityClosedloopControl(targetSpeed[2]);
        lk4.angularVelocityClosedloopControl(targetSpeed[3]);

        // 合帧: 一帧发出4个电机的力矩, 同时触发4个电机的掉线检测
        const uint8_t *data = lk1.getMergedControlData(lk2, lk3, lk4);
        if (lk1.isLastMergeComplete()) {
            uint32_t mailbox;
            HAL_CAN_AddTxMessage(&hcan1, lk1.getMotorControlHeader(), data, &mailbox);
        }
        // 合帧不完整说明有电机的控制模式不一致或ID配置错误, 属于程序错误, 这里选择不发送

        vTaskDelayUntil(&taskLastWakeTime, 2); // 500Hz, 500Kbps和1Mbps下均可用
    }
}
```

电机数量少于 4 个时使用对应的重载：`lk1.getMergedControlData(lk2)`、`lk1.getMergedControlData(lk2, lk3)`。
只有一个电机时直接使用 `lk1.getMotorControlData()`。

### 5.2 使用电机内置的速度环、位置环

```cpp
lk1.hardwareAngularVelocityClosedloopControl(3.14f); // rad/s
lk2.hardwareAngularVelocityClosedloopControl(-3.14f);
const uint8_t *data = lk1.getMergedControlData(lk2);  // 帧ID为0x281
HAL_CAN_AddTxMessage(&hcan1, lk1.getMotorControlHeader(), data, &mailbox);
```

位置环同理，使用 `hardwareAngleClosedloopControl(targetAngle)`，单位 rad，带符号的绝对位置。
位置帧中缺失的槽位会让对应电机转向 0° 位置，发送前务必检查 `isLastMergeComplete()`。

### 5.3 读取错误标志、清除错误、开关电机

```cpp
// 低频查询状态1: 该周期用混合命令帧代替控制帧
lk1.setMotorCommand(MotorLKBroadcast::READ_STATE_1);
lk2.setMotorCommand(MotorLKBroadcast::READ_STATE_1);
const uint8_t *data = lk1.getMergedControlData(lk2);  // 帧ID为0x288
HAL_CAN_AddTxMessage(&hcan1, lk1.getMotorControlHeader(), data, &mailbox);

// 之后在任意位置读取
if (lk1.getErrorState() & MotorLKBroadcast::ERROR_STALL) {
    // 堵转处理
}
```

`setMotorCommand` 之后再次调用任一控制方法，该电机的控制帧 ID 会自动切换回对应的控制帧。

## 6：常见问题

| 现象 | 可能原因 |
| --- | --- |
| 电机不动，`isMotorConected()` 为 false | 未开启广播模式；波特率不一致；反馈基地址选错（0x140 / 0x180） |
| 电机会动但 `isMotorConected()` 为 false | 反馈基地址选错；接收回调中没有调用该电机的解析函数 |
| `isLastMergeComplete()` 为 false，某个电机不动、执行 0 目标或执行了别的对象的目标 | 该电机与调用者的控制模式不一致；电机 ID 重复或超出 1~4 |
| 角度只在一小段范围内变化，圈数不累加 | 编码器位数选错 |
| 进入位置模式后其他电机突然转动 | 位置帧中未设置目标的电机槽位为 0，会转到 0° 位置 |
| 反馈时断时续 | 发送频率超过协议上限；总线负载过高 |
