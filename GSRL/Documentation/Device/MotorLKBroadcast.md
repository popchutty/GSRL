# 瓴控广播模式(一控四)电机驱动使用说明

作者：yuki  
版本：v1.1.0  
最后更新日期：2026-11-13  
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

### 已实测的范围

本类按广播协议 V2.35 编码了全部四种帧，但**不是每款电机固件都支持全部命令**：
厂家《MG60–100 使用手册 V2.38》写明广播模式只支持扭矩控制，而广播协议 V2.35 另外列出了速度、位置和混合命令。
使用前请在自己的电机上逐项确认。

2026-11-12、11-13 在单台 MG8016E-i6 V3（减速比 6、驱动固件 V2.36、ID 1、CAN1 1Mbps）上的结果：

| 项目 | 结果 |
| --- | --- |
| 上位机开启广播模式，断电重启 | 设置保持 |
| `0x280` 全零力矩帧，2 秒内发 1000 帧 | 1000 帧全部收到回复，回复帧 ID 为 `0x141`，命令字节 `0xA1` |
| `0x280` 全零力矩帧，500Hz 持续 120 秒 | 发送、回复各 59980 帧，发送失败 0 |
| 零力矩下只给电机断电再上电 | 断电时 `isMotorConected` 变为 false，上电后恢复 |
| 零力矩下手转输出轴约一圈 | `getCurrentRevolutions` 增加约 1.02 圈，即**角度反馈对应输出轴**（粗测，未精确对准） |
| `0x280` 力矩 +10 / −10，各 50 ms | 反馈位移符号相反，幅度很小，肉眼看不出转动 |
| `0x280` 力矩 +25，最长 200 ms | 电机明显转动；80 ms 时转速原始反馈达到 308 dps，被测试程序的转速保护提前结束 |
| `0x280` 力矩 −25，200 ms | 转速原始反馈峰值 49 dps，位移为负，肉眼未见转动；正反向差异的原因未定位 |
| 转速反馈对应电机轴还是输出轴 | 未单独测量 |
| 广播模式开启时发 `0x288` 状态查询 | 无回复；该固件下 `getBusVoltage`、`getErrorState`、`isMotorOn` 不会更新，`hasReceivedState1()` 一直为 false |
| `0x281` 速度、`0x282` 位置 | 未测试 |
| 四台电机同时运行 | 未测试，合帧只有主机端验证 |

上表的转速、位移是电机反馈值，没有用外部仪器标定。测试中上电初期温度反馈出现过一次短暂的异常值（124），来源未定位。

## 3：协议要点

控制帧为标准数据帧，DLC = 8，每个电机占 2 字节，**低字节在前**：

| 电机 ID | 1 | 2 | 3 | 4 |
| --- | --- | --- | --- | --- |
| 数据槽位 | D[0:1] | D[2:3] | D[4:5] | D[6:7] |

| 帧 ID | 命令 | 槽位内容 | 本类对应方法 |
| --- | --- | --- | --- |
| 0x280 | 力矩控制 | int16，-2000~2000 | `openloopControl` 及基类所有软件闭环方法 |
| 0x281 | 速度控制 | int16，1dps/LSB，-32768~32767 | `hardwareAngularVelocityClosedloopControl` |
| 0x282 | 位置控制 | int16，0.01°/LSB，绝对位置，-32768~32767 | `hardwareAngleClosedloopControl` |
| 0x288 | 混合命令 | 低字节为命令字节，高字节为 0x00 | `setMotorCommand` |

混合命令支持：`READ_STATE_1`(0x9A)、`CLEAR_ERROR`(0x9B)、`READ_STATE_2`(0x9C)、`MOTOR_OFF`(0x80)、`MOTOR_ON`(0x88)、`MOTOR_STOP`(0x81)。

电机收到广播帧后按 ID 从低到高依次回复，回复的数据格式与单电机模式对应命令的回复相同：

| 回复命令字节 D[0] | 内容 | 更新的状态 |
| --- | --- | --- |
| 0x9C、0xA0~0xA8 | 温度、转矩电流、转速(dps)、编码器 | 角度、角速度、转矩电流、温度 |
| 0x9A、0x9B | 温度、母线电压、母线电流、电机状态、错误标志 | 温度、`getBusVoltage`、`getBusCurrent`、`isMotorOn`、`getErrorState`，并使 `hasReceivedState1()` 变为 true |
| 0x80、0x88 | 与发送相同 | `isMotorOn` |
| 0x81 | 与发送相同 | 无，只用于确认电机在线 |

**反馈帧 ID 以电机实际回复为准**：实测 MG8016E-i6 V3（固件 V2.36）回复 0x140 + ID，与本库 `MotorLKMG` 一致；
部分协议文档和固件配置使用 0x180 + ID。构造函数参数 `feedbackBaseID` 默认 `FEEDBACK_BASE_0x140`，
只有确认电机回复 0x180 + ID 时才传入 `FEEDBACK_BASE_0x180`。
选错时电机会执行控制命令，但所有反馈都被拒收，软件闭环会一直使用旧的反馈值。
不确定时在 CAN 接收回调中观察收到的 `StdId` 即可判断。

**编码器位数与电机型号有关**：协议允许 14、15、16 位三种，构造函数参数 `encoderResolution` 用于选择，默认 `ENCODER_16BIT`。
选错时角度和圈数都会出错，例如 14 位编码器按 16 位解析，半圈会被算成 45°。

## 4：必须了解的特性

- **广播帧没有"空槽位"。** 总线上每个在线电机都会执行帧中自己槽位的值，未写入的槽位为 0：
  力矩帧中是 0 力矩，速度帧中是 0 速度，**位置帧中是转到 0° 位置**，混合命令帧中是无命令。
  所以位置模式下必须给总线上所有电机都设置目标。
- **同一帧内的电机必须处于同一种控制模式。** 每个电机对象的控制帧 ID 由它最后调用的控制方法决定。
- **用 `buildControlFrame` 合帧。** 用 `activeMask` 声明总线上参与广播控制的全部电机（bit0~bit3 对应 ID 1~4），
  传入的电机必须与之一一对应且控制模式一致。电机不齐、模式不一致、ID 重复或非法、指针为空时返回 false，
  并把帧头和数据清零，不会产生部分帧；返回 false 时不要发送。
  它每个控制周期只调用一次：调用时会触发各电机的掉线检测。
- **控制量自动限幅。** 力矩限幅到 ±2000，速度、位置限幅到 -32768~32767，NaN 按 0 处理。
  基类各闭环方法的返回值是限幅取整后实际发送的力矩控制量。
- **硬件位置闭环的范围很小。** 16 位、0.01°/LSB，电机轴角度范围只有 -327.68°~327.67°，带减速箱时输出轴范围还要除以减速比。
  需要更大范围时请使用基类的软件闭环（力矩帧）。
- **硬件位置闭环和软件角度闭环的目标互不相通。** `hardwareAngleClosedloopControl(targetAngle)` 的目标以电机自身零点为基准，
  并按减速比换算到电机轴；基类 `angleClosedloopControl` 的目标是以 `encoderOffset` 为零点的编码器单圈角度 [0, 2π)。
  两者分开保存，切换控制方式时需要重新给出目标。
- **减速比。** `gearboxRatio` 用于换算角速度反馈和硬件闭环的目标值；角度反馈直接使用回传的编码器值，不做减速比换算，与 `MotorLKMG` 一致。
  编码器值对应电机轴还是输出轴取决于电机：实测 MG8016E-i6 V3 手转输出轴一圈，角度反馈约一圈，即对应输出轴；角速度反馈的轴侧未单独测量。
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
// 反馈基地址和编码器位数必须与实际电机一致, 以电机实际回复的帧ID为准
MotorLKBroadcast lk1(1, &pid1, 0, 1.0f, MotorLKBroadcast::FEEDBACK_BASE_0x140, MotorLKBroadcast::ENCODER_16BIT);
MotorLKBroadcast lk2(2, &pid2, 0, 1.0f, MotorLKBroadcast::FEEDBACK_BASE_0x140, MotorLKBroadcast::ENCODER_16BIT);
MotorLKBroadcast lk3(3, &pid3, 0, 1.0f, MotorLKBroadcast::FEEDBACK_BASE_0x140, MotorLKBroadcast::ENCODER_16BIT);
MotorLKBroadcast lk4(4, &pid4, 0, 1.0f, MotorLKBroadcast::FEEDBACK_BASE_0x140, MotorLKBroadcast::ENCODER_16BIT);
MotorLKBroadcast *const motors[] = {&lk1, &lk2, &lk3, &lk4};

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
        // 0x0F: 总线上参与广播控制的电机是 ID1~ID4
        CAN_TxHeaderTypeDef header;
        uint8_t data[8];
        if (MotorLKBroadcast::buildControlFrame(motors, 4, 0x0F, header, data)) {
            uint32_t mailbox;
            HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
        }
        // 返回false说明有电机的控制模式不一致或ID配置错误, 属于程序错误, 不发送

        vTaskDelayUntil(&taskLastWakeTime, 2); // 500Hz, 500Kbps和1Mbps下均可用
    }
}
```

电机数量少于 4 个时只传实际接入的电机，并让 `activeMask` 与之对应，例如只有 ID1 和 ID2 时传 2 个电机、`0x03`。

### 5.2 使用电机内置的速度环、位置环

```cpp
MotorLKBroadcast *const motors[] = {&lk1, &lk2};
CAN_TxHeaderTypeDef header;
uint8_t data[8];

lk1.hardwareAngularVelocityClosedloopControl(3.14f); // rad/s
lk2.hardwareAngularVelocityClosedloopControl(-3.14f);
// 0x03: 总线上参与广播控制的电机是 ID1 和 ID2; 帧ID为0x281
if (MotorLKBroadcast::buildControlFrame(motors, 2, 0x03, header, data)) {
    HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}
```

位置环使用 `hardwareAngleClosedloopControl(targetAngle)`，单位 rad，带符号的绝对位置，合帧方式相同（帧 ID 为 0x282）。
位置帧中缺失的槽位会让对应电机转向 0° 位置，所以 `activeMask` 必须包含总线上的全部电机。

硬件速度、位置模式的目标按减速比换算到电机轴。新版协议 V2.40 把广播速度、位置目标定义在输出轴，
不同固件可能不一致，使用前先用小目标值在实物上确认转过的角度。

### 5.3 读取错误标志、清除错误、开关电机

```cpp
// 低频查询状态1: 该周期用混合命令帧代替控制帧
lk1.setMotorCommand(MotorLKBroadcast::READ_STATE_1);
lk2.setMotorCommand(MotorLKBroadcast::READ_STATE_1);
if (MotorLKBroadcast::buildControlFrame(motors, 2, 0x03, header, data)) { // 帧ID为0x288
    HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}

// 之后在任意位置读取; 没收到过状态1回复时错误标志仍是初始值0
if (lk1.hasReceivedState1() && (lk1.getErrorState() & MotorLKBroadcast::ERROR_STALL)) {
    // 堵转处理
}
```

`setMotorCommand` 之后再次调用任一控制方法，该电机的控制帧 ID 会自动切换回对应的控制帧。
部分固件在广播模式下不回复混合命令（见第 2 节实测结果），此时错误标志、母线电压和开关状态保持初始值，`hasReceivedState1()` 为 false。

## 6：常见问题

| 现象 | 可能原因 |
| --- | --- |
| 电机不动，`isMotorConected()` 为 false | 未开启广播模式；波特率不一致；反馈基地址选错（0x140 / 0x180） |
| 电机会动但 `isMotorConected()` 为 false | 反馈基地址选错；接收回调中没有调用该电机的解析函数 |
| `buildControlFrame` 返回 false | 传入的电机控制模式不一致；电机 ID 重复或超出 1~4；传入的电机与 `activeMask` 不一一对应 |
| 角度只在一小段范围内变化，圈数不累加 | 编码器位数选错 |
| 进入位置模式后其他电机突然转动 | 位置帧中未设置目标的电机槽位为 0，会转到 0° 位置 |
| 反馈时断时续 | 发送频率超过协议上限；总线负载过高 |
