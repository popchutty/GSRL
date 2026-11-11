# 瓴控 MG 电机广播控制

本类依据 CAN 广播通讯协议 V2.35 和 CAN 总线通讯协议 V2.36，实现 MG 系列电机的一控四广播模式。MF、MS 系列不在本类的支持范围内。

## 使用前提

通过厂商上位机开启广播模式，将波特率设为 500kbps 或 1Mbps。同一 CAN 总线的电机 ID 必须为互不重复的 1～4。

`MotorLKMGmulti` 直接继承 `Motor`，不会自动发送 CAN 报文，也不会修改电机的持久化配置。

## 构造与单位

```cpp
MotorLKMGmulti motor(id, controller, encoderOffset, encoderBits, gearboxRatio);
```

- `encoderOffset` 默认为 0，单位为编码器计数。
- `encoderBits` 默认为 16，可选 14、15、16，必须与设备一致。
- `gearboxRatio` 默认为 1，必须为正的有限数，表示物理减速比。反馈角速度除以该值，硬件速度指令乘以该值；编码器角度、位置指令不进行减速比换算。
- `isConfigurationValid()` 查询构造参数是否有效。无效配置不能组帧或解析反馈。
- 软件闭环需要有效的 `Controller`；直接开环及硬件闭环允许传入 `nullptr`。

## 控制接口

| 接口 | CAN ID | 协议单位与范围 |
| --- | --- | --- |
| `openloopControl(value)` 及继承的软件闭环方法 | `0x280` | MG 力矩控制量，−2000～2000 |
| `hardwareAngularVelocityClosedloopControl(radPerSecond)` | `0x281` | 每 LSB 为 1°/s，有符号 16 位 |
| `hardwareAngleClosedloopControl(radians)` | `0x282` | 每 LSB 为 0.01°，−327.68°～327.67° |
| `setManagementCommand(command)` | `0x288` | 每台一个命令字节，随后一个零字节 |

两个硬件控制接口返回 `bool`。无效配置、非有限输入或换算溢出返回 `false`，并保留上一命令；有限超范围值按协议限幅后四舍五入。位置接口表示绝对位置，不是单圈方向控制或无限多圈控制。

力矩控制量使用原始协议单位，不是安培。非有限力矩输出生成零控制量。

管理命令枚举 `MotorLKMGmulti::Management` 包含：

- `ReadStatus1`：读取状态和错误标志，`0x9A`。
- `ClearError`：清除错误标志，`0x9B`。
- `ReadStatus2`：读取温度、电流、速度、编码器，`0x9C`。
- `Disable`：关闭电机，`0x80`。
- `Enable`：开启电机，`0x88`。
- `Stop`：停止电机，`0x81`。

## 四台组帧

```cpp
MotorLKMGmulti motor1(1, nullptr);
MotorLKMGmulti motor3(3, nullptr);
motor1.openloopControl(100);
motor3.openloopControl(-100);

MotorLKMGmulti *motors[] = {&motor1, &motor3};
CAN_TxHeaderTypeDef header{};
uint8_t data[8]{};

if (MotorLKMGmulti::composeControlFrame(motors, 2, header, data)) {
    // ID = 0x280, data = 64 00 00 00 9C FF 00 00
    // 使用项目的 CAN_Send_Data(hcan, &header, data) 发送。
}
```

每个列表只能包含同一 CAN 总线上的电机，由调用方保证总线归属。列表顺序不影响槽位；槽位偏移为 `(ID - 1) * 2`，数据采用小端字节序。

同一帧必须使用同一种控制类型。`0x288` 的“混合”只表示允许不同管理指令，不允许混合力矩、速度和位置控制。

以下情况返回 `false`，不修改输出帧，也不触发连接检查：空列表、数量为 0 或大于 4、空指针、无效配置、重复电机 ID、不同控制类型。成功组帧时，每台调用一次基类连接检查，并从清零的缓冲区复制各自槽位，避免旧数据污染。

空槽位填零。零不是通用的“不操作”：位置帧会令该 ID 的在线电机前往零位，必须列出总线上所有需要控制的电机。

## 反馈与连接状态

使用基类接收入口 `decodeCanRxMessageFromISR` 或 `decodeCanRxMessageFromQueue`。仅接受 ID 为 `0x180 + ID`、DLC 为 8 的标准数据帧。

状态 2 和对应控制回复更新温度、原始转矩电流、角速度及编码器位置；状态 1 和清错回复更新温度、电机状态和错误标志。通过 `getErrorState()` 和 `getMotorState()` 读取对应状态。

管理回复不代表新的编码器采样；周期控制需要持续接收状态 2 或控制指令的运动反馈。本类沿用基类掉线计数，仅报告连接状态，不会自动清零控制量或发送停止命令。应用层应决定停机策略。

接收与组帧访问同一对象时，由应用层提供同步。

## 固件差异与限制

现有 `MotorLKMG` 使用 `0x140 + ID` 作为反馈 ID；本类按附件 V2.36 使用 `0x180 + ID`。实机应确认固件版本与 CAN 过滤器配置。

广播协议未列出抱闸命令 `0x8C`，本类不会自动释放抱闸。需要抱闸控制时，由应用按实际固件单独处理。

厂商给出的四台通讯频率约为 500kbps 下 600Hz、1Mbps 下 1.2kHz；实际调度需要考虑其他总线负载。

主机端测试覆盖协议示例、组帧、限幅、反馈和连接检测；尚未完成 STM32 固件全量构建或实机 CAN 验证。
