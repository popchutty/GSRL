# 瓴控广播模式(一控四)电机驱动使用说明

作者：yuki  
版本：v1.0.0  
最后更新日期：2026-11-12
对应协议：《瓴控科技电机 CAN 广播通讯协议 V2.35》《电机 CAN 总线通讯协议 V2.36》

---

## 1：概述

`MotorLKBroadcast` 用于控制开启了"广播模式"的瓴控 MF/MG/MS 系列电机。广播模式下一帧 CAN 报文同时携带 4 个电机的控制量，
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

### 固件适用范围与已验证能力

下文列出广播协议 V2.35 的编码能力，**不表示每款电机固件均支持所有命令**。
[厂家 MG60–100 V2.38 使用手册](http://www.lkmotor.cn/upload/20260730145252f.pdf)限定广播为扭矩控制；
[独立广播协议 V2.35](http://www.lkmotor.cn/upload/20260317100100f.pdf)另列速度、位置与混合命令。
应按具体产品、驱动固件和实际回复选择功能，不能仅比较版本号。

2026-11-12，单台 MG8016E-i6 V3、驱动固件 V2.36、ID1、CAN1 1Mbps 的实测：

| 项目 | 结果与边界 |
| --- | --- |
| 广播设置 ON，断电重启后读回 | 保持 ON |
| `0x280` / DLC8 / 八字节全零扭矩 | 收到 `0x141` 回复 `A1 1D 0A 00 00 00 07 85`，真实反馈速度为 0 |
| 固定 2 秒全零广播窗口（禁用主控自动重传） | 1000 次 TXOK、1000 帧 `0x141/A1` 回复，发送失败 0；窗口结束后邮箱全空且不再发送 |
| 广播 ON 时 `0x288` 状态查询及 `0x141` 单机查询 | 本次对照无回复；恢复 OFF 并重启后，单机查询回复恢复 |
| 单台有限非零扭矩探测 | UART读回故障0、速度0后，5帧`+25`原始目标获得TXOK；速度阈值触发提前回零，300ms零尾段后停发。现场未观察到明显动作，不据此声称机械运动幅度或额定扭矩已验证 |
| 广播速度/位置、四台同时响应、MS型号 | 尚未实物验收；主机编码测试不能代替这些结果 |

全零扭矩是控制指令，不是制动或电机关闭命令。不能依赖本机未验证的 `0x288` 停止指令实现失联保护；
运动前需独立验证电机 Lost Input 保护、主控命令期限及故障锁存。单次零帧回复也不提供持续的电压/故障监测。

本次还将 Lost Input 设为 100 ms / Enable(not recoverable)，掉电后读回保持；无输入时实际报告错误位7（`0x80`）。
随后恢复上述全零广播，在通信窗口内通过 UART 读取仍为 `0x80`、速度为0。
这支持失联故障在该零量工况中不随恢复通信自动清除，**尚不证明非零运动时的功率级抑制、精确保护时间或实际停止距离**。

有限非零探测使用独立台架程序，固定目标、一次触发、反馈阈值和有限零尾段；这些保护不是本库自动提供的功能。
该次完整通信会话记录18527次TXOK及18527条回复，包含准备阶段和零尾段，不能将全部回复归因于5帧非零目标。
结束后UART速度为0，失联故障再次锁存。此结果证明有限命令发送及提前退出路径，不构成闭环性能或机械停机验收。

## 3：协议要点

控制帧为标准数据帧，DLC = 8，每个电机占 2 字节，**低字节在前**：

| 电机 ID | 1 | 2 | 3 | 4 |
| --- | --- | --- | --- | --- |
| 数据槽位 | D[0:1] | D[2:3] | D[4:5] | D[6:7] |

| 帧 ID | 命令 | 槽位内容 | 本类对应方法 |
| --- | --- | --- | --- |
| 0x280 | 力矩 / 开环电压 | int16，MF/MG为-2000~2000，MS为-850~850 | `openloopControl` 及基类所有软件闭环方法 |
| 0x281 | 速度控制 | int16，1dps/LSB，-32768~32767 | `hardwareAngularVelocityClosedloopControl` |
| 0x282 | 位置控制 | int16，0.01°/LSB，绝对位置，-32768~32767 | `hardwareAngleClosedloopControl` |
| 0x288 | 混合命令 | 低字节为命令字节，高字节为 0x00 | `setMotorCommand` |

混合命令支持：`READ_STATE_1`(0x9A)、`CLEAR_ERROR`(0x9B)、`READ_STATE_2`(0x9C)、`MOTOR_OFF`(0x80)、`MOTOR_ON`(0x88)、`MOTOR_STOP`(0x81)。

电机收到广播帧后按 ID 从低到高依次回复，回复的数据格式与单电机模式对应命令的回复相同：

| 回复命令字节 D[0] | 内容 | 更新的状态 |
| --- | --- | --- |
| 0x9C、0xA0~0xA8 | 温度、转矩电流(MF/MG)或功率(MS)、转速(dps)、编码器 | 角度、角速度、输出反馈原始值、温度 |
| 0x9A、0x9B | 温度、母线电压、母线电流、电机状态、错误标志 | 温度、`getBusVoltage`、`getBusCurrent`、`isMotorOn`、`getErrorState` |
| 0x80、0x88 | 与发送相同 | `isMotorOn` |
| 0x81 | 与发送相同 | 无，只用于确认电机在线 |

**反馈帧 ID 必须核对匹配固件的文档、配置及实际回复**，不能仅凭“V2.36”字样判定为 0x180 + ID。
构造函数参数 `feedbackBaseID` 默认 `FEEDBACK_BASE_0x140`。当前 MG8016E-i6 V3 / 固件 V2.36 的单机查询及全零扭矩广播实测均回复 0x141（ID1）；其他配置仍需复核。
只有确认实际回复为 0x180 + ID 时，才传入 `FEEDBACK_BASE_0x180`。
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
  此时返回 `nullptr`，必须拒绝发送；这改变了旧版返回部分帧的行为，调用代码必须检查返回值。
  `isLastMergeComplete()` 只表示传入的电机是否全部写入，不表示 4 个槽位齐全，也不表示发送成功。
- **控制、合帧、发送要在同一个任务里依次完成。** 合帧得到的数据和 `getMotorControlHeader()` 的帧 ID 是分两次取的，
  中间如果有别的任务切换了电机的控制模式，会把旧数据配上新的帧 ID 发出去。
- **电机 ID 只能是 1~4。** 传入其他值时对象失效：不写入控制数据、不解析反馈、不参与合帧，`isMotorIDValid()` 返回 false。
- **控制量自动限幅。** MF/MG力矩限幅到 ±2000，MS开环电压限幅到 ±850，速度、位置限幅到 -32768~32767，NaN 按 0 处理。
  基类各闭环方法的返回值是限幅取整后实际发送的控制量。浮点rad换算在半LSB附近可能有量化误差。
- **硬件位置闭环的范围很小。** 16 位、0.01°/LSB，电机轴角度范围只有 -327.68°~327.67°，带减速箱时输出轴范围还要除以减速比。
  需要更大范围时请使用基类的软件闭环（力矩帧）。
- **硬件位置闭环和软件角度闭环的目标互不相通。** `hardwareAngleClosedloopControl(targetAngle)` 的目标以电机自身零点为基准，
  并按减速比换算到电机轴；基类 `angleClosedloopControl` 的目标是以 `encoderOffset` 为零点的电机轴单圈角度 [0, 2π)。
  两者分开保存，切换控制方式时需要重新给出目标。
- **减速比。** `gearboxRatio` 用于换算角速度反馈和硬件闭环的目标值，角度反馈为电机轴编码器角度，不受减速比影响，与 `MotorLKMG` 一致。
  传入 0、负数、NaN 或无穷大时按 1 处理。
- **在线与数据有效性。** LK 使用 HAL 毫秒时基，100 ms 没有可识别反馈即离线，读取控制缓冲不会改变在线状态。
  `isState1Fresh()` 默认期限1000 ms，`isState2Fresh()` 默认100 ms；可传入与实际查询周期匹配的期限。
  `isMotorStateKnown()` 为 false 时，`isMotorOn()` 的 false 表示未知或过期，不能据此断言电机已停机。
  电压/故障等数值 getter 保留上次值，使用前必须检查对应有效性。只有状态1回复不能证明速度/位置仍有效。
- **圈数语义。** 首次角度反馈保留编码器偏移后的单圈绝对初值，不把首次观测当成跨零。
  状态1/开关应答不参与计圈。状态2间隔达到100 ms后，多圈历史失效；重连不自动恢复可信度。
  `isRevolutionsValid()` 用于检查；在新位置反馈有效且已确认坐标后，可显式 `resetCurrentRevolutionsToZero()` 重新锚定。
  正常计圈仍要求采样间位移小于半圈，不能恢复丢失的整圈。
- **共享对象需同步。** 类本身不加锁。推荐同一任务依序解析、控制、合帧。驱动在ISR取帧时填写 `receivedAtMs` / `receivedAtValid`，队列复制保留该时间；积压帧不会以任务解析时间续期，乱序/未来时间帧被拒绝。LK队列接口拒绝缺少有效时间的帧；直接ISR接口对未标时帧使用当前HAL时间。消息须在2^31 ms内消费，HAL时基须持续运行。
- **接收结构升级需完整重编译。** `can_rx_message_t` 新增时间和 `sourceBus`（1=CAN1、2=CAN2）；自定义接收者须初始化新增字段。共享队列消费方须先按 `sourceBus` 分流，再传给所属总线电机。驱动ISR时间不是CAN硬件线上到达时间。
- **初始化与发送失败必须处理。** `CAN_Init` 返回过滤器/启动/通知失败状态，通知失败时尝试停止CAN；启动阶段顺序调用，不支持并发初始化。`CAN_Send_Data` 使用有效的HAL邮箱输出参数并原样返回HAL状态。失败时不得继续运动发送，队列申请失败可在排除原因后重试。
  若接收ISR直接解析，任务读取反馈和执行闭环计算时应使用能屏蔽该RX中断的短临界区。不能只给Watch变量加volatile。
- **发送不等于安全联锁。** 驱动只编码命令，离线不会自动发送停机，也不会自动清除目标。
  应用必须处理目标过期、反馈过期、故障锁存、重连重新解锁和过期邮箱；电机独立失联保护须另行配置、实测。
  硬件闭环的控制量限幅不等于内部电流/力矩限制。
- **MS型号需显式选择。** 构造函数最后一个参数 `motorFamily` 默认 `MF_MG`，保持原调用含义。MS传入 `MS`，`openloopControl()` 及软件闭环输出均编码为开环电压原始控制量，不能当作力矩电流。非法枚举值拒绝控制编码、反馈和合帧，可用 `isMotorFamilyValid()` 检查。
  状态2使用 `getOutputFeedbackRaw()`：MF/MG返回iq原始值，MS返回power原始值(-1000~1000)。继承的 `getCurrentTorqueCurrent()` 为兼容保留同一字段；对MS不得把该名称理解为电流，更不能直接当安培。读取前检查 `isState2Fresh()`。本次MS只有协议回归证据，没有MS实物验证。

```cpp
MotorLKBroadcast ms2(2, nullptr, 0, 1.0f,
    MotorLKBroadcast::FEEDBACK_BASE_0x140,
    MotorLKBroadcast::ENCODER_16BIT, MotorLKBroadcast::MS);
ms2.openloopControl(100.0f); // 仅准备ID2槽位的开环电压编码; 本段不发送
```

## 5：典型用法

### 5.1 四个电机，主控做闭环，准备一帧力矩数据

下面只演示反馈分发、闭环计算和合帧，不包含自动发送任务。调用方完成解锁、目标期限、反馈与故障检查后才可调用；PID参数仅用于展示接口，需按实物整定。

```cpp
#include "dvc_motor.hpp"

SimplePID::PIDParam speedParam = {20.0f, 0.0f, 0.0f, 500.0f, 0.0f};
SimplePID pid1(SimplePID::PID_POSITION, speedParam);
SimplePID pid2(SimplePID::PID_POSITION, speedParam);
SimplePID pid3(SimplePID::PID_POSITION, speedParam);
SimplePID pid4(SimplePID::PID_POSITION, speedParam);

// 参数: 电机ID, 控制器, 编码器偏移量, 减速比, 反馈基地址, 编码器位数
// 示例假定已核对回复为0x140+ID及16bit编码器, 不能仅凭版本号照抄。
MotorLKBroadcast lk1(1, &pid1, 0, 1.0f, MotorLKBroadcast::FEEDBACK_BASE_0x140, MotorLKBroadcast::ENCODER_16BIT);
MotorLKBroadcast lk2(2, &pid2, 0, 1.0f, MotorLKBroadcast::FEEDBACK_BASE_0x140, MotorLKBroadcast::ENCODER_16BIT);
MotorLKBroadcast lk3(3, &pid3, 0, 1.0f, MotorLKBroadcast::FEEDBACK_BASE_0x140, MotorLKBroadcast::ENCODER_16BIT);
MotorLKBroadcast lk4(4, &pid4, 0, 1.0f, MotorLKBroadcast::FEEDBACK_BASE_0x140, MotorLKBroadcast::ENCODER_16BIT);

extern "C" void can1RxCallback(can_rx_message_t *pRxMsg)
{
    lk1.decodeCanRxMessageFromISR(pRxMsg);
    lk2.decodeCanRxMessageFromISR(pRxMsg);
    lk3.decodeCanRxMessageFromISR(pRxMsg);
    lk4.decodeCanRxMessageFromISR(pRxMsg);
}

bool prepareFourMotorFrame(const float (&targetSpeed)[4],
                          CAN_TxHeaderTypeDef &header, uint8_t (&data)[8])
{
    taskENTER_CRITICAL(); // 本例要求CAN RX中断能被FreeRTOS临界区屏蔽
    lk1.angularVelocityClosedloopControl(targetSpeed[0]);
    lk2.angularVelocityClosedloopControl(targetSpeed[1]);
    lk3.angularVelocityClosedloopControl(targetSpeed[2]);
    lk4.angularVelocityClosedloopControl(targetSpeed[3]);
    MotorLKBroadcast *motors[] = {&lk1, &lk2, &lk3, &lk4};
    const bool complete = MotorLKBroadcast::buildControlFrame(motors, 4, 0x0F, header, data);
    taskEXIT_CRITICAL();
    return complete; // false必须拒绝发送; true只表示编码完整
}
```

应用初始化时须检查 `CAN_Init(&hcan1, can1RxCallback) == HAL_OK`，失败应锁存并停止后续发送。
发送端须处理 `HAL_CAN_AddTxMessage` 的返回值并观察邮箱完成状态；`HAL_OK` 只表示入邮箱，不代表总线发送成功或电机执行。

电机数量少于 4 个时使用对应的重载：`lk1.getMergedControlData(lk2)`、`lk1.getMergedControlData(lk2, lk3)`。
推荐统一使用 `buildControlFrame`：1台ID1时 count=1、activeMask=0x01；2台ID1/ID3时 count=2、activeMask=0x05。
掩码必须覆盖总线上所有会执行广播控制的电机；软件无法自动知道未声明的实物节点。

### 5.2 使用电机内置的速度环、位置环

```cpp
lk1.hardwareAngularVelocityClosedloopControl(3.14f); // rad/s
lk2.hardwareAngularVelocityClosedloopControl(-3.14f);
const uint8_t *data = lk1.getMergedControlData(lk2);  // 帧ID为0x281
// data为nullptr时拒绝发送; 非空时由应用发送层处理邮箱结果和目标期限。
```

位置环同理，使用 `hardwareAngleClosedloopControl(targetAngle)`，单位 rad，带符号的绝对位置。
位置帧推荐使用 `buildControlFrame` 校验参与掩码；`isLastMergeComplete()` 只能检查传入对象，不能发现漏传的实物节点。
不同固件的硬件目标轴侧定义可能不同：本类当前保留乘减速比的电机轴编码约定，只有匹配该约定时可用。
不能直接套用新V2.40输出轴目标说明到旧V2.36实物；目标坐标未核实前禁止使用硬件速度/位置模式。

### 5.3 读取错误标志、清除错误、开关电机

```cpp
// 低频查询状态1: 该周期用混合命令帧代替控制帧
lk1.setMotorCommand(MotorLKBroadcast::READ_STATE_1);
lk2.setMotorCommand(MotorLKBroadcast::READ_STATE_1);
const uint8_t *data = lk1.getMergedControlData(lk2);  // 帧ID为0x288
// data为nullptr时拒绝发送; 非空时交给应用发送层。

// 在与反馈更新同步的上下文内检查; stale=true不能被当作“无故障”。
const bool stale = !lk1.isState1Fresh();
const bool stalled = !stale && (lk1.getErrorState() & MotorLKBroadcast::ERROR_STALL);
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
