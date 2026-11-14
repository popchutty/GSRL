# MotorLKMGmulti 使用说明（瓴控 MG 广播 CAN）

作者：王煜惟
版本：1.0.0
最后更新日期：2026-11-15

---

## 1：概述

`MotorLKMGmulti` 是 GSRL 中用于瓴控 MG 系列电机一控四广播模式的驱动类，负责生成广播控制槽位、合并报文及解析反馈。控制格式依据 V2.35 广播协议，反馈沿用 V2.36 格式；不代表兼容 V2.40 的全部字段和轴坐标定义。

类定义与实现位置：

- 头文件：[dvc_motor.hpp](../../Device/inc/dvc_motor.hpp)
- 源文件：[dvc_motor.cpp](../../Device/src/dvc_motor.cpp)

本类不初始化外设、不直接发送报文、不修改驱动器持久化配置，也不自动使能、清错或重启电机。

---

## 2：适用环境与依赖

- MCU：STM32，使用 STM32 HAL 的 CAN 帧结构。
- 时间基准：`HAL_GetTick()`，用于通信和运动反馈超时判断。
- 中断保护：CMSIS 的 PRIMASK 接口，退出临界区时恢复原状态。
- 控制器接口：`Controller`，仅继承的软件闭环方法需要有效控制器。

先通过厂商工具确认广播模式、波特率及固件支持。同一总线电机 ID 须为互不重复的1～4，广播波特率按适用协议设置为500kbps或1Mbps。编码器位数、减速比和回复 ID 必须按实际设备配置，不能把默认值当作已确认参数。

---

## 3：构造与参数配置

```cpp
using M = MotorLKMGmulti;
M motor(id, controller, encoderOffset, encoderBits, gearboxRatio,
        feedbackTimeoutMs, M::FeedbackIdBase::Id140);
```

| 参数 | 含义与默认值 |
| --- | --- |
| `id` | 电机 ID，1～4 |
| `controller` | 软件控制器；直接原始电流目标及硬件闭环可传 `nullptr` |
| `encoderOffset` | 编码器计数偏移，默认0 |
| `encoderBits` | CAN反馈计数位数，14～16，默认16；不等于物理编码器的标称位数 |
| `gearboxRatio` | 电机轴速度/输出轴速度，须为正的有限数，默认1 |
| `feedbackTimeoutMs` | 反馈有效期，须大于0，默认100ms |
| `feedbackIdBase` | `Id140` 或 `Id180`；默认 `Id180` 保持已有调用行为 |

`isConfigurationValid()` 检查配置。回复 ID 为所选基址加电机 ID：ID1选择 `Id140` 时接收 `0x141`，选择 `Id180` 时接收 `0x181`。只接收配置的一种 ID，不同时放行两个范围。

当前速度接口以输出轴弧度/秒输入，乘减速比编码为电机轴度/秒；反馈速度除以减速比。位置接口使用驱动器零点下的电机轴绝对角度，软件 `encoderOffset` 不改变驱动器的绝对零点。

---

## 4：控制接口与报文格式

### 4.1 运动接口

所有广播控制报文均为标准数据帧、DLC为8。每台占两个字节，偏移为 `(ID - 1) * 2`，低字节在前。

| 接口 | CAN ID | 输入与编码 |
| --- | --- | --- |
| `hardwareTorqueCurrentClosedloopControl(rawCurrent)` | `0x280` | 原始电流目标−2000～2000，不是安培或牛米 |
| `openloopControl(value)` | `0x280` | 控制器原始输出，限幅至−2000～2000；非有限值转为零 |
| `hardwareAngularVelocityClosedloopControl(radPerSecond)` | `0x281` | 输出轴弧度/秒，换算后每单位为电机轴1度/秒，有符号16位 |
| `hardwareAngleClosedloopControl(radians)` | `0x282` | 电机轴绝对角度，每单位0.01度，范围−327.68～327.67度 |
| `setManagementCommand(command)` | `0x288` | 槽位低字节为管理命令，高字节为零 |

直接电流、硬件速度、位置及管理接口返回 `bool`。失败保留原缓存，调用方必须跳过发送；编码成功也不代表合帧或发送成功。速度和位置按协议分辨率四舍五入，不能套用单电机 `0xA2` 的0.01度/秒比例。

### 4.2 管理命令

| 枚举 | 命令 | 用途 |
| --- | --- | --- |
| `ReadStatus1` | `0x9A` | 读取状态和故障字节 |
| `ClearError` | `0x9B` | 清除故障 |
| `ReadStatus2` | `0x9C` | 读取温度、电流、速度和编码器 |
| `Disable` | `0x80` | 关闭电机 |
| `Enable` | `0x88` | 开启电机 |
| `Stop` | `0x81` | 停止电机 |

不同电机可以在同一管理帧中使用不同管理命令，但不能混合力矩、速度、位置控制类型。广播类没有单电机 `setBrake()` 接口，不接受 `0x8C`，不自动释放抱闸。

---

## 5：快速接入示例

下面以同一总线安装 ID1、ID3，且实际回复基址为 `0x140` 为例。编码器16位、减速比6均为示例前提，部署时须替换为已确认配置。CAN初始化及接收回调注册由工程完成。

```cpp
#include "dvc_motor.hpp"
#include "can.h"

using M = MotorLKMGmulti;
M motor1(1, nullptr, 0, 16, 6.0f, 100, M::FeedbackIdBase::Id140);
M motor3(3, nullptr, 0, 16, 6.0f, 100, M::FeedbackIdBase::Id140);
M *motors[] = {&motor1, &motor3};
constexpr uint8_t installedMask = 0x05;

extern "C" void lkBroadcastRxCallback(can_rx_message_t *message)
{
    if (message == nullptr) return;
    motor1.decodeCanRxMessageFromISR(message);
    motor3.decodeCanRxMessageFromISR(message);
}

bool composeAndQueue()
{
    CAN_TxHeaderTypeDef header{};
    uint8_t data[8]{};
    M::ComposeResult reason;
    if (!M::composeControlFrame(motors, 2, header, data, installedMask, &reason)) {
        return false;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) return false;
    uint32_t mailbox = 0;
    return HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) == HAL_OK;
}

bool probeMotionWithZeroTorque()
{
    if (!motor1.hardwareTorqueCurrentClosedloopControl(0) ||
        !motor3.hardwareTorqueCurrentClosedloopControl(0)) return false;
    return composeAndQueue();
}

bool queueTorqueTargets()
{
    if (!motor1.hardwareTorqueCurrentClosedloopControl(100) ||
        !motor3.hardwareTorqueCurrentClosedloopControl(-100)) return false;
    return composeAndQueue();
}

bool queueStop()
{
    if (!motor1.setManagementCommand(M::Management::Stop) ||
        !motor3.setManagementCommand(M::Management::Stop)) return false;
    return composeAndQueue();
}
```

示例仅演示准备、合帧和邮箱受理，不自动执行旋转。应用须继续跟踪对应邮箱的 TXOK、错误及有限超时，并保留明确的停止入口。`composeAndQueue()` 返回 `true` 不等于电机已经执行命令。

`queueTorqueTargets()` 只有在两台都收到新鲜运动反馈后才能合帧，成功时为：

```text
ID 0x280，DLC 8：64 00 00 00 9C FF 00 00
```

每次发送前必须为同一总线的全部安装对象设置本次命令。安装掩码位0～3对应ID1～4；仅安装ID1时为 `0x01`，四台为 `0x0F`。遗漏、重复、多列对象或混用控制类型均会被拒绝。传掩码0或使用旧重载会跳过安装集合核验。

未列出的槽位填零，软件不能发现总线上的遗漏电机；零位置会成为实际目标，不代表保持当前位置。合帧失败时输出保持原值，禁止误发上一次成功的报文。

---

## 6：反馈、启动与停止

### 6.1 接收与有效期

库仅接受匹配配置 ID、标准数据帧、DLC为8且命令受支持的回复。每台选择 ISR 或队列接收之一，避免将同一帧重复交付。当前基类队列接口从后向前选择有效帧；使用累计圈数时仍须关注采样连续性，不能把它当作逐帧轨迹回放。

| 接口 | 含义 |
| --- | --- |
| `getFeedbackState()` | `NeverReceived`、`Online` 或 `Offline`，按合法回复时间判断 |
| `isMotorConected()` | 广播类的通信在线状态；通过非虚基类 `Motor` 查询仍是基类行为 |
| `hasFreshMotionFeedback()` | 最近一次 `0x9C/A1/A2/A3` 运动回复仍在有效期内 |
| `hasValidMotorStatus()` | 曾收到有效电机状态，不能代替当前在线判断 |
| `hasValidErrorStatus()` | 曾收到有效故障状态，初始错误值0不代表已确认无故障 |

默认100ms有效期从解码时刻开始。状态回复可以更新通信在线状态，但不能刷新运动反馈有效期；读取缓存或合帧也不会刷新有效期。队列应及时消费，避免旧帧被当作实时数据。

### 6.2 合帧门控

| 命令 | 无反馈或过期时能否合帧 |
| --- | --- |
| 状态查询、`Stop`、`Disable`、零力矩 | 可以，用于启动探测和恢复 |
| `Enable`、`ClearError` | 必须通信在线 |
| 非零力矩、速度（包括零速度）、位置 | 必须具有新鲜运动反馈 |

零力矩恢复按对象槽位检查，不会放宽同一帧中其他非零槽位的门控。零力矩不保证制动或失能，不能替代 Stop/Disable。

### 6.3 应用启动与停止

先发送查询或零力矩探测，交付真实回复并建立在线状态，再合成使能命令。发送结果、使能执行结果和运动反馈应分别检查；没有回复时应超时退出，不得伪造反馈或删除非零运动门控。

部分设备未观察到广播管理查询或回显，但会返回力矩控制的 `0xA1` 反馈。因此不能把“发送了使能”或“收到 A1”直接记为已确认使能、无故障或已经停止，具体启动策略仍由应用按实际固件设计。

运行中反馈过期、合帧失败或总线异常时，应停止正常运动下发，利用零力矩、Stop/Disable恢复路径有限次尝试停止，检查发送结果和后续反馈，不自动反复清错或重发旧目标。

---

## 7：PR 复现范围与限制

[PR #77](https://github.com/popchutty/GSRL/pull/77) 的相关调用在驱动中以中文注释标出：

- `convertControllerOutputToMotorControlData()`：原始力矩编码。
- `composeControlFrame()`：完整安装集合、控制类型和反馈门控检查。
- `decodeCanRxMessage()`：实际回复 ID、帧格式及运动数据解析。

新增的反馈基址选择、直接电流接口和零力矩恢复用于处理实机发现的兼容性问题。`Id180` 仍为默认值；实际回复 `0x141` 的ID1设备须显式选择 `Id140`。

当前分支的广播修订已进行主机模拟及 Cortex-M4 模块编译；其他工作副本上的电机实测不能直接写成本分支已完整上板通过。广播速度、位置、管理命令执行回显及全部故障路径仍需按实际固件验证，也不能把力矩模式的旋转结果写成恒速控制成功。
