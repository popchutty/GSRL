# 裁判系统驱动开发文档 (Referee System Driver Docs)

## 1. 概述
本驱动用于解析 RoboMaster 裁判系统串口协议（V1.6.1），支持数据接收、校验、解包及数据结构化存储。

*   **适用硬件**: STM32F407 (或其他支持 UART 的 MCU)
*   **通信接口**: USART6 (RX: PG9, TX: PG14)
*   **波特率**: 115200, 8N1
*   **接收方式**: DMA + 空闲中断 (Idle Line Detection)

## 2. 协议说明
遵循 RoboMaster 串口协议 V1.6.1 标准。

### 2.1 数据帧结构
| SOF (1B) | DataLength (2B) | Seq (1B) | CRC8 (1B) | CmdID (2B) | Data (nB) | CRC16 (2B) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0xA5 | 数据段长度 | 包序号 | 帧头校验 | 命令ID | 数据内容 | 整帧校验 |

### 2.2 关键配置
*   **CRC8 初始值**: `0xFF`
*   **CRC16 初始值**: `0x0000` (注意：非 0xFFFF，2024年这版大疆给的经测试发现不对)
*   **多字节序**: Little Endian (小端模式)

## 3. 接口使用

### 3.1 初始化
在任务初始化阶段调用：
```cpp
// 缓冲区建议 >= 128 字节，防止长包截断
UART_Init(&huart6, refereeITCallback, 128);
```

### 3.2 中断回调
在串口中断回调函数中将数据喂给解析器：
```cpp
// 定义全局解析器实例
RefereeParser referee(RefereeLink::Main);

extern "C" void refereeITCallback(uint8_t *Buffer, uint16_t Length)
{
    for (uint16_t i = 0; i < Length; i++)
    {
        referee.onByteReceived(Buffer[i]);
    }
}
```

### 3.3 数据获取
解析后的数据存储在 `RefereeInfo` 结构体中，可直接访问：

```cpp
const RefereeInfo& info = referee.getInfo();

// 1. 检查数据是否更新
if (info.hasRobotStatus) {
    // 2. 读取具体字段
    uint8_t my_id = info.robotStatus.robot_id;
    uint16_t hp = info.robotStatus.current_HP;
    float power = info.powerHeat.chassis_power;
}

// 3. 比赛状态（注意：单机调试时可能没有此包）
if (info.hasGameStatus) {
    int stage = info.gameStatus.game_progress;
}
```

## 4. 支持的数据包列表

| Cmd ID | 描述 | 对应结构体 | 备注 |
| :--- | :--- | :--- | :--- |
| **0x0001** | 比赛状态 | `GameStatus` | 包含倒计时、比赛阶段 |
| **0x0002** | 比赛结果 | `GameResult` | 胜负判定 |
| **0x0003** | 机器人血量 | `GameRobotHP` | 红蓝双方所有单位血量 |
| **0x0201** | 机器人状态 | `RobotStatus` | ID、血量、功率限制、热量限制 |
| **0x0202** | 功率热量 | `PowerHeatData` | 实时功率、枪口热量 |
| **0x0203** | 机器人位置 | `RobotPos` | x, y, angle |
| **0x0204** | 增益状态 | `Buff` | 补给站Buff、防御Buff |
| **0x0206** | 伤害数据 | `HurtData` | 装甲板ID、伤害类型 |
| **0x0207** | 射击数据 | `ShootData` | 射速、弹丸初速 |
| **0x0208** | 弹丸余量 | `ProjectileAllowance` | 剩余发弹量 |
| **0x0303** | 小地图命令 | `MapCommand` | 选手端点击坐标 |
| **0x0304** | 键鼠数据 | `RemoteControl` | **仅图传链路有效** |

## 5. 常见问题排查

1.  **完全无数据**: 检查 `board_config.h` 是否定义 `USE_USART6`，检查 RX/TX 线序。
2.  **校验错误 (CRC Fail)**: 确认 `dvc_referee.cpp` 中 `CRC16_INIT` 为 `0x0000`。
3.  **长度错误 (Len Mismatch)**: 确认 `dvc_referee.hpp` 中的结构体定义是否与 V1.6.1 协议一致。
