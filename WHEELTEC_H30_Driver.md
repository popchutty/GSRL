# WHEELTEC_H30 驱动模块开发文档

## 1. 模块概述 (Overview)

**WHEELTEC_H30** 类是针对 H30 高性能惯性导航模块开发的驱动程序。该模块基于 `IMU` 基类派生，通过 UART 串口与 STM32 通信，支持解析模块内部解算好的高精度姿态数据（欧拉角、四元数）以及原始传感器数据。

*   **所属文件**:
    *   头文件: `GSRL/Device/inc/dvc_imu.hpp`
    *   源文件: `GSRL/Device/src/dvc_imu.cpp`
*   **父类**: `IMU`
*   **通信接口**: UART (异步串口)
*   **协议特征**: YIS 通信协议 (帧头 `0x59 0x53`)

## 2. 核心功能 (Key Features)

该驱动不仅仅是读取原始数据，还充分利用了 H30 模块内部强大的解算能力，实现了以下核心特性：

1.  **硬件姿态直出**: 重写了 `solveAttitude`、`getEulerAngle` 和 `getQuaternion` 虚函数，直接返回模块内部解算结果，**跳过了板载软件 AHRS 算法**，极大节省了 MCU 算力。
2.  **全数据解析**: 支持解析加速度、角速度、磁力计、欧拉角、四元数、温度及时间戳。
3.  **协议解析**: 实现了完整的帧头检测、长度校验和双字节校验和 (CK1/CK2) 机制。
4.  **错误处理**: 包含校验错误、帧头错误、长度错误等多种错误状态反馈。

## 3. 接口说明 (API Reference)

### 3.1 构造函数
```cpp
H30(AHRS *ahrs, UART_HandleTypeDef *huart, ErrorCallback errorCallback = nullptr);
```
*   **ahrs**: 传入 AHRS 接口指针（虽然 H30 自带解算，但为了兼容基类仍需传入）。
*   **huart**: 指定使用的串口句柄（如 `&huart1`）。
*   **errorCallback**: 可选的错误回调函数。

### 3.2 初始化
```cpp
bool init() override;
```
*   发送配置指令 `0x59 0x53 0x03 0x0A 0x00 0x04 0x11 0x2E` 给模块，使其进入工作状态。

### 3.3 数据接收 (关键)
```cpp
void onReceiveData(uint8_t *data, uint16_t length);
```
*   **用途**: 该函数需要被放置在串口接收中断回调（如 `HAL_UART_RxCpltCallback` 或 DMA 空闲中断）中调用。
*   **逻辑**: 将接收到的原始字节流存入缓冲区，供后续解析使用。

### 3.4 姿态获取
由于重写了基类方法，调用以下接口将直接获取 H30 的高精度数据：
*   `solveAttitude()`: 更新并返回欧拉角。
*   `getEulerAngle()`: 获取当前欧拉角 (Roll, Pitch, Yaw)。
*   `getQuaternion()`: 获取当前四元数 (q0, q1, q2, q3)。

## 4. 协议解析逻辑 (Implementation Details)

驱动内部维护了一个状态机式的解析流程 (`readRawData`)：

1.  **帧头校验**: 检查前两个字节是否为 `0x59 0x53`。
2.  **长度检查**: 读取 Payload 长度，确保接收到的数据量足够。
3.  **校验和验证**:
    *   算法：累加校验 (Checksum)。
    *   公式：`CK1 = sum(data)`, `CK2 = sum(CK1)`。
    *   范围：覆盖 TID、Length 和 Payload。
4.  **数据包解包 (Payload Parsing)**:
    *   遍历 Payload 中的数据项 (Data ID)。
    *   **0x10 (ACCEL)**: 解析加速度，单位转换为 $m/s^2$。
    *   **0x20 (GYRO)**: 解析角速度，单位转换为 $rad/s$。
    *   **0x40 (EULER)**: 解析欧拉角，单位 $deg$。
    *   **0x41 (QUAT)**: 解析四元数。
    *   **0x51 (TIMESTAMP)**: 解析时间戳。

## 5. 使用示例 (Usage Example)

```cpp
// 1. 定义对象 (在 main.c 或任务中)
H30 imu_h30(&ahrs_algorithm, &huart1);

// 2. 初始化
imu_h30.init();

// 3. 在串口中断中调用接收
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 假设使用了 DMA 接收
        imu_h30.onReceiveData(rx_buffer, rx_len);
    }
}

// 4. 在任务中读取姿态
void Task_IMU(void *argument) {
    while(1) {
        // 获取欧拉角 (直接来自 H30 内部解算)
        const IMU::Vector3f& angle = imu_h30.solveAttitude();
        
        printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", 
               angle[0], angle[1], angle[2]);
               
        osDelay(5);
    }
}
```
