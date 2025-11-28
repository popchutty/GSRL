/**
 ******************************************************************************
 * @file           : dvc_imu.hpp
 * @brief          : header file for dvc_imu.cpp
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"
#include "drv_spi.h"
#include "alg_ahrs.hpp"
#include "Define/def_bmi088.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief IMU基类
 * @note 该类不可实例化
 * @details 包含IMU通用的数据读取、校准、姿态解算方法
 * @details 该类的派生类需实现以下方法：
 * - init 初始化
 * - readRawData 读取原始数据
 * - dataCalibration 数据校准
 */
class IMU
{
protected:
    using Vector3f  = GSRLMath::Vector3f;
    using Matrix33f = GSRLMath::Matrix33f;
    AHRS *m_ahrs;             // 姿态解算算法接口
    Vector3f m_gyroRawData;   // 陀螺仪原始数据
    Vector3f m_accelRawData;  // 加速度计原始数据
    Vector3f m_magnetRawData; // 磁力计原始数据
    Vector3f m_gyroData;      // 陀螺仪数据(校准后)
    Vector3f m_accelData;     // 加速度计数据(校准后)
    Vector3f m_magnetData;    // 磁力计数据(校准后)

public:
    virtual ~IMU() = default;
    virtual bool init() = 0;
    const Vector3f &solveAttitude();
    const Vector3f &getEulerAngle() const;
    const fp32 *getQuaternion() const;

protected:
    IMU(AHRS *ahrs);
    virtual void readRawData() = 0;
    virtual void dataCalibration() = 0;
};

class IST8310
{
};

/**
 * @brief BMI088类
 * @details 该类继承自IMU类, 实现了BMI088的初始化、数据读取、校准等方法, 基于大疆官方例程修改
 */
class BMI088 : public IMU
{
public:
    enum ErrorCode : uint8_t {
        BMI088_NO_ERROR                     = 0x00,
        BMI088_ACC_PWR_CTRL_ERROR           = 0x01,
        BMI088_ACC_PWR_CONF_ERROR           = 0x02,
        BMI088_ACC_CONF_ERROR               = 0x03,
        BMI088_ACC_SELF_TEST_ERROR          = 0x04,
        BMI088_ACC_RANGE_ERROR              = 0x05,
        BMI088_INT1_IO_CTRL_ERROR           = 0x06,
        BMI088_INT_MAP_DATA_ERROR           = 0x07,
        BMI088_GYRO_RANGE_ERROR             = 0x08,
        BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
        BMI088_GYRO_LPM1_ERROR              = 0x0A,
        BMI088_GYRO_CTRL_ERROR              = 0x0B,
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
        BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,
        BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,
        BMI088_SELF_TEST_GYRO_ERROR         = 0x40,
        BMI088_NO_SENSOR                    = 0xFF,
    };
    typedef void (*ErrorCallback)(ErrorCode errorCode); // 错误回调函数

    /**
     * @brief SPI配置结构体
     * @note 用于BMI088的SPI通信
     * - hspi SPI句柄
     * - csGPIOGroup CS引脚GPIO组
     * - csGPIOPin CS引脚编号
     */
    struct SPIConfig {
        SPI_HandleTypeDef *hspi;   // SPI句柄
        GPIO_TypeDef *csGPIOGroup; // CS引脚GPIO组
        uint16_t csGPIOPin;        // CS引脚编号
    };

    /**
     * @brief 陀螺仪校准信息结构体
     * @note 用于存储BMI088的校准信息
     * - gyroOffset 陀螺仪零飘补偿
     * - accelOffset 加速度计零飘补偿
     * - magnetOffset 磁力计零飘补偿
     * - installSpinMatrix 安装误差修正旋转矩阵
     */
    struct CalibrationInfo {
        Vector3f gyroOffset;         // 陀螺仪零飘补偿
        Vector3f accelOffset;        // 加速度计零飘补偿
        Vector3f magnetOffset;       // 磁力计零飘补偿
        Matrix33f installSpinMatrix; // 安装误差修正旋转矩阵
    };

protected:
    SPIConfig m_accelSPIConfig;              // 加速度计SPI配置
    SPIConfig m_gyroSPIConfig;               // 陀螺仪SPI配置
    const CalibrationInfo m_calibrationInfo; // 校准信息
    ErrorCallback m_errorCallback;           // 错误回调函数
    ErrorCode m_errorCode;                   // 错误码
    IST8310 *m_magnet;                       // 磁力计扩展类
    fp32 m_temperature;                      // 温度值
    // 常量定义
    static constexpr fp32 ACCEL_SEN = BMI088_ACCEL_3G_SEN;
    static constexpr fp32 GYRO_SEN  = BMI088_GYRO_2000_SEN;

public:
    BMI088(AHRS *ahrs, SPIConfig accelSPIConfig, SPIConfig gyroSPIConfig, CalibrationInfo calibrationInfo, ErrorCallback errorCallback = nullptr, IST8310 *magnet = nullptr);
    bool init() override;

protected:
    void readRawData() override;
    void dataCalibration() override;

private:
    inline void readSingleReg(const SPIConfig &spiConfig, uint8_t reg, uint8_t &rxData);
    inline void readMultiReg(const SPIConfig &spiConfig, uint8_t reg, uint8_t *pRxData, uint8_t length);
    inline void writeSingleReg(const SPIConfig &spiConfig, uint8_t reg, const uint8_t &txData);
    inline void handleError(ErrorCode errorCode);
    bool initAccel();
    bool selfTestAccel();
    bool initGyro();
    bool selfTestGyro();
};


/**
 * @brief H30系列 UART接口IMU类
 * @details 适用于H30等使用YIS通讯协议的模块
 */
class H30 : public IMU
{
public:
    // 错误码定义
    enum ErrorCode : uint8_t {
        NO_ERROR           = 0x00,
        CHECKSUM_ERROR     = 0x01, // CK1/CK2校验失败
        FRAME_HEAD_ERROR   = 0x02, // 帧头不是 0x59 0x53
        FRAME_LENGTH_ERROR = 0x03, // 长度不符合预期
        TIMEOUT_ERROR      = 0x04, // 超时未收到数据
    };
    
    typedef void (*ErrorCallback)(ErrorCode errorCode); 

    /**
     * @brief 构造函数
     * @param ahrs 姿态解算算法接口
     * @param huart 串口句柄
     * @param errorCallback 错误回调函数
     */
    H30(AHRS *ahrs, UART_HandleTypeDef *huart, ErrorCallback errorCallback = nullptr);

    bool init() override;

    /**
     * @brief 数据接收回调处理
     * @note 需要在 drv_uart 的回调函数中调用此方法
     */
    void onReceiveData(uint8_t *data, uint16_t length);

protected:
    void readRawData() override;
    void dataCalibration() override;

private:
    UART_HandleTypeDef *m_huart;
    ErrorCallback m_errorCallback;
    ErrorCode m_errorCode;

    // 协议常量
    static constexpr uint8_t FRAME_HEAD_0 = 0x59;
    static constexpr uint8_t FRAME_HEAD_1 = 0x53;
    static constexpr uint8_t DATA_ID_TEMP = 0x01;
    static constexpr uint8_t DATA_ID_ACCEL = 0x10;
    static constexpr uint8_t DATA_ID_GYRO = 0x20;
    static constexpr uint8_t DATA_ID_MAG_NORM = 0x30;
    static constexpr uint8_t DATA_ID_MAG_RAW = 0x31;
    static constexpr uint8_t DATA_ID_EULER = 0x40;
    static constexpr uint8_t DATA_ID_QUAT = 0x41;
    static constexpr uint8_t DATA_ID_TIMESTAMP = 0x51;

    // 接收缓冲区 
    uint8_t m_rxBuffer[128]; 
    uint16_t m_rxLength;

    // 额外数据存储
    fp32 m_temperature;
    Vector3f m_eulerRawData; // 模块直接输出的欧拉角
    fp32 m_quatRawData[4];   // 模块直接输出的四元数
    uint32_t m_timestamp;    // 时间戳

    // 内部辅助函数
    void handleError(ErrorCode errorCode);
    bool validateChecksum(uint8_t *data, uint16_t len);
};
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
