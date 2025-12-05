/**
 ******************************************************************************
 * @file           : dvc_imu.cpp
 * @brief          : IMU姿态传感器驱动
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_imu.hpp"
#include "drv_misc.h"
#include "cmsis_os.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                            IMU类实现
 ******************************************************************************/

/**
 * @brief 姿态解算
 * @note 使用前先调用init()方法, 放入freeRTOS任务中运行, 尽量保证定周期运行
 * @retval 欧拉角姿态
 */
const IMU::Vector3f &IMU::solveAttitude()
{
    readRawData();
    dataCalibration();
    m_ahrs->update(m_gyroData, m_accelData, m_magnetData);
    return m_ahrs->getEulerAngle();
}

/**
 * @brief 获取陀螺仪数据
 * @note 数据经过AHRS姿态解算算法处理(矫正、滤波等)
 * @retval 陀螺仪角速度(rad/s)
 */
const IMU::Vector3f &IMU::getGyro() const
{
    return m_ahrs->getGyro();
}

/**
 * @brief 获取加速度计数据
 * @note 数据经过AHRS姿态解算算法处理(矫正、滤波等)
 * @retval 加速度(m/s²)
 */
const IMU::Vector3f &IMU::getAccel() const
{
    return m_ahrs->getAccel();
}

/**
 * @brief 获取机体坐标系下的运动加速度
 * @note 经过AHRS姿态解算算法处理加速度，去除重力分量
 * @retval 机体坐标系下的运动加速度(m/s²)
 */
const IMU::Vector3f &IMU::getMotionAccelBodyFrame() const
{
    return m_ahrs->getMotionAccelBodyFrame();
}

/**
 * @brief 获取地球坐标系下的运动加速度
 * @note 经过AHRS姿态解算算法处理加速度，去除重力分量，并转换到地球坐标系
 * @retval 地球坐标系下的运动加速度(m/s²)
 */
const IMU::Vector3f &IMU::getMotionAccelEarthFrame() const
{
    return m_ahrs->getMotionAccelEarthFrame();
}

/**
 * @brief 获取四元数姿态
 * @retval 四元数
 */
const fp32 *IMU::getQuaternion() const
{
    return m_ahrs->getQuaternion();
}

/**
 * @brief 获取欧拉角姿态
 * @retval 欧拉角
 */
const IMU::Vector3f &IMU::getEulerAngle() const
{
    return m_ahrs->getEulerAngle();
}

/**
 * @brief IMU构造函数
 * @param ahrs 姿态解算算法接口
 */
IMU::IMU(AHRS *ahrs)
    : m_ahrs(ahrs), m_gyroRawData(), m_accelRawData(), m_magnetRawData() {}

/******************************************************************************
 *                            BMI088类实现
 ******************************************************************************/

/**
 * @brief BMI088构造函数
 * @param ahrs 姿态解算算法接口
 * @param accelSPIConfig 加速度计SPI配置
 * @param gyroSPIConfig 陀螺仪SPI配置
 * @param calibrationInfo 校准信息
 * @param errorCallback 错误回调函数
 * @param magnet 磁力计扩展类接口
 */
BMI088::BMI088(AHRS *ahrs, SPIConfig accelSPIConfig, SPIConfig gyroSPIConfig, CalibrationInfo calibrationInfo, ErrorCallback errorCallback, IST8310 *magnet,TIM_HandleTypeDef *heatTim, uint32_t heatChannel)
    : IMU(ahrs), m_accelSPIConfig(accelSPIConfig), m_gyroSPIConfig(gyroSPIConfig), m_calibrationInfo(calibrationInfo), m_errorCallback(errorCallback), m_magnet(magnet), m_heatTim(heatTim), m_heatChannel(heatChannel)
{
    m_errorCode = BMI088_NO_ERROR;
}

/**
 * @brief 初始化BMI088
 * @note 在进入循环解算前调用一次
 */
bool BMI088::init()
{
    m_errorCode = BMI088_NO_ERROR;
    if (m_heatTim != NULL) {
        HAL_TIM_PWM_Start(m_heatTim, m_heatChannel);
    }

    // self test pass and init
    if (selfTestAccel()) {
        initAccel();
    } else {
        handleError(BMI088_SELF_TEST_ACCEL_ERROR);
        return false;
    }

    if (selfTestGyro()) {
        initGyro();
    } else {
        handleError(BMI088_SELF_TEST_GYRO_ERROR);
        return false;
    }
    return true;
}

/**
 * @brief 读取BMI088原始数据
 */
void BMI088::readRawData()
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    readMultiReg(m_accelSPIConfig, BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp   = (int16_t)((buf[1]) << 8) | buf[0];
    m_accelRawData[0] = bmi088_raw_temp * ACCEL_SEN;
    bmi088_raw_temp   = (int16_t)((buf[3]) << 8) | buf[2];
    m_accelRawData[1] = bmi088_raw_temp * ACCEL_SEN;
    bmi088_raw_temp   = (int16_t)((buf[5]) << 8) | buf[4];
    m_accelRawData[2] = bmi088_raw_temp * ACCEL_SEN;

    readMultiReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
        bmi088_raw_temp  = (int16_t)((buf[3]) << 8) | buf[2];
        m_gyroRawData[0] = bmi088_raw_temp * GYRO_SEN;
        bmi088_raw_temp  = (int16_t)((buf[5]) << 8) | buf[4];
        m_gyroRawData[1] = bmi088_raw_temp * GYRO_SEN;
        bmi088_raw_temp  = (int16_t)((buf[7]) << 8) | buf[6];
        m_gyroRawData[2] = bmi088_raw_temp * GYRO_SEN;
    }
    readMultiReg(m_accelSPIConfig, BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023) {
        bmi088_raw_temp -= 2048;
        
    }

    m_temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    controlTemperature();
}
void BMI088::controlTemperature()
{
    
    if (m_temperature < -40.0f || m_temperature > 120.0f) {
        if (m_heatTim != NULL) __HAL_TIM_SET_COMPARE(m_heatTim, m_heatChannel, 0);
        return;
    }

    
    float error = m_tempTarget - m_temperature;

    if (error > -5.0f && error < 5.0f) m_tempErrorSum += error;

    if (m_tempErrorSum > m_tempMaxIout) m_tempErrorSum = m_tempMaxIout;
    else if (m_tempErrorSum < -m_tempMaxIout) m_tempErrorSum = -m_tempMaxIout;

    float output = (m_tempKp * error) + (m_tempKi * m_tempErrorSum) + (m_tempKd * (error - m_tempLastError));
    m_tempLastError = error;

    if (output > m_tempMaxOut) output = m_tempMaxOut;
    else if (output < 0.0f) output = 0.0f;

  
    if (m_heatTim != NULL) {
        __HAL_TIM_SET_COMPARE(m_heatTim, m_heatChannel, (uint32_t)output);
    }
}

/**
 * @brief 根据校准信息对数据进行校准, 修正安装误差与补偿零飘
 */
void BMI088::dataCalibration()
{
    m_gyroData  = m_calibrationInfo.installSpinMatrix * m_gyroRawData + m_calibrationInfo.gyroOffset;
    m_accelData = m_calibrationInfo.installSpinMatrix * m_accelRawData + m_calibrationInfo.accelOffset;
}

/**
 * @brief 初始化加速度计
 * @retval true 初始化成功
 * @retval false 初始化失败
 */
bool BMI088::initAccel()
{
    uint8_t res           = 0;
    uint8_t write_reg_num = 0;

    static const uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
        {
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
            {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
            {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
            {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

        };
    // check commiunication
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    // accel software reset
    writeSingleReg(m_accelSPIConfig, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);

    // check commiunication is normal after reset
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    // set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {

        writeSingleReg(m_accelSPIConfig, write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        readSingleReg(m_accelSPIConfig, write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1]) {
            handleError((ErrorCode)write_BMI088_accel_reg_data_error[write_reg_num][2]);
            return false;
        }
    }
    return true;
}

/**
 * @brief 加速度计自检
 * @retval true 自检通过
 * @retval false 自检失败
 */
bool BMI088::selfTestAccel()
{

    int16_t self_test_accel[2][3];

    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    uint8_t res    = 0;

    uint8_t write_reg_num = 0;

    static const uint8_t write_BMI088_ACCEL_self_test_Reg_Data_Error[6][3] =
        {
            {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
            {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR}

        };

    // check commiunication is normal
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    // reset  bmi088 accel sensor and wait for > 50ms
    writeSingleReg(m_accelSPIConfig, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);

    // check commiunication is normal
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    // set the accel register
    for (write_reg_num = 0; write_reg_num < 4; write_reg_num++) {

        writeSingleReg(m_accelSPIConfig, write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        readSingleReg(m_accelSPIConfig, write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]) {
            handleError((ErrorCode)write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][2]);
            return false;
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        osDelay(BMI088_LONG_DELAY_TIME);
    }

    // self test include postive and negative
    for (write_reg_num = 0; write_reg_num < 2; write_reg_num++) {

        writeSingleReg(m_accelSPIConfig, write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        readSingleReg(m_accelSPIConfig, write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]) {
            handleError((ErrorCode)write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][2]);
            return false;
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        osDelay(BMI088_LONG_DELAY_TIME);

        // read response accel
        readMultiReg(m_accelSPIConfig, BMI088_ACCEL_XOUT_L, buf, 6);

        self_test_accel[write_reg_num][0] = (int16_t)((buf[1]) << 8) | buf[0];
        self_test_accel[write_reg_num][1] = (int16_t)((buf[3]) << 8) | buf[2];
        self_test_accel[write_reg_num][2] = (int16_t)((buf[5]) << 8) | buf[4];
    }

    // set self test off
    writeSingleReg(m_accelSPIConfig, BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_accelSPIConfig, BMI088_ACC_SELF_TEST, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != (BMI088_ACC_SELF_TEST_OFF)) {
        handleError(BMI088_ACC_SELF_TEST_ERROR);
        return false;
    }

    // reset the accel sensor
    writeSingleReg(m_accelSPIConfig, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) || (self_test_accel[0][1] - self_test_accel[1][1] < 1365) || (self_test_accel[0][2] - self_test_accel[1][2] < 680)) {
        handleError(BMI088_SELF_TEST_ACCEL_ERROR);
        return false;
    }

    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return true;
}

/**
 * @brief 初始化陀螺仪
 * @retval true 初始化成功
 * @retval false 初始化失败
 */
bool BMI088::initGyro()
{
    uint8_t write_reg_num = 0;
    uint8_t res           = 0;
    static const uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
        {
            {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
            {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
            {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
            {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
            {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
            {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

        };

    // check commiunication
    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    // reset the gyro sensor
    writeSingleReg(m_gyroSPIConfig, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);
    // check commiunication is normal after reset
    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    // set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {

        writeSingleReg(m_gyroSPIConfig, write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        readSingleReg(m_gyroSPIConfig, write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1]) {
            handleError((ErrorCode)write_BMI088_gyro_reg_data_error[write_reg_num][2]);
            return false;
        }
    }
    return true;
}

/**
 * @brief 陀螺仪自检
 * @retval true 自检通过
 * @retval false 自检失败
 */
bool BMI088::selfTestGyro()
{
    uint8_t res   = 0;
    uint8_t retry = 0;
    // check commiunication is normal
    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        handleError(BMI088_NO_SENSOR);
        return false;
    }
    // reset the gyro sensor
    writeSingleReg(m_gyroSPIConfig, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);
    // check commiunication is normal after reset
    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    writeSingleReg(m_gyroSPIConfig, BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    osDelay(BMI088_LONG_DELAY_TIME);

    do {

        readSingleReg(m_gyroSPIConfig, BMI088_GYRO_SELF_TEST, res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        retry++;
    } while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10) {
        handleError(BMI088_SELF_TEST_GYRO_ERROR);
        return false;
    }

    if (res & BMI088_GYRO_BIST_FAIL) {
        handleError(BMI088_SELF_TEST_GYRO_ERROR);
        return false;
    }

    return true;
}

/**
 * @brief 错误处理, 根据错误码调用错误回调函数
 * @param errorCode 错误码
 */
inline void BMI088::handleError(BMI088::ErrorCode errorCode)
{
    m_errorCode = errorCode;
    if (m_errorCode != BMI088_NO_ERROR && m_errorCallback != nullptr)
        m_errorCallback(errorCode);
}

/**
 * @brief 读取单个寄存器
 * @param spiConfig SPI配置
 * @param reg 寄存器地址
 * @param rxData 接收数据
 */
inline void BMI088::readSingleReg(const SPIConfig &spiConfig, uint8_t reg, uint8_t &rxData)
{
    reg |= 0x80;                                                                   // 设置读操作位
    uint8_t dummy = 0x55;                                                          // 占位符
    HAL_GPIO_WritePin(spiConfig.csGPIOGroup, spiConfig.csGPIOPin, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(spiConfig.hspi, &reg, 1, 1000);                               // 发送寄存器地址
    if (&spiConfig == &m_accelSPIConfig) {
        HAL_SPI_Transmit(spiConfig.hspi, &dummy, 1, 1000); // 仅加速度计需要发送Dummy字节
    }
    HAL_SPI_Receive(spiConfig.hspi, &rxData, 1, 1000);                           // 接收数据
    HAL_GPIO_WritePin(spiConfig.csGPIOGroup, spiConfig.csGPIOPin, GPIO_PIN_SET); // CS HIGH
}

/**
 * @brief 读取多个寄存器
 * @param spiConfig SPI配置
 * @param reg 寄存器地址
 * @param pRxData 接收数据
 * @param length 数据长度
 */
inline void BMI088::readMultiReg(const SPIConfig &spiConfig, uint8_t reg, uint8_t *pRxData, uint8_t length)
{
    reg |= 0x80;                                                                   // 设置读操作位
    uint8_t dummy = 0x55;                                                          // 占位符
    HAL_GPIO_WritePin(spiConfig.csGPIOGroup, spiConfig.csGPIOPin, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(spiConfig.hspi, &reg, 1, 1000);                               // 发送寄存器地址
    if (&spiConfig == &m_accelSPIConfig) {
        HAL_SPI_Transmit(spiConfig.hspi, &dummy, 1, 1000); // 仅加速度计需要发送Dummy字节
    }
    HAL_SPI_Receive(spiConfig.hspi, pRxData, length, 1000);                      // 接收数据
    HAL_GPIO_WritePin(spiConfig.csGPIOGroup, spiConfig.csGPIOPin, GPIO_PIN_SET); // CS HIGH
}

/**
 * @brief 写入单个寄存器
 * @param spiConfig SPI配置
 * @param reg 寄存器地址
 * @param txData 发送数据
 */
inline void BMI088::writeSingleReg(const SPIConfig &spiConfig, uint8_t reg, const uint8_t &txData)
{
    HAL_GPIO_WritePin(spiConfig.csGPIOGroup, spiConfig.csGPIOPin, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(spiConfig.hspi, &reg, 1, 1000);                               // 发送寄存器地址
    HAL_SPI_Transmit(spiConfig.hspi, &txData, 1, 1000);                            // 发送数据
    HAL_GPIO_WritePin(spiConfig.csGPIOGroup, spiConfig.csGPIOPin, GPIO_PIN_SET);   // CS HIGH
}
