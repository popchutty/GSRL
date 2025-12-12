/**
 ******************************************************************************
 * @file           : dvc_motor.cpp
 * @brief          : 电机模块
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_motor.hpp"
#include "alg_general.hpp"
#include <math.h>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                           Motor电机基类实现
 ******************************************************************************/

/**
 * @brief 获取电机CAN控制消息ID
 * @return uint32_t 电机CAN控制消息ID
 */
uint32_t Motor::getMotorControlMessageID() const
{
    return m_motorControlMessageID;
}

/**
 * @brief 获取电机CAN反馈消息ID
 * @return uint32_t 电机CAN反馈消息ID
 */
uint32_t Motor::getMotorFeedbackMessageID() const
{
    return m_motorFeedbackMessageID;
}

/**
 * @brief 获取电机CAN控制消息头结构体指针
 * @return const CAN_TxHeaderTypeDef* 电机CAN控制消息头指针
 */
const CAN_TxHeaderTypeDef *Motor::getMotorControlHeader() const
{
    return &m_motorControlHeader;
}

/**
 * @brief 获取电机电机CAN控制数组数据, 同时检测电机连接状态
 * @return const uint8_t* 电机CAN控制数据motroControlData[8]
 */
const uint8_t *Motor::getMotorControlData()
{
    // 检测电机连接状态
    if (m_motorFeedbackSequence == m_motorLastFeedbackSequence) {
        increaseMotorFeedbackErrorCount();
    } else {
        m_motorLastFeedbackSequence = m_motorFeedbackSequence; // 滚动更新反馈数据序号
    }

    return m_motorControlData;
}

/**
 * @brief 从CAN接收缓冲数组中从后往前解析电机反馈数据
 * @param rxMessage CAN接收缓冲数组
 * @param Size CAN接收缓冲数组大小
 * @return bool 解析成功返回true，否则返回false
 * @note 从后往前解析, 缓冲数组中数据应从旧到新排列
 * @note Size可小于等于缓冲数组大小，推荐设置为接收到的CAN包数量
 * @example 若FreeRTOS消息队列长度为16:
 * uint8_t rxCount = 0;
 * for (rxCount = 0; rxCount < 16; rxCount++) {
 *     if (osMessageQueueGet(canRxQueueHandle, &rxMessage[rxCount], NULL, 0) != osOK) {
 *         break;
 *     }
 * }
 * motor1.decodeCanRxMessageFromQueue(rxMessage, rxCount);
 * motor2.decodeCanRxMessageFromQueue(rxMessage, rxCount);
 * ...
 */
bool Motor::decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t Size)
{
    for (uint8_t i = Size; i > 0; i--) {
        if (decodeCanRxMessage(rxMessage[i - 1])) {
            clearMotorFeedbackErrorCount();
            updateCurrentRevolutions();
            return true;
        }
    }
    increaseMotorFeedbackErrorCount(); // 未解析到数据增加错误计数
    return false;
}

/**
 * @brief 从中断服务函数中解析电机反馈数据
 * @param rxMessage CAN接收消息
 * @return bool 解析成功返回true，否则返回false
 * @note 该函数仅供中断服务函数调用
 * @note 返回false可能是ID不匹配，属正常情况
 */
bool Motor::decodeCanRxMessageFromISR(const can_rx_message_t *rxMessage)
{
    if (decodeCanRxMessage(*rxMessage)) {
        m_motorFeedbackSequence++;
        clearMotorFeedbackErrorCount();
        updateCurrentRevolutions();
        return true;
    }
    return false;
}

/**
 * @brief 获取电机CAN反馈原始数据
 * @return const uint8_t* 电机CAN反馈原始数据motorFeedbackData[8]
 */
const uint8_t *Motor::getMotorFeedbackData() const
{
    return m_motorFeedbackData;
}

/**
 * @brief 获取电机当前角度
 * @return fp32 电机当前角度 单位rad [0, 2PI)
 */
fp32 Motor::getCurrentAngle() const
{
    return m_currentAngle;
}

/**
 * @brief 获取电机当前角速度
 * @return fp32 电机当前角速度 单位rad/s
 */
fp32 Motor::getCurrentAngularVelocity() const
{
    return m_currentAngularVelocity;
}

/**
 * @brief 电机旋转圈数清零
 */
void Motor::resetCurrentRevolutionsToZero()
{
    m_currentRevolutions = 0.0f;
    m_roundCount         = 0;
    m_zeroAngle          = m_currentAngle;
    m_lastAngle          = m_currentAngle;
}

/**
 * @brief 获取电机当前旋转圈数
 * @return fp32 电机当前旋转圈数
 */
fp32 Motor::getCurrentRevolutions() const
{
    return m_currentRevolutions;
}

/**
 * @brief 获取电机当前力矩电流
 * @return int16_t 电机当前力矩电流
 */
int16_t Motor::getCurrentTorqueCurrent() const
{
    return m_currentTorqueCurrent;
}

/**
 * @brief 获取电机当前温度
 * @return uint8_t 电机当前温度 单位℃
 */
int8_t Motor::getTemperature() const
{
    return m_temperature;
}

/**
 * @brief 检查电机是否连接
 * @return true 电机连接正常
 * @return false 电机连接异常
 */
bool Motor::isMotorConected() const
{
    return m_isMotorConnected;
}

/**
 * @brief 设置电机目标角度
 * @param targetAngle 电机目标角度 单位rad [0, 2PI)
 */
void Motor::setTargetAngle(fp32 targetAngle)
{
    m_targetAngle = targetAngle;
}

/**
 * @brief 设置电机目标角速度
 * @param targetAngularVelocity 电机目标角速度 单位rad/s
 */
void Motor::setTargetAngularVelocity(fp32 targetAngularVelocity)
{
    m_targetAngularVelocity = targetAngularVelocity;
}

/**
 * @brief 设置电机目标旋转圈数
 * @param targetRevolutions 电机目标旋转圈数
 */
void Motor::setTargetRevolutions(fp32 targetRevolutions)
{
    m_targetRevolutions = targetRevolutions;
}

/**
 * @brief 设置电机目标力矩电流
 * @param targetTorqueCurrent 电机目标力矩电流
 */
void Motor::setTargetTorqueCurrent(int16_t targetTorqueCurrent)
{
    m_targetTorqueCurrent = targetTorqueCurrent;
}

/**
 * @brief 设置电机绑定的控制器
 * @param controller 电机控制器
 */
void Motor::setController(Controller *controller)
{
    m_controller = controller;
}

/**
 * @brief 设置电机控制器输出极性
 * @param polarity 电机控制器输出极性
 */
void Motor::setControllerOutputPolarity(bool polarity)
{
    m_controllerOutputPolarity = polarity;
}

/**
 * @brief 电机开环控制
 * @param controlValue 控制值
 */
void Motor::openloopControl(fp32 controlValue)
{
    m_controllerOutput = controlValue;
    convertControllerOutputToMotorControlData();
}

/**
 * @brief 角度闭环控制
 * @return fp32 控制器输出
 * @note 可使用串级PID，反馈数据数组顺序为[角度误差, 角速度]
 */
fp32 Motor::angleClosedloopControl()
{
    fp32 angleError      = GSRLMath::normalizeDeltaAngle(m_currentAngle - m_targetAngle);
    fp32 feedBackData[2] = {angleError, m_currentAngularVelocity};
    m_controllerOutput   = m_controller->controllerCalculate(0.0f, feedBackData, 2);
    if (m_controllerOutputPolarity) {
        m_controllerOutput = -m_controllerOutput;
    }
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

/**
 * @brief 角度闭环控制
 * @param targetAngle 目标角度 单位rad [0, 2PI)
 * @return fp32 控制器输出
 */
fp32 Motor::angleClosedloopControl(fp32 targetAngle)
{
    m_targetAngle = targetAngle;
    return angleClosedloopControl();
}

/**
 * @brief 角速度闭环控制
 * @return fp32 控制器输出
 */
fp32 Motor::angularVelocityClosedloopControl()
{
    fp32 feedBackData[1] = {m_currentAngularVelocity};
    m_controllerOutput   = m_controller->controllerCalculate(m_targetAngularVelocity, feedBackData, 1);
    if (m_controllerOutputPolarity) {
        m_controllerOutput = -m_controllerOutput;
    }
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

/**
 * @brief 角速度闭环控制
 * @param targetAngularVelocity 目标角速度 单位rad/s
 * @return fp32 控制器输出
 */
fp32 Motor::angularVelocityClosedloopControl(fp32 targetAngularVelocity)
{
    m_targetAngularVelocity = targetAngularVelocity;
    return angularVelocityClosedloopControl();
}

/**
 * @brief 旋转圈数闭环控制
 * @return fp32 控制器输出
 * @note 可使用串级PID，反馈数据数组顺序为[当前多圈圈数, 角速度]
 */
fp32 Motor::revolutionsClosedloopControl()
{
    fp32 feedBackData[2] = {m_currentRevolutions, m_currentAngularVelocity};
    m_controllerOutput   = m_controller->controllerCalculate(m_targetRevolutions, feedBackData, 2);
    if (m_controllerOutputPolarity) {
        m_controllerOutput = -m_controllerOutput;
    }
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

/**
 * @brief 旋转圈数闭环控制
 * @param targetRevolutions 目标旋转圈数
 * @return fp32 控制器输出
 */
fp32 Motor::revolutionsClosedloopControl(fp32 targetRevolutions)
{
    m_targetRevolutions = targetRevolutions;
    return revolutionsClosedloopControl();
}

/**
 * @brief 力矩电流闭环控制
 * @return int16_t 控制器输出
 */
int16_t Motor::torqueCurrentClosedloopControl()
{
    fp32 feedBackData[1] = {(fp32)m_currentTorqueCurrent};
    m_controllerOutput   = m_controller->controllerCalculate((fp32)m_targetTorqueCurrent, feedBackData, 1);
    if (m_controllerOutputPolarity) {
        m_controllerOutput = -m_controllerOutput;
    }
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

/**
 * @brief 力矩电流闭环控制
 * @param targetTorqueCurrent 目标力矩电流
 * @return int16_t 控制器输出
 */
int16_t Motor::torqueCurrentClosedloopControl(int16_t targetTorqueCurrent)
{
    m_targetTorqueCurrent = targetTorqueCurrent;
    return torqueCurrentClosedloopControl();
}

/**
 * @brief 外部反馈数据闭环控制
 * @param setPoint 目标值
 * @param feedBackData 反馈数据数组
 * @param feedBackSize 反馈数据数组大小
 */
fp32 Motor::externalClosedloopControl(fp32 setPoint, const fp32 *feedBackData, uint8_t feedBackSize)
{
    m_controllerOutput = m_controller->controllerCalculate(setPoint, feedBackData, feedBackSize);
    if (m_controllerOutputPolarity) {
        m_controllerOutput = -m_controllerOutput;
    }
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

/**
 * @brief 电机基类构造函数，用于初始化电机基本参数
 * @param canControlID 电机CAN控制ID
 * @param canFeedbackID 电机CAN反馈ID
 * @param controller 电机绑定控制器
 * @param encoderOffset 电机编码器偏移量，默认为0
 * @note 该构造函数不可被直接调用，需由派生类构造函数调用
 */
Motor::Motor(uint32_t canControlID, uint32_t canFeedbackID, Controller *controller, uint16_t encoderOffset)
    : m_motorControlMessageID(canControlID),
      m_motorFeedbackMessageID(canFeedbackID),
      m_motorFeedbackSequence(0),
      m_motorLastFeedbackSequence(0),
      m_currentAngle(0.0f), m_lastAngle(0.0f),
      m_currentAngularVelocity(0.0f),
      m_roundCount(0),
      m_zeroAngle(0.0f),
      m_currentRevolutions(0.0f),
      m_currentTorqueCurrent(0),
      m_temperature(0),
      m_motorFeedbackErrorCount(0),
      m_isMotorConnected(false),
      m_targetAngle(0.0f),
      m_targetAngularVelocity(0.0f),
      m_targetRevolutions(0.0f),
      m_targetTorqueCurrent(0),
      m_controller(controller),
      m_controllerOutput(0.0f),
      m_controllerOutputPolarity(false),
      m_encoderOffset(encoderOffset)
{
    m_motorControlHeader.DLC   = 8;
    m_motorControlHeader.IDE   = CAN_ID_STD;
    m_motorControlHeader.RTR   = CAN_RTR_DATA;
    m_motorControlHeader.StdId = m_motorControlMessageID;
    memset(m_motorControlData, 0, sizeof(m_motorControlData));
    memset(m_motorFeedbackData, 0, sizeof(m_motorFeedbackData));
}

/**
 * @brief 更新电机当前旋转圈数
 * @return fp32 电机当前旋转圈数
 * @note 该函数调用周期应小于电机旋转周期的一半
 * @note 内部函数，电机接收到数据后会自动调用
 */
fp32 Motor::updateCurrentRevolutions()
{
    if (m_currentAngle - m_lastAngle > MATH_PI) {
        m_roundCount--;
    } else if (m_currentAngle - m_lastAngle < -MATH_PI) {
        m_roundCount++;
    }
    m_currentRevolutions = m_roundCount + (m_currentAngle - m_zeroAngle) / (2 * MATH_PI);
    m_lastAngle          = m_currentAngle;
    return m_currentRevolutions;
}

/**
 * @brief 增加电机反馈错误计数，同时更新电机连接状态
 * @note 电机反馈错误计数超过2则认为电机连接异常
 * @note 该函数仅供内部或派生类调用
 */
inline void Motor::increaseMotorFeedbackErrorCount()
{
    if (m_motorFeedbackErrorCount < 255) {
        m_motorFeedbackErrorCount++;
    }
    m_isMotorConnected = m_motorFeedbackErrorCount < 10;
}

/**
 * @brief 清除电机反馈错误计数，同时更新电机连接状态
 * @note 该函数仅供内部或派生类调用
 */
inline void Motor::clearMotorFeedbackErrorCount()
{
    m_motorFeedbackErrorCount = 0;
    m_isMotorConnected        = true;
}

/******************************************************************************
 *                           GM6020电机类实现
 ******************************************************************************/

/**
 * @brief GM6020电机类构造函数, 用于初始化GM6020电机参数
 * @param dji6020MotorID GM6020电机ID
 * @param controller 电机绑定控制器
 * @param encoderOffset 电机编码器偏移量，默认为0
 * @note GM6020电机ID对应关系如下:
 * | 电机ID |   1   |   2   |   3   |   4   |   5   |   6   |   7   |
 * | 反馈ID | 0x205 | 0x206 | 0x207 | 0x208 | 0x209 | 0x20A | 0x20B |
 * | 控制ID |             0x1FF             |         0x2FF         |
 */
MotorGM6020::MotorGM6020(uint8_t dji6020MotorID, Controller *controller, uint16_t encoderOffset)
    : Motor(dji6020MotorID < 5 ? 0x1FF : 0x2FF,
            dji6020MotorID + 0x204,
            controller,
            encoderOffset),
      m_djiMotorID(dji6020MotorID) {}

/**
 * @brief 将控制器输出转换为GM6020电机CAN控制数据
 */
void MotorGM6020::convertControllerOutputToMotorControlData()
{
    int16_t giveControlValue = (int16_t)m_controllerOutput; // 给定控制量, 根据驱动版本可能是电压或电流
    if (m_djiMotorID < 5) {
        m_motorControlData[m_djiMotorID * 2 - 2] = giveControlValue >> 8;
        m_motorControlData[m_djiMotorID * 2 - 1] = giveControlValue;
    } else {
        m_motorControlData[m_djiMotorID * 2 - 10] = giveControlValue >> 8;
        m_motorControlData[m_djiMotorID * 2 - 9]  = giveControlValue;
    }
}

/**
 * @brief 解析GM6020电机CAN反馈数据核心函数
 * @param rxMessage CAN接收消息
 * @return true ID匹配，解析成功
 * @return false ID不匹配，解析失败
 * @note 内部函数, 被decodeCanRxMessageFromQueue或decodeCanRxMessageFromISR调用
 */
bool MotorGM6020::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    if (rxMessage.header.StdId != m_motorFeedbackMessageID) return false;

    m_encoderHistory[1]      = m_encoderHistory[0];
    m_encoderHistory[0]      = (uint16_t)((((rxMessage.data[0] << 8) | rxMessage.data[1]) - m_encoderOffset) & 0x1FFF);
    m_currentAngle           = (fp32)m_encoderHistory[0] * 2 * MATH_PI / 8192;
    m_currentRPMSpeed        = (int16_t)((rxMessage.data[2] << 8) | rxMessage.data[3]);
    m_currentAngularVelocity = (fp32)m_currentRPMSpeed * 2 * MATH_PI / 60;
    m_currentTorqueCurrent   = (int16_t)((rxMessage.data[4] << 8) | rxMessage.data[5]);
    m_temperature            = rxMessage.data[6];
    return true;
}

/**
 * @brief 获取大疆电机ID
 * @return uint8_t 大疆电机ID
 */
uint8_t MotorGM6020::getDjiMotorID() const
{
    return m_djiMotorID;
}

/**
 * @brief 大疆电机加法运算符重载，用于合并CAN控制数据
 * @param otherMotor 另一个同控制ID的大疆电机
 */
MotorGM6020 MotorGM6020::operator+(const MotorGM6020 &otherMotor) const
{
    if (m_motorControlMessageID != otherMotor.m_motorControlMessageID) {
        return *this;
    }

    MotorGM6020 combineMotor = *this;
    if (otherMotor.m_djiMotorID < 5) {
        uint8_t offset                              = otherMotor.m_djiMotorID * 2 - 2;
        combineMotor.m_motorControlData[offset]     = otherMotor.m_motorControlData[offset];
        combineMotor.m_motorControlData[offset + 1] = otherMotor.m_motorControlData[offset + 1];
    } else {
        uint8_t offset                              = otherMotor.m_djiMotorID * 2 - 10;
        combineMotor.m_motorControlData[offset]     = otherMotor.m_motorControlData[offset];
        combineMotor.m_motorControlData[offset + 1] = otherMotor.m_motorControlData[offset + 1];
    }
    return combineMotor;
}

/******************************************************************************
 *                            M3508电机类实现
 ******************************************************************************/

/**
 * @brief M3508电机类构造函数, 用于初始化M3508电机参数
 * @param dji3508MotorID M3508电机ID
 * @param controller 电机绑定控制器
 * @param encoderOffset 电机编码器偏移量，默认为0
 * @param gearboxRatio 电机减速比倒数，用于换算电机转速、角速度和圈数，
 *                     电机角度不受此值影响，默认为1即不换算
 * @note M3508电机ID对应关系如下:
 * | 电机ID |   1   |   2   |   3   |   4   |   5   |   6   |   7   |
 * | 反馈ID | 0x201 | 0x202 | 0x203 | 0x204 | 0x205 | 0x206 | 0x207 |
 * | 控制ID |             0x200             |         0x1FF         |
 */
MotorM3508::MotorM3508(uint8_t dji3508MotorID, Controller *controller, uint16_t encoderOffset, uint8_t gearboxRatio)
    : MotorGM6020(dji3508MotorID, controller, encoderOffset),
      m_gearboxRatio(gearboxRatio)
{
    // 调用GM6020构造函数后修正为M3508电机发送和接收ID
    m_motorControlMessageID    = dji3508MotorID < 5 ? 0x200 : 0x1FF;
    m_motorFeedbackMessageID   = dji3508MotorID + 0x200;
    m_motorControlHeader.StdId = m_motorControlMessageID;
}

/**
 * @brief 解析M3508电机反馈数据核心函数
 * @param rxMessage CAN接收消息
 * @return true ID匹配，解析成功
 * @return false ID不匹配，解析失败
 * @note 内部函数, 被decodeCanRxMessageFromQueue或decodeCanRxMessageFromISR调用
 */
bool MotorM3508::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    // 调用GM6020解析函数后修正M3508电机转速和角速度
    if (MotorGM6020::decodeCanRxMessage(rxMessage)) {
        m_currentRPMSpeed /= m_gearboxRatio;
        m_currentAngularVelocity /= m_gearboxRatio;
        return true;
    }
    return false;
}

/******************************************************************************
 *                           DMJ4310电机类实现
 ******************************************************************************/

/**
 * @brief 达妙DMJ4310电机类构造函数, 用于初始化DMJ4310电机参数
 * @param dmControlID DMJ4310电机控制ID
 * @param dmMasterID DMJ4310电机反馈ID
 * @param pmax 电机最大位置
 * @param vmax 电机最大速度
 * @param tmax 电机最大力矩
 * @param controller 电机绑定控制器
 * @note PMAX、VMAX、TMAX需要与电机固件参数一致，PMAX设置为PI(3.141593)
 */
MotorDM4310::MotorDM4310(uint8_t dmControlID, uint8_t dmMasterID, fp32 pmax, fp32 vmax, fp32 tmax, Controller *controller)
    : Motor(dmControlID, dmMasterID, controller, 0),
      m_motorState(DISABLE),
      m_dmEncoderPosition(0.0f),
      m_mosfetTemperature(0.0f),
      m_coliTemperature(0.0f),
      m_setZeroPositionFlag(false),
      PMAX(pmax),
      VMAX(vmax),
      TMAX(tmax) {}

/**
 * @brief 将控制器输出转换为DMJ4310电机CAN控制数据
 * @note 在转换控制数据前会根据需求先转换特殊电机控制指令如清除错误、使能电机、设置零点
 */
void MotorDM4310::convertControllerOutputToMotorControlData()
{
    // 清除电机失联错误
    if (m_motorState == COMMUNICATION_LOST) {
        memset(m_motorControlData, 0xFF, sizeof(m_motorControlData));
        m_motorControlData[7] = 0xFB;
        return;
    }
    // 使能电机
    if (m_motorState == DISABLE) {
        memset(m_motorControlData, 0xFF, sizeof(m_motorControlData));
        m_motorControlData[7] = 0xFC;
        return;
    }
    // 设置电机零点
    if (m_setZeroPositionFlag) {
        memset(m_motorControlData, 0xFF, sizeof(m_motorControlData));
        m_motorControlData[7] = 0xFE;
        m_setZeroPositionFlag = false;
        return;
    }
    // 电机控制
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp               = GSRLMath::convertFloatToUint(0.0f, -PMAX, PMAX, 16);
    vel_tmp               = GSRLMath::convertFloatToUint(0.0f, -VMAX, VMAX, 12);
    kp_tmp                = GSRLMath::convertFloatToUint(0.0f, 0.0f, 500.0f, 12);
    kd_tmp                = GSRLMath::convertFloatToUint(0.0f, 0.0f, 5.0f, 12);
    tor_tmp               = GSRLMath::convertFloatToUint(m_controllerOutput, -TMAX, TMAX, 12);
    m_motorControlData[0] = (pos_tmp >> 8);
    m_motorControlData[1] = pos_tmp;
    m_motorControlData[2] = (vel_tmp >> 4);
    m_motorControlData[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    m_motorControlData[4] = kp_tmp;
    m_motorControlData[5] = (kd_tmp >> 4);
    m_motorControlData[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    m_motorControlData[7] = tor_tmp;
}

/**
 * @brief 解析DMJ4310电机反馈数据核心函数
 * @param rxMessage CAN接收消息
 * @return true ID匹配，解析成功
 * @return false ID不匹配，解析失败
 * @note 内部函数, 被decodeCanRxMessageFromQueue或decodeCanRxMessageFromISR调用
 */
bool MotorDM4310::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    if (rxMessage.header.StdId != m_motorFeedbackMessageID) return false;
    if ((rxMessage.data[0] & 0x0F) != m_motorControlMessageID) return false;

    m_motorState             = (DMMotorState)((rxMessage.data[0]) >> 4);
    int rxRawPosition        = (rxMessage.data[1] << 8) | rxMessage.data[2];
    int rxRawVelocity        = (rxMessage.data[3] << 4) | (rxMessage.data[4] >> 4);
    int rxRawTorque          = ((rxMessage.data[4] & 0xF) << 8) | rxMessage.data[5];
    m_dmEncoderPosition      = GSRLMath::convertUintToFloat(rxRawPosition, -PMAX, PMAX, 16); // (-12.5,12.5)
    m_currentAngle           = GSRLMath::normalizeAngle(m_dmEncoderPosition);                // [0,2PI)
    m_currentAngularVelocity = GSRLMath::convertUintToFloat(rxRawVelocity, -VMAX, VMAX, 12); // (-45.0,45.0)
    m_currentTorqueCurrent   = GSRLMath::convertUintToFloat(rxRawTorque, -TMAX, TMAX, 12);   // (-18.0,18.0)
    m_mosfetTemperature      = (float)(rxMessage.data[6]);
    m_coliTemperature        = (float)(rxMessage.data[7]);
    return true;
}

/**
 * @brief 设置电机反馈位置零点
 */
void MotorDM4310::setMotorZeroPosition()
{
    m_setZeroPositionFlag = true;
}

/******************************************************************************
 *                           瓴控MG系列电机类实现
 ******************************************************************************/

/**
 * @brief 瓴控MG系列电机类构造函数, 用于初始化电机参数
 * @param lkMotorID 瓴控电机ID (控制ID和反馈ID相同)
 * @param controller 绑定的控制器
 * @param encoderOffset 编码器偏移量，默认为0
 * @param gearboxRatio 电机减速比倒数，用于换算电机转速、角速度和圈数，
 *                     电机角度不受此值影响，默认为1即不换算
 * @note lkMotorID范围为1-32，对应反馈ID和控制ID均为0x140 + lkMotorID，Ox140为MG系列电机CAN ID基地址
 */
MotorLKMG::MotorLKMG(uint8_t lkMotorID, Controller *controller, uint16_t encoderOffset, uint8_t gearboxRatio)
    : Motor(0x140u + lkMotorID,
            0x140u + lkMotorID,
            controller,
            encoderOffset),
      m_lkMotorID(lkMotorID),
      m_encoderRaw(0),
      m_isBraked(0),//初始化默认为刹车状态
      m_gearboxRatio(gearboxRatio) {}

/**
 * @brief 解析MG电机反馈数据核心函数
 * @param rxMessage CAN接收消息，是结构体
 * @return true ID匹配，解析成功
 * @return false ID不匹配，解析失败
 * @note 内部函数, 被decodeCanRxMessageFromQueue或decodeCanRxMessageFromISR调用
 * @note 目前只解析0x9C命令，其他命令可根据需要添加
 *       0x9C命令数据格式参考MG系列电机协议文档，为：温度、扭矩电流、速度、原始编码器
 */
bool MotorLKMG::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    if (rxMessage.header.StdId != m_motorFeedbackMessageID) return false;

    uint8_t cmd = rxMessage.data[0];

    switch (cmd) {
        case 0xA1:
        case 0x9A:
        case 0xA2:
        case 0xAD:
        case 0xA3:
        case 0xA4:
        case 0xA5:
        case 0xA6:
        case 0xA7:
        case 0xA8:
            {
            m_temperature            = (int8_t)rxMessage.data[1];
            m_currentTorqueCurrent   = (int16_t)((rxMessage.data[3] << 8) | rxMessage.data[2]);
            int16_t speedDegreePerSecond   = (int16_t)((rxMessage.data[5] << 8) | rxMessage.data[4]);
            m_currentAngularVelocity = (fp32)speedDegreePerSecond * (MATH_PI / 180.0f) / m_gearboxRatio;
            m_encoderRaw             = (uint16_t)((rxMessage.data[7] << 8) | rxMessage.data[6]);
            uint16_t encAdj          = (uint16_t)(m_encoderRaw - m_encoderOffset);
            m_currentAngle           = (fp32)encAdj * 2.0f * MATH_PI / (fp32)m_encoderResolution;
            } break;
        case 0x8C:
            {
              m_isBraked = !(bool)rxMessage.data[1];
            } break;

            // 有需要再根据协议添加解析其他命令       
        default:
            return false;
    }
    return true;
}

/**
 * @brief 将控制器输出转换为MG系列电机CAN控制数据
 */
void MotorLKMG::convertControllerOutputToMotorControlData()
{
    if (m_isBraked)
    {
        hardwareBrakeControl(0);
    }
    else
    {
    int16_t iqControl = (int16_t)m_controllerOutput;
    GSRLMath::constrain(iqControl, m_openloopLimit); // 控制量范围 -2048~2048 对应协议
    m_motorControlData[0] = 0xA1; // 命令字节：转矩闭环
    m_motorControlData[1] = 0x00;
    m_motorControlData[2] = 0x00;
    m_motorControlData[3] = 0x00;
    m_motorControlData[4] = (uint8_t)(iqControl & 0xFF);        // 低字节
    m_motorControlData[5] = (uint8_t)((iqControl >> 8) & 0xFF); // 高字节
    m_motorControlData[6] = 0x00;
    m_motorControlData[7] = 0x00;
    }
}

/**
 * @brief 将角速度闭环控制输出值转换为MG电机CAN控制数据
 * @note 正反转由符号决定
 */
void MotorLKMG::hardwareAngularVelocityClosedloopControl()
{
    if (m_isBraked) {

        hardwareBrakeControl(0);
    }
    else
    {
    float targetSpeedDps  = m_targetAngularVelocity;
    int32_t speedControl  = (int32_t)(targetSpeedDps * 100.0f * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
    uint8_t *p            = (uint8_t *)&speedControl;
    m_motorControlData[0] = 0xA2;
    m_motorControlData[1] = 0x00;
    m_motorControlData[2] = 0x00;
    m_motorControlData[3] = 0x00;
    m_motorControlData[4] = p[0];
    m_motorControlData[5] = p[1];
    m_motorControlData[6] = p[2];
    m_motorControlData[7] = p[3];
    }
}

/**
 * @brief 将角速度闭环控制输出值转换为MG电机CAN控制数据
 * @param AngleVelocity 目标角速度，单位rad/s
 * @note 正反转由符号决定
 */
void MotorLKMG::hardwareAngularVelocityClosedloopControl(fp32 targetAngularVelocity)
{
    m_targetAngularVelocity = targetAngularVelocity;
    hardwareAngularVelocityClosedloopControl();
}

/**
 * @brief 将单圈角度闭环控制输出值转换为MG电机CAN控制数据
 * @note 顺逆转由isMotorClockwise决定，true表示顺时针，false表示逆时针
 */
void MotorLKMG::hardwareAngleClosedloopControl()
{
    if (m_isBraked) {
        hardwareBrakeControl(0);
    }
    else
    {
    float targetAngleDps       = m_targetAngle;
    int32_t targetAngleControl = (int32_t)(targetAngleDps * 100.0f * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
    uint8_t *p                 = (uint8_t *)&targetAngleControl;
    float maxVelocityDps       = m_maxVelocity;
    int32_t maxVelocityControl = (int32_t)(maxVelocityDps * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
    uint8_t *q                 = (uint8_t *)&maxVelocityControl;
    m_motorControlData[0]      = 0xA6;
    m_motorControlData[1]      = m_isMotorClockwise;
    m_motorControlData[2]      = q[0];
    m_motorControlData[3]      = q[1];
    m_motorControlData[4]      = p[0];
    m_motorControlData[5]      = p[1];
    m_motorControlData[6]      = p[2];

    m_motorControlData[7]      = p[3];
    }
}

/**
 * @brief 将单圈角度闭环控制输出值转换为MG电机CAN控制数据
 * @param targetAngle 目标角度，单位rad [0, 2PI)
 * @param maxVelocity 最大允许角速度，单位rad/s
 * @param isMotorClockwise 是否顺时针旋转到目标角度，true表示顺时针，false表示逆时针
 * @note 是由电机内置的PID控制器进行闭环控制
 */

void MotorLKMG::hardwareAngleClosedloopControl(fp32 targetAngle, fp32 maxVelocity, bool isMotorClockwise)
{
    if (m_isBraked) {
        hardwareBrakeControl(0);
    }
    m_targetAngle      = targetAngle;
    m_maxVelocity      = maxVelocity;
    m_isMotorClockwise = !isMotorClockwise;
    hardwareAngleClosedloopControl();
}

/**
 * @brief 将多圈角度闭环控制输出值转换为MG电机CAN控制数据
 * @param targetAngle 目标角度，无限制，单位rad
 * @param maxVelocity 最大允许角速度，单位rad/s
 * @note 由点击内部自带的PID控制器完成控制
 */
void MotorLKMG::hardwareRevolutionsClosedloopControl()
{
    if (m_isBraked)
    {
        hardwareBrakeControl(0);
    }
    else
    {
    float targetAngleDps       = m_targetAngle;
    int32_t targetAngleControl = (int32_t)(targetAngleDps * 100.0f * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
    uint8_t *p                 = (uint8_t *)&targetAngleControl;
    float maxVelocityDps       = m_maxVelocity;
    int32_t maxVelocityControl = (int32_t)(maxVelocityDps * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
    uint8_t *q                 = (uint8_t *)&maxVelocityControl;
    m_motorControlData[0]      = 0xA4;
    m_motorControlData[1]      = 0x00;
    m_motorControlData[2]      = q[0];
    m_motorControlData[3]      = q[1];
    m_motorControlData[4]      = p[0];
    m_motorControlData[5]      = p[1];
    m_motorControlData[6]      = p[2];
    m_motorControlData[7]      = p[3];
    }
}

/**
 * @brief 将多圈角度闭环控制输出值转换为MG电机CAN控制数据
 * @param targetAngle 目标角度，无限制，单位rad
 * @param maxVelocity 最大允许角速度，单位rad/s
 * @note 由点击内部自带的PID控制器完成控制
 */
void MotorLKMG::hardwareRevolutionsClosedloopControl(fp32 targetAngle, fp32 maxVelocity)
{
    m_targetAngle = targetAngle;
    m_maxVelocity = maxVelocity;
    revolutionsClosedloopControl();
}

void MotorLKMG::hardwareBrakeControl(bool isBraked)
{
    if (isBraked)
    {
        m_motorControlData[0] = 0x8C;
        m_motorControlData[1] = 0x01;
        m_motorControlData[2] = 0x00;
        m_motorControlData[3] = 0x00;
        m_motorControlData[4] = 0x00;
        m_motorControlData[5] = 0x00;
        m_motorControlData[6] = 0x00;
        m_motorControlData[7] = 0x00;
    }
    else if(!isBraked)
    {
        m_motorControlData[0] = 0x8C;
        m_motorControlData[1] = 0x00;
        m_motorControlData[2] = 0x00;
        m_motorControlData[3] = 0x00;
        m_motorControlData[4] = 0x00;
        m_motorControlData[5] = 0x00;
        m_motorControlData[6] = 0x00;
        m_motorControlData[7] = 0x00;
    }
}