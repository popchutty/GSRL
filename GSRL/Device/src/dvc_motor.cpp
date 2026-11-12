/**
 ******************************************************************************
 * @file           : dvc_motor.cpp
 * @brief          : 电机模块
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_motor.hpp"
#include "alg_general.hpp"
#include "std_typedef.h"
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
 * @brief 设置前馈值
 * @param feedForward 前馈变量
 * @note 请在每次控制电机前调用，电机控制方法被调用后会自动清零前馈值
 * @note 只对闭环方法生效，对硬件控制无效
 * @note 前馈在极性反转后叠加
 */
void Motor::setFeedForwardOnce(fp32 feedForward)
{
    m_feedForward = feedForward;
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
    applyControllerOutputPostProcess();
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
    applyControllerOutputPostProcess();
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
    applyControllerOutputPostProcess();
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
    applyControllerOutputPostProcess();
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
    applyControllerOutputPostProcess();
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
      m_feedForward(0.0f),
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

/**
 * @brief 电机接收到控制器输出的后处理
 * @note 前馈自动归零
 * @note 前馈在控制器极性调换之后应用
 */
inline void Motor::applyControllerOutputPostProcess()
{
    if (m_controllerOutputPolarity) {
        m_controllerOutput = -m_controllerOutput;
    }
    m_controllerOutput += m_feedForward;
    m_feedForward = 0.0f;
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
 * @brief 合并两个同控制ID的大疆电机CAN控制数据, 同时触发双方掉线检测
 * @param otherMotor 另一个同控制ID的大疆电机
 * @return const uint8_t* 合并后的8字节CAN控制数据
 * @note 返回的指针指向内部静态缓冲区, 下次调用会覆盖
 */
const uint8_t *MotorGM6020::getMergedControlData(MotorGM6020 &otherMotor)
{
    const uint8_t *selfData  = this->getMotorControlData();
    const uint8_t *otherData = otherMotor.getMotorControlData();

    if (m_motorControlMessageID != otherMotor.m_motorControlMessageID) {
        return selfData;
    }

    memcpy(m_mergedData, selfData, 8);
    uint8_t offset             = (uint8_t)(((otherMotor.m_djiMotorID - 1) & 0x3) * 2);
    m_mergedData[offset]     = otherData[offset];
    m_mergedData[offset + 1] = otherData[offset + 1];
    return m_mergedData;
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
MotorM3508::MotorM3508(uint8_t dji3508MotorID, Controller *controller, uint16_t encoderOffset, fp32 gearboxRatio)
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
 *                        达妙一控四固件电机类实现
 ******************************************************************************/

/**
 * @brief 达妙一控四固件电机类构造函数, 用于初始化电机参数
 * @param dmMotorID 达妙电机ID, 取值范围 [1, 8]
 * @param controller 电机绑定控制器
 * @param encoderOffset 电机编码器偏移量, 默认为0
 * @note 达妙一控四电机ID对应关系如下:
 * | 电机ID |   1   |   2   |   3   |   4   |   5   |   6   |   7   |   8   |
 * | 反馈ID | 0x301 | 0x302 | 0x303 | 0x304 | 0x305 | 0x306 | 0x307 | 0x308 |
 * | 控制ID |             0x3FE             |             0x4FE             |
 */
MotorDMmulti::MotorDMmulti(uint8_t dmMotorID, Controller *controller, uint16_t encoderOffset)
    : Motor(dmMotorID <= 4 ? 0x3FE : 0x4FE,
            0x300u + dmMotorID,
            controller,
            encoderOffset),
      m_dmMotorID(dmMotorID),
      m_currentRPMSpeed(0),
      m_errorState(0)
{
    m_encoderHistory[0] = 0;
    m_encoderHistory[1] = 0;
}

/**
 * @brief 将控制器输出转换为达妙一控四电机CAN控制数据
 * @note 数据段为小端字节序, 与大疆GM6020的大端字节序相反
 */
void MotorDMmulti::convertControllerOutputToMotorControlData()
{
    int16_t giveControlValue = (int16_t)m_controllerOutput; // 控制电流标幺值
    uint8_t offset           = (uint8_t)(((m_dmMotorID - 1) & 0x3) * 2);
    m_motorControlData[offset]     = (uint8_t)(giveControlValue & 0xFF);        // 低 8 位
    m_motorControlData[offset + 1] = (uint8_t)((giveControlValue >> 8) & 0xFF); // 高 8 位
}

/**
 * @brief 解析达妙一控四电机CAN反馈数据核心函数
 * @param rxMessage CAN接收消息
 * @return true ID匹配, 解析成功
 * @return false ID不匹配, 解析失败
 * @note 内部函数, 被decodeCanRxMessageFromQueue或decodeCanRxMessageFromISR调用
 * @note 反馈帧格式: D[0:1]位置(0~8191), D[2:3]速度(RPM*100), D[4:5]扭矩电流(mA), D[6]线圈温度, D[7]错误状态
 */
bool MotorDMmulti::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    if (rxMessage.header.StdId != m_motorFeedbackMessageID) return false;

    m_encoderHistory[1] = m_encoderHistory[0];
    m_encoderHistory[0] = (uint16_t)((((rxMessage.data[0] << 8) | rxMessage.data[1]) - m_encoderOffset) & 0x1FFF);
    m_currentAngle      = (fp32)m_encoderHistory[0] * 2.0f * MATH_PI / 8192.0f;

    int16_t rawSpeed100      = (int16_t)((rxMessage.data[2] << 8) | rxMessage.data[3]);
    m_currentRPMSpeed        = (int16_t)(rawSpeed100 / 100);                          // 真实RPM
    m_currentAngularVelocity = (fp32)rawSpeed100 * (2.0f * MATH_PI / 60.0f) / 100.0f; // 保留精度

    m_currentTorqueCurrent = (int16_t)((rxMessage.data[4] << 8) | rxMessage.data[5]); // mA
    m_temperature          = (int8_t)rxMessage.data[6];                               // ℃
    m_errorState           = rxMessage.data[7];
    return true;
}

/**
 * @brief 获取达妙电机ID
 * @return uint8_t 达妙电机ID
 */
uint8_t MotorDMmulti::getDmMotorID() const
{
    return m_dmMotorID;
}

/**
 * @brief 获取达妙电机错误状态
 * @return uint8_t 反馈帧D[7]错误状态字节, 具体含义详见达妙错误状态说明书
 */
uint8_t MotorDMmulti::getErrorState() const
{
    return m_errorState;
}

/**
 * @brief 合并两个同控制ID的一控四固件达妙电机CAN控制数据, 同时触发双方掉线检测
 * @param otherMotor 另一个同控制ID的达妙一控四电机
 * @return const uint8_t* 合并后的8字节CAN控制数据
 * @note 返回的指针指向内部静态缓冲区, 下次调用会覆盖
 */
const uint8_t *MotorDMmulti::getMergedControlData(MotorDMmulti &otherMotor)
{
    const uint8_t *selfData  = this->getMotorControlData();
    const uint8_t *otherData = otherMotor.getMotorControlData();

    if (m_motorControlMessageID != otherMotor.m_motorControlMessageID) {
        return selfData;
    }

    memcpy(m_mergedData, selfData, 8);
    uint8_t offset             = (uint8_t)(((otherMotor.m_dmMotorID - 1) & 0x3) * 2);
    m_mergedData[offset]     = otherData[offset];
    m_mergedData[offset + 1] = otherData[offset + 1];
    return m_mergedData;
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
MotorLKMG::MotorLKMG(uint8_t lkMotorID, Controller *controller, uint16_t encoderOffset, fp32 gearboxRatio)
    : Motor(0x140u + lkMotorID,
            0x140u + lkMotorID,
            controller,
            encoderOffset),
      m_lkMotorID(lkMotorID),
      m_encoderRaw(0),
      m_gearboxRatio(gearboxRatio),
      m_isBraked(true) {}

/**
 * @brief 解析MG电机反馈数据核心函数
 * @param rxMessage CAN接收消息结构体
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
        case 0x9C:
        case 0xA0:
        case 0xA1:
        case 0xA2:
        case 0xAD:
        case 0xA3:
        case 0xA4:
        case 0xA5:
        case 0xA6:
        case 0xA7:
        case 0xA8: {
            m_temperature                = (int8_t)rxMessage.data[1];
            m_currentTorqueCurrent       = (int16_t)((rxMessage.data[3] << 8) | rxMessage.data[2]);
            int16_t speedDegreePerSecond = (int16_t)((rxMessage.data[5] << 8) | rxMessage.data[4]);
            m_currentAngularVelocity     = (fp32)speedDegreePerSecond * (MATH_PI / 180.0f) / m_gearboxRatio;
            m_encoderRaw                 = (uint16_t)((rxMessage.data[7] << 8) | rxMessage.data[6]);
            uint16_t encAdj              = (uint16_t)(m_encoderRaw - m_encoderOffset);
            m_currentAngle               = (fp32)encAdj * 2.0f * MATH_PI / (fp32)m_encoderResolution;
        } break;
        case 0x8C: {
            m_isBraked = !(bool)rxMessage.data[1];
        } break;
        // 有需要再根据协议添加解析其他命令
        default:
            return false;
    }
    return true;
}

/**
 * @brief 将控制器输出转换为MG系列电机CAN控制数据(力矩闭环)
 */
void MotorLKMG::convertControllerOutputToMotorControlData()
{
    if (m_isBraked) {
        setBrake(false);
    } else {
        int16_t iqControl = (int16_t)m_controllerOutput;
        GSRLMath::constrain(iqControl, m_openloopLimit); // 控制量范围 -2048~2048 对应协议
        m_motorControlData[0] = 0xA1;                    // 命令字节：转矩闭环
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
 * @brief 电机硬件角速度闭环控制
 * @note 正反转由符号决定
 */
void MotorLKMG::hardwareAngularVelocityClosedloopControl()
{
    if (m_isBraked) {
        setBrake(false);
    } else {
        int32_t speedControl  = (int32_t)(m_targetAngularVelocity * 100.0f * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
        uint8_t *speedBytes   = (uint8_t *)&speedControl;
        m_motorControlData[0] = 0xA2;
        m_motorControlData[1] = 0x00;
        m_motorControlData[2] = 0x00;
        m_motorControlData[3] = 0x00;
        m_motorControlData[4] = speedBytes[0];
        m_motorControlData[5] = speedBytes[1];
        m_motorControlData[6] = speedBytes[2];
        m_motorControlData[7] = speedBytes[3];
    }
}

/**
 * @brief 电机硬件角速度闭环控制
 * @param targetAngularVelocity 目标角速度，单位rad/s
 * @note 正反转由符号决定
 */
void MotorLKMG::hardwareAngularVelocityClosedloopControl(fp32 targetAngularVelocity)
{
    m_targetAngularVelocity = targetAngularVelocity;
    hardwareAngularVelocityClosedloopControl();
}

/**
 * @brief 电机硬件单圈角度闭环控制
 */
void MotorLKMG::hardwareAngleClosedloopControl()
{
    if (m_isBraked) {
        setBrake(false);
    } else {
        int32_t targetAngleControl = (int32_t)(m_targetAngle * 100.0f * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
        uint8_t *angleBytes        = (uint8_t *)&targetAngleControl;
        float maxVelocityDps       = m_maxVelocity;
        int32_t maxVelocityControl = (int32_t)(maxVelocityDps * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
        uint8_t *velocityBytes     = (uint8_t *)&maxVelocityControl;
        m_motorControlData[0]      = 0xA6;
        m_motorControlData[1]      = m_isMotorClockwise;
        m_motorControlData[2]      = velocityBytes[0];
        m_motorControlData[3]      = velocityBytes[1];
        m_motorControlData[4]      = angleBytes[0];
        m_motorControlData[5]      = angleBytes[1];
        m_motorControlData[6]      = angleBytes[2];
        m_motorControlData[7]      = angleBytes[3];
    }
}

/**
 * @brief 电机硬件单圈角度闭环控制
 * @param targetAngle 目标角度，单位rad [0, 2PI)
 * @param maxVelocity 最大允许角速度，单位rad/s
 * @param isMotorClockwise 是否顺时针旋转到目标角度，true表示顺时针，false表示逆时针
 */
void MotorLKMG::hardwareAngleClosedloopControl(fp32 targetAngle, fp32 maxVelocity, bool isMotorClockwise)
{
    if (m_isBraked) {
        setBrake(false);
    }
    m_targetAngle      = targetAngle;
    m_maxVelocity      = maxVelocity;
    m_isMotorClockwise = !isMotorClockwise;
    hardwareAngleClosedloopControl();
}

/**
 * @brief 电机硬件多圈角度闭环控制
 */
void MotorLKMG::hardwareRevolutionsClosedloopControl()
{
    if (m_isBraked) {
        setBrake(false);
    } else {
        int32_t targetAngleControl = (int32_t)(m_targetAngle * 100.0f * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
        uint8_t *angleBytes        = (uint8_t *)&targetAngleControl;
        float maxVelocityDps       = m_maxVelocity;
        int32_t maxVelocityControl = (int32_t)(maxVelocityDps * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
        uint8_t *velocityBytes     = (uint8_t *)&maxVelocityControl;
        m_motorControlData[0]      = 0xA4;
        m_motorControlData[1]      = 0x00;
        m_motorControlData[2]      = velocityBytes[0];
        m_motorControlData[3]      = velocityBytes[1];
        m_motorControlData[4]      = angleBytes[0];
        m_motorControlData[5]      = angleBytes[1];
        m_motorControlData[6]      = angleBytes[2];
        m_motorControlData[7]      = angleBytes[3];
    }
}

/**
 * @brief 电机硬件多圈角度闭环控制
 * @param targetAngle 目标角度，无限制，单位rad
 * @param maxVelocity 最大允许角速度，单位rad/s
 */
void MotorLKMG::hardwareRevolutionsClosedloopControl(fp32 targetAngle, fp32 maxVelocity)
{
    m_targetAngle = targetAngle;
    m_maxVelocity = maxVelocity;
    revolutionsClosedloopControl();
}

/**
 * @brief 设置MG电机刹车(抱闸器)状态
 * @param isBraked 是否刹车，true表示刹车，false表示释放刹车
 */
void MotorLKMG::setBrake(bool isBraked)
{
    if (isBraked) {
        m_motorControlData[0] = 0x8C;
        m_motorControlData[1] = 0x00;
        m_motorControlData[2] = 0x00;
        m_motorControlData[3] = 0x00;
        m_motorControlData[4] = 0x00;
        m_motorControlData[5] = 0x00;
        m_motorControlData[6] = 0x00;
        m_motorControlData[7] = 0x00;
    } else {
        m_motorControlData[0] = 0x8C;
        m_motorControlData[1] = 0x01;
        m_motorControlData[2] = 0x00;
        m_motorControlData[3] = 0x00;
        m_motorControlData[4] = 0x00;
        m_motorControlData[5] = 0x00;
        m_motorControlData[6] = 0x00;
        m_motorControlData[7] = 0x00;
    }
}

/******************************************************************************
 *                        瓴控广播模式(一控四)电机类实现
 ******************************************************************************/

/**
 * @brief 瓴控广播模式电机类构造函数, 用于初始化电机参数
 * @param lkMotorID 瓴控电机ID, 取值范围 [1, 4]
 * @param controller 绑定的控制器, 只使用硬件闭环和开环控制时可为nullptr
 * @param encoderOffset 编码器偏移量，默认为0
 * @param gearboxRatio 电机减速比，用于换算电机转速、角速度和硬件闭环目标值，
 *                     电机角度不受此值影响，默认为1即不换算，非有限正数按1处理
 * @param feedbackBaseID 反馈帧ID基地址, 默认0x140; 以匹配固件文档、配置和实测回复为准
 * @param encoderResolution 编码器位数, 默认为16位, 需与电机型号一致, 否则角度和圈数错误
 * @note 瓴控广播模式电机ID对应关系如下:
 * | 电机ID   |   1   |   2   |   3   |   4   |
 * | 反馈ID   | 基地址+1 | 基地址+2 | 基地址+3 | 基地址+4 |
 * | 数据槽位 | D[0:1] | D[2:3] | D[4:5] | D[6:7] |
 * | 控制ID   | 0x280力矩 / 0x281速度 / 0x282位置 / 0x288混合命令 |
 * @note 电机ID超出范围时该对象失效: 不写入控制数据, 不解析反馈, 不参与合帧,
 *       可通过isMotorIDValid()检查
 */
MotorLKBroadcast::MotorLKBroadcast(uint8_t lkMotorID, Controller *controller, uint16_t encoderOffset, fp32 gearboxRatio, LKFeedbackBaseID feedbackBaseID, LKEncoderResolution encoderResolution, LKMotorFamily motorFamily)
    : Motor(TORQUE_CONTROL,
            (lkMotorID >= 1 && lkMotorID <= m_maxMotorID) ? (uint32_t)feedbackBaseID + lkMotorID : m_invalidFeedbackID,
            controller,
            encoderOffset),
      m_lkMotorID(lkMotorID),
      m_motorFamily(motorFamily),
      m_isMotorIDValid(lkMotorID >= 1 && lkMotorID <= m_maxMotorID),
      m_dataOffset((uint8_t)(((lkMotorID - 1) & 0x3) * 2)),
      m_encoderRaw(0),
      m_encoderMask(encoderResolution == ENCODER_14BIT ? 0x3FFF : (encoderResolution == ENCODER_15BIT ? 0x7FFF : 0xFFFF)),
      m_gearboxRatio((isfinite(gearboxRatio) && gearboxRatio > 0.0f) ? gearboxRatio : 1.0f),
      m_hardwareTargetAngle(0.0f),
      m_busVoltage(0.0f),
      m_busCurrent(0.0f),
      m_motorState(m_motorStateOff),
      m_errorState(0),
      m_isLastMergeComplete(false),
      m_mergedData{}
{
    // 基类未初始化的帧头字段, 避免非静态对象发送时读取到不确定的值
    m_motorControlHeader.ExtId              = 0;
    m_motorControlHeader.TransmitGlobalTime = DISABLE;
}

// LK按时间判断在线, 读取控制缓冲本身不再推进连接状态。
const uint8_t *MotorLKBroadcast::getMotorControlData()
{
    return m_motorControlData;
}

bool MotorLKBroadcast::isMotorConected() const
{
    return m_hasFeedback && (uint32_t)(HAL_GetTick() - m_lastFeedbackMs) < 100;
}

bool MotorLKBroadcast::isState1Fresh(uint32_t maxAgeMs) const
{
    return m_hasState1 && (uint32_t)(HAL_GetTick() - m_lastState1Ms) < maxAgeMs;
}

bool MotorLKBroadcast::isState2Fresh(uint32_t maxAgeMs) const
{
    return m_hasState2 && (uint32_t)(HAL_GetTick() - m_lastState2Ms) < maxAgeMs;
}

bool MotorLKBroadcast::isMotorStateKnown(uint32_t maxAgeMs) const
{
    return isMotorConected() && m_hasMotorState &&
           (uint32_t)(HAL_GetTick() - m_lastMotorStateMs) < maxAgeMs;
}

bool MotorLKBroadcast::isRevolutionsValid() const
{
    return m_revolutionsValid && isState2Fresh();
}

void MotorLKBroadcast::resetCurrentRevolutionsToZero()
{
    Motor::resetCurrentRevolutionsToZero();
    m_revolutionsValid = isState2Fresh();
}

fp32 MotorLKBroadcast::updateCurrentRevolutions()
{
    // 状态1/开关应答没有角度, 不能参与跨零计数。
    if (!m_positionPending) return m_currentRevolutions;
    m_positionPending = false;
    return Motor::updateCurrentRevolutions();
}

bool MotorLKBroadcast::decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t Size)
{
    if (rxMessage == nullptr) return false;
    bool decoded = false;
    // 同一批可能既有状态1又有状态2, 且跨零过程不能被逆序/只取最新帧丢弃。
    // 缺少接收时间的旧/手工队列帧不能用于证明在线。直接ISR调用可用当前时间。
    for (uint8_t i = 0; i < Size; i++) {
        if (rxMessage[i].receivedAtValid != 1) continue;
        if (decodeCanRxMessageFromISR(&rxMessage[i])) decoded = true;
    }
    return decoded;
}

bool MotorLKBroadcast::buildControlFrame(MotorLKBroadcast *const motors[], uint8_t count,
                                        uint8_t activeMask, CAN_TxHeaderTypeDef &header, uint8_t (&data)[8])
{
    header = {};
    memset(data, 0, sizeof(data));
    if (motors == nullptr || count == 0 || count > 4 || activeMask == 0 || (activeMask & 0xF0)) return false;

    uint8_t occupied = 0;
    uint32_t frameID = 0;
    for (uint8_t i = 0; i < count; i++) {
        const MotorLKBroadcast *motor = motors[i];
        if (motor == nullptr || !motor->m_isMotorIDValid || !motor->isMotorFamilyValid()) return false;
        const uint8_t bit = (uint8_t)(1u << (motor->m_lkMotorID - 1));
        if ((occupied & bit) || !(activeMask & bit)) return false;
        if (i == 0) frameID = motor->m_motorControlMessageID;
        if (motor->m_motorControlMessageID != frameID) return false;
        occupied |= bit;
    }
    if (occupied != activeMask) return false;

    // 全部校验通过后再发布; 缺失电机不能被默认为位置零目标。
    header = motors[0]->m_motorControlHeader;
    for (uint8_t i = 0; i < count; i++) {
        const uint8_t offset = motors[i]->m_dataOffset;
        data[offset] = motors[i]->m_motorControlData[offset];
        data[offset + 1] = motors[i]->m_motorControlData[offset + 1];
    }
    return true;
}

/**
 * @brief 解析瓴控广播模式电机反馈数据核心函数
 * @param rxMessage CAN接收消息结构体
 * @return true ID匹配，解析成功
 * @return false ID不匹配或不是可识别的回复，解析失败
 * @note 内部函数, 被decodeCanRxMessageFromQueue或decodeCanRxMessageFromISR调用
 * @note 广播模式下电机的回复与单电机模式对应命令的回复相同, 按命令字节D[0]区分:
 *       状态2格式(0x9C及控制命令0xA0~0xA8): D[1]温度, D[2:3]转矩电流, D[4:5]速度(dps), D[6:7]编码器
 *       状态1格式(0x9A、0x9B): D[1]温度, D[2:3]母线电压(0.01V), D[4:5]母线电流(0.01A), D[6]电机状态, D[7]错误标志
 *       开关命令(0x80、0x88、0x81): 回复与发送相同, 只用于确认电机在线和开关状态
 */
bool MotorLKBroadcast::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    if (!m_isMotorIDValid || !isMotorFamilyValid()) return false;
    if (rxMessage.header.IDE != CAN_ID_STD || rxMessage.header.RTR != CAN_RTR_DATA) return false;
    if (rxMessage.header.StdId != m_motorFeedbackMessageID) return false;
    if (rxMessage.header.DLC != 8) return false;

    const uint32_t currentTime = HAL_GetTick();
    const uint32_t now = rxMessage.receivedAtValid == 1 ? rxMessage.receivedAtMs : currentTime;
    // uint32差值支持tick回绕; 拒绝未来/逆序时间, 避免旧帧覆盖较新状态。
    // 队列必须在2^31毫秒以内消费, HAL tick必须持续运行。
    if ((uint32_t)(currentTime - now) > 0x7FFFFFFFu ||
        (m_hasFeedback && (uint32_t)(now - m_lastFeedbackMs) > 0x7FFFFFFFu)) return false;
    switch (rxMessage.data[0]) {
        case READ_STATE_2:
        case 0xA0:
        case 0xA1:
        case 0xA2:
        case 0xA3:
        case 0xA4:
        case 0xA5:
        case 0xA6:
        case 0xA7:
        case 0xA8: {
            const bool firstPosition = !m_hasState2;
            const bool positionGap = m_hasState2 && (uint32_t)(now - m_lastState2Ms) >= 100;
            m_temperature                = (int8_t)rxMessage.data[1];
            m_currentTorqueCurrent       = (int16_t)((rxMessage.data[3] << 8) | rxMessage.data[2]);
            int16_t speedDegreePerSecond = (int16_t)((rxMessage.data[5] << 8) | rxMessage.data[4]);
            m_currentAngularVelocity     = (fp32)speedDegreePerSecond * (MATH_PI / 180.0f) / m_gearboxRatio;
            m_encoderRaw                 = (uint16_t)((rxMessage.data[7] << 8) | rxMessage.data[6]);
            uint16_t encoderAdjusted     = (uint16_t)((m_encoderRaw - m_encoderOffset) & m_encoderMask);
            m_currentAngle               = (fp32)encoderAdjusted * 2.0f * MATH_PI / ((fp32)m_encoderMask + 1.0f);
            if (firstPosition || positionGap) {
                // 首帧保留单圈绝对初值, 不虚构跨圈。断流后未知整圈不可恢复。
                m_lastAngle = m_currentAngle;
                m_revolutionsValid = firstPosition;
            }
            m_lastState2Ms = now;
            m_hasState2 = true;
            m_positionPending = true;
        } break;
        case READ_STATE_1:
        case CLEAR_ERROR: {
            m_temperature = (int8_t)rxMessage.data[1];
            m_busVoltage  = (fp32)(int16_t)((rxMessage.data[3] << 8) | rxMessage.data[2]) * 0.01f;
            m_busCurrent  = (fp32)(int16_t)((rxMessage.data[5] << 8) | rxMessage.data[4]) * 0.01f;
            m_motorState  = rxMessage.data[6];
            m_errorState  = rxMessage.data[7];
            m_lastState1Ms = now;
            m_hasState1 = true;
            m_lastMotorStateMs = now;
            m_hasMotorState = true;
        } break;
        case MOTOR_OFF:
            m_motorState = m_motorStateOff;
            m_lastMotorStateMs = now;
            m_hasMotorState = true;
            break;
        case MOTOR_ON:
            m_motorState = m_motorStateOn;
            m_lastMotorStateMs = now;
            m_hasMotorState = true;
            break;
        case MOTOR_STOP:
            break;
        default:
            return false;
    }
    m_lastFeedbackMs = now;
    m_hasFeedback = true;
    memcpy(m_motorFeedbackData, rxMessage.data, sizeof(m_motorFeedbackData));
    return true;
}

/**
 * @brief 将控制器输出转换为广播模式力矩控制数据(0x280)
 * @note MF/MG力矩控制量±2000, MS开环电压控制量±850; 超出范围限幅, NaN按0处理
 * @note m_controllerOutput会被更新为实际发送的控制量, 因此基类各闭环方法的返回值是限幅取整后的值
 */
void MotorLKBroadcast::convertControllerOutputToMotorControlData()
{
    const int16_t limit = m_motorFamily == MS ? m_voltageLimit : (m_motorFamily == MF_MG ? m_torqueLimit : 0);
    int16_t torqueControl = convertToControlValue(m_controllerOutput, -limit, limit);
    m_controllerOutput    = (fp32)torqueControl;
    setBroadcastControlData(TORQUE_CONTROL, (uint8_t)(torqueControl & 0xFF), (uint8_t)((torqueControl >> 8) & 0xFF));
}

/**
 * @brief 电机硬件角速度闭环控制(0x281)
 * @note 正反转由符号决定, 分辨率为电机轴1dps, 电机轴速度限幅到 -32768~32767dps
 */
void MotorLKBroadcast::hardwareAngularVelocityClosedloopControl()
{
    fp32 speedDegreePerSecond = m_targetAngularVelocity * m_gearboxRatio * (180.0f / MATH_PI); // 转换为电机轴速度，单位度每秒
    int16_t speedControl      = convertToControlValue(speedDegreePerSecond, -32768, 32767);
    setBroadcastControlData(SPEED_CONTROL, (uint8_t)(speedControl & 0xFF), (uint8_t)((speedControl >> 8) & 0xFF));
}

/**
 * @brief 电机硬件角速度闭环控制(0x281)
 * @param targetAngularVelocity 目标角速度，单位rad/s
 * @note 正反转由符号决定
 */
void MotorLKBroadcast::hardwareAngularVelocityClosedloopControl(fp32 targetAngularVelocity)
{
    m_targetAngularVelocity = targetAngularVelocity;
    hardwareAngularVelocityClosedloopControl();
}

/**
 * @brief 电机硬件位置闭环控制(0x282)
 * @param targetAngle 目标绝对位置，单位rad，带符号，范围约 ±5.72rad / 减速比，超出范围限幅
 * @note 目标以电机自身零点为基准并按减速比换算到电机轴, 分辨率为电机轴0.01°, 电机轴角度范围 -327.68°~327.67°
 * @note 该目标与基类软件角度闭环的目标(setTargetAngle, 以编码器偏移量为零点的单圈角度)相互独立
 * @note 位置帧中未写入的槽位代表0°位置, 需为总线上所有电机设置目标后再合帧发送
 */
void MotorLKBroadcast::hardwareAngleClosedloopControl(fp32 targetAngle)
{
    m_hardwareTargetAngle = targetAngle;
    fp32 angleCentidegree = m_hardwareTargetAngle * m_gearboxRatio * (18000.0f / MATH_PI); // 转换为电机轴角度，单位0.01度
    int16_t angleControl  = convertToControlValue(angleCentidegree, -32768, 32767);
    setBroadcastControlData(ANGLE_CONTROL, (uint8_t)(angleControl & 0xFF), (uint8_t)((angleControl >> 8) & 0xFF));
}

/**
 * @brief 设置混合命令帧(0x288)中本电机执行的命令
 * @param command 电机命令, 不在LKBroadcastCommand范围内的值会被忽略
 * @note 调用后本电机的控制帧ID变为0x288, 直到下一次调用其他控制方法
 */
void MotorLKBroadcast::setMotorCommand(LKBroadcastCommand command)
{
    switch (command) {
        case MOTOR_OFF:
        case MOTOR_STOP:
        case MOTOR_ON:
        case READ_STATE_1:
        case CLEAR_ERROR:
        case READ_STATE_2:
            setBroadcastControlData(MIXED_COMMAND, (uint8_t)command, 0x00);
            break;
        default:
            break;
    }
}

/**
 * @brief 合并两个同控制帧ID的瓴控广播模式电机CAN控制数据
 * @param otherMotor 另一个同控制帧ID的瓴控广播模式电机
 * @return const uint8_t* 合并后的8字节CAN控制数据, 帧头使用本电机的getMotorControlHeader()
 * @note 返回的指针指向内部缓冲区, 下次调用会覆盖
 * @note 控制帧ID与本电机不一致、电机ID非法或重复的电机会被跳过, 不会覆盖已写入的槽位,
 *       此时isLastMergeComplete()返回false并返回nullptr, 禁止发送部分帧
 * @note isLastMergeComplete()只表示传入的电机是否全部写入, 不表示4个槽位齐全或发送成功
 */
const uint8_t *MotorLKBroadcast::getMergedControlData(MotorLKBroadcast &otherMotor)
{
    MotorLKBroadcast *const otherMotors[] = {&otherMotor};
    return mergeControlData(otherMotors, 1);
}

/**
 * @brief 合并三个同控制帧ID的瓴控广播模式电机CAN控制数据
 * @param otherMotor1 第二个电机
 * @param otherMotor2 第三个电机
 * @return const uint8_t* 合并后的8字节CAN控制数据
 */
const uint8_t *MotorLKBroadcast::getMergedControlData(MotorLKBroadcast &otherMotor1, MotorLKBroadcast &otherMotor2)
{
    MotorLKBroadcast *const otherMotors[] = {&otherMotor1, &otherMotor2};
    return mergeControlData(otherMotors, 2);
}

/**
 * @brief 合并四个同控制帧ID的瓴控广播模式电机CAN控制数据
 * @param otherMotor1 第二个电机
 * @param otherMotor2 第三个电机
 * @param otherMotor3 第四个电机
 * @return const uint8_t* 合并后的8字节CAN控制数据
 */
const uint8_t *MotorLKBroadcast::getMergedControlData(MotorLKBroadcast &otherMotor1, MotorLKBroadcast &otherMotor2, MotorLKBroadcast &otherMotor3)
{
    MotorLKBroadcast *const otherMotors[] = {&otherMotor1, &otherMotor2, &otherMotor3};
    return mergeControlData(otherMotors, 3);
}

/**
 * @brief 写入本电机在广播帧中的槽位, 同时切换控制帧ID
 * @param frameID 控制帧ID
 * @param lowByte 槽位低字节
 * @param highByte 槽位高字节
 * @note 只写本电机的2字节槽位, 其余字节恒为0
 * @note 该函数仅供内部或派生类调用
 */
void MotorLKBroadcast::setBroadcastControlData(LKBroadcastFrameID frameID, uint8_t lowByte, uint8_t highByte)
{
    if (!m_isMotorIDValid || !isMotorFamilyValid()) return;

    m_motorControlMessageID              = frameID;
    m_motorControlHeader.StdId           = frameID;
    m_motorControlData[m_dataOffset]     = lowByte;
    m_motorControlData[m_dataOffset + 1] = highByte;
}

/**
 * @brief 合并本电机与其他电机的广播帧槽位
 * @param otherMotors 其他电机指针数组
 * @param otherMotorCount 其他电机数量
 * @return const uint8_t* 合并后的8字节CAN控制数据
 * @note 读取控制缓冲不会更新在线状态; 在线状态由接收时间决定
 * @note 只拷贝各电机自己的槽位, 以本电机的控制帧ID为准
 * @note 该函数仅供内部或派生类调用
 */
const uint8_t *MotorLKBroadcast::mergeControlData(MotorLKBroadcast *const otherMotors[], uint8_t otherMotorCount)
{
    const uint8_t *selfData = getMotorControlData();
    uint8_t occupiedSlots   = 0; // 已写入槽位的位图, 用于检查电机ID重复

    memset(m_mergedData, 0, sizeof(m_mergedData));
    m_isLastMergeComplete = m_isMotorIDValid && isMotorFamilyValid();
    if (m_isLastMergeComplete) {
        m_mergedData[m_dataOffset]     = selfData[m_dataOffset];
        m_mergedData[m_dataOffset + 1] = selfData[m_dataOffset + 1];
        occupiedSlots |= (uint8_t)(1u << (m_dataOffset / 2));
    }

    for (uint8_t i = 0; i < otherMotorCount; i++) {
        MotorLKBroadcast *otherMotor = otherMotors[i];
        // 同一个电机对象重复传入时判定合帧失败
        bool isRepeated = (otherMotor == this);
        for (uint8_t j = 0; j < i && !isRepeated; j++) {
            isRepeated = (otherMotors[j] == otherMotor);
        }
        if (isRepeated) {
            m_isLastMergeComplete = false;
            continue;
        }

        const uint8_t *otherData = otherMotor->getMotorControlData();
        uint8_t otherOffset      = otherMotor->m_dataOffset;
        uint8_t otherSlot        = (uint8_t)(1u << (otherOffset / 2));
        if (!otherMotor->m_isMotorIDValid || !otherMotor->isMotorFamilyValid() ||
            otherMotor->m_motorControlMessageID != m_motorControlMessageID ||
            (occupiedSlots & otherSlot) != 0) {
            m_isLastMergeComplete = false;
            continue;
        }
        m_mergedData[otherOffset]     = otherData[otherOffset];
        m_mergedData[otherOffset + 1] = otherData[otherOffset + 1];
        occupiedSlots |= otherSlot;
    }
    return m_isLastMergeComplete ? m_mergedData : nullptr;
}

/**
 * @brief 将浮点控制量限幅并四舍五入为16位整型控制量
 * @param value 浮点控制量
 * @param minimum 下限
 * @param maximum 上限
 * @return int16_t 范围 minimum~maximum 的控制量, 输入为NaN时返回0
 * @note 先在浮点域限幅再转换, 避免超出int16范围的浮点数转换产生回绕
 */
int16_t MotorLKBroadcast::convertToControlValue(fp32 value, int16_t minimum, int16_t maximum)
{
    if (isnan(value)) return 0;
    // 先在浮点域舍入并限幅, 包括INT16_MIN附近, 避免转换越界。
    value = roundf(value);
    if (value >= (fp32)maximum) return maximum;
    if (value <= (fp32)minimum) return minimum;
    return (int16_t)value;
}
