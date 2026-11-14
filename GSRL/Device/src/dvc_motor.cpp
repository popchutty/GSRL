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
#include <limits>

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
            ++m_motorFeedbackSequence; // 与中断接收路径一致，仅在解析到有效反馈后更新序号。
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
    MotionFeedback motion;
    if (!readMotionFeedback(motion)) return zeroControllerOutput();
    const fp32 error = GSRLMath::normalizeDeltaAngle(motion.angle - m_targetAngle);
    const fp32 feedback[] = {error, motion.positionAngularVelocity};
    return runController(0.0f, feedback, 2);
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
    MotionFeedback motion;
    if (!readMotionFeedback(motion)) return zeroControllerOutput();
    return runController(m_targetAngularVelocity, &motion.angularVelocity, 1);
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
    MotionFeedback motion;
    if (!readMotionFeedback(motion) || !motion.revolutionsValid) return zeroControllerOutput();
    const fp32 feedback[] = {motion.revolutions, motion.positionAngularVelocity};
    return runController(m_targetRevolutions, feedback, 2);
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
    MotionFeedback motion;
    if (!readMotionFeedback(motion)) return static_cast<int16_t>(zeroControllerOutput());
    const fp32 current = static_cast<fp32>(motion.torqueCurrent);
    fp32 output = runController(m_targetTorqueCurrent, &current, 1);
    GSRLMath::constrain(output, static_cast<fp32>(INT16_MIN), static_cast<fp32>(INT16_MAX));
    return static_cast<int16_t>(output);
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
    return runController(setPoint, feedBackData, feedBackSize);
}

bool Motor::readMotionFeedback(MotionFeedback &feedback) const
{
    feedback = {m_currentAngle, m_currentAngularVelocity, m_currentAngularVelocity,
                m_currentRevolutions, m_currentTorqueCurrent, true};
    return true;
}

fp32 Motor::zeroControllerOutput()
{
    m_feedForward = 0.0f;
    openloopControl(0.0f);
    return 0.0f;
}

fp32 Motor::runController(fp32 target, const fp32 *feedback, uint8_t size)
{
    if (!m_controller || !feedback || size == 0 || !isfinite(target)) return zeroControllerOutput();
    for (uint8_t i = 0; i < size; ++i) {
        if (!isfinite(feedback[i])) return zeroControllerOutput();
    }
    m_controllerOutput = m_controller->controllerCalculate(target, feedback, size);
    applyControllerOutputPostProcess();
    if (!isfinite(m_controllerOutput)) return zeroControllerOutput();
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

namespace {
// STM32F4 上保证任务读取与 CAN 中断更新互斥。恢复原 PRIMASK，允许在临界区或 ISR 内调用。
class MotorIrqGuard
{
public:
    MotorIrqGuard() : m_primask(__get_PRIMASK()) { __disable_irq(); }
    ~MotorIrqGuard() { __set_PRIMASK(m_primask); }
    MotorIrqGuard(const MotorIrqGuard &) = delete;
    MotorIrqGuard &operator=(const MotorIrqGuard &) = delete;
private:
    uint32_t m_primask;
};

uint16_t readLE16(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0] | (uint32_t(p[1]) << 8));
}

int16_t readSignedLE16(const uint8_t *p)
{
    const int32_t raw = readLE16(p);
    return static_cast<int16_t>(raw >= 0x8000 ? raw - 0x10000 : raw);
}

int8_t signedTemperature(uint8_t byte)
{
    return static_cast<int8_t>(byte >= 128 ? int(byte) - 256 : byte);
}

void writeLE16(uint8_t *p, uint16_t value)
{
    p[0] = static_cast<uint8_t>(value);
    p[1] = static_cast<uint8_t>(value >> 8);
}

void writeLE32(uint8_t *p, uint32_t value)
{
    writeLE16(p, static_cast<uint16_t>(value));
    writeLE16(p + 2, static_cast<uint16_t>(value >> 16));
}

// 比较时保留整数上界的精度，避免 float(INT32_MAX) 被舍入为 2^31 后再强转。
// 沿用单电机协议原实现的向零截断。广播协议的 16 位目标另按最近整数取整。
template <class T>
bool protocolInteger(double value, T &encoded)
{
    if (!isfinite(value) || value < static_cast<double>(std::numeric_limits<T>::lowest()) ||
        value > static_cast<double>(std::numeric_limits<T>::max())) return false;
    encoded = static_cast<T>(value);
    return true;
}

constexpr double degreesPerRadian = 180.0 / static_cast<double>(MATH_PI);
} // namespace

MotorLKMGBase::MotorLKMGBase(uint32_t controlID, uint8_t motorID, uint8_t maxMotorID, Controller *controller,
                             uint16_t encoderOffset, uint8_t encoderBits, fp32 gearboxRatio, uint32_t feedbackTimeoutMs)
    : Motor(controlID, 0x180u + motorID, controller, encoderOffset),
      m_lkMotorID(motorID), m_gearboxRatio(gearboxRatio), m_maxMotorID(maxMotorID),
      m_encoderBits(encoderBits), m_feedbackTimeoutMs(feedbackTimeoutMs) {}

bool MotorLKMGBase::isConfigurationValid() const
{
    return m_lkMotorID >= 1 && m_lkMotorID <= m_maxMotorID &&
           m_encoderBits >= 14 && m_encoderBits <= 16 &&
           isfinite(m_gearboxRatio) && m_gearboxRatio > 0.0f && m_feedbackTimeoutMs > 0;
}

MotorLKMGBase::FeedbackState MotorLKMGBase::getFeedbackState() const
{
    const MotorIrqGuard lock;
    if (!m_hasReceivedFeedback) return FeedbackState::NeverReceived;
    return HAL_GetTick() - m_lastFeedbackMs < m_feedbackTimeoutMs ? FeedbackState::Online : FeedbackState::Offline;
}

bool MotorLKMGBase::isMotorConected() const
{
    return getFeedbackState() == FeedbackState::Online;
}

bool MotorLKMGBase::hasFreshMotionFeedback() const
{
    const MotorIrqGuard lock;
    return m_hasMotionFeedback && HAL_GetTick() - m_lastMotionMs < m_feedbackTimeoutMs;
}

bool MotorLKMGBase::readMotionFeedback(MotionFeedback &feedback) const
{
    const MotorIrqGuard lock;
    if (!hasFreshMotionFeedback()) return false;
    feedback = {m_currentAngle, m_currentAngularVelocity, m_motorShaftVelocity, m_currentRevolutions,
                m_currentTorqueCurrent, m_turnTracking == TurnTracking::Tracking};
    return true;
}

void MotorLKMGBase::resetCurrentRevolutionsToZero()
{
    const MotorIrqGuard lock;
    Motor::resetCurrentRevolutionsToZero();
    m_turnTracking = hasFreshMotionFeedback() ? TurnTracking::Tracking : TurnTracking::AwaitingFirst;
}

bool MotorLKMGBase::decodeCanRxMessageFromISR(const can_rx_message_t *message)
{
    return message && acceptFeedback(*message);
}

bool MotorLKMGBase::acceptFeedback(const can_rx_message_t &message)
{
    if (message.header.IDE != CAN_ID_STD || message.header.RTR != CAN_RTR_DATA ||
        message.header.DLC != 8 || message.header.StdId != m_motorFeedbackMessageID || !isConfigurationValid())
        return false;

    const MotorIrqGuard lock;
    if (!decodeCanRxMessage(message)) return false;
    memcpy(m_motorFeedbackData, message.data, sizeof(m_motorFeedbackData));
    m_lastFeedbackMs = HAL_GetTick();
    m_hasReceivedFeedback = true;
    return true;
}

bool MotorLKMGBase::decodeCanRxMessageFromQueue(const can_rx_message_t *messages, uint8_t size)
{
    if (!messages) return false;
    bool received = false;
    for (uint8_t i = 0; i < size; ++i) {
        if (acceptFeedback(messages[i])) received = true;
    }
    return received;
}

bool MotorLKMGBase::decodeMotionFeedback(const uint8_t (&data)[8])
{
    const uint32_t resolution = 1u << m_encoderBits;
    const uint16_t raw = readLE16(data + 6);
    if (raw >= resolution) return false;
    const fp32 shaftSpeed = readSignedLE16(data + 4) * (MATH_PI / 180.0f);
    const fp32 outputSpeed = shaftSpeed / m_gearboxRatio;
    if (!isfinite(outputSpeed)) return false;

    const uint32_t adjusted = (uint32_t(raw) - m_encoderOffset) & (resolution - 1u);
    m_currentAngle = adjusted * (2.0f * MATH_PI / resolution);
    m_motorShaftVelocity = shaftSpeed;
    m_currentAngularVelocity = outputSpeed;
    m_currentTorqueCurrent = readSignedLE16(data + 2);
    m_temperature = signedTemperature(data[1]);

    const uint32_t now = HAL_GetTick();
    if (m_turnTracking == TurnTracking::AwaitingFirst) {
        Motor::resetCurrentRevolutionsToZero();
        m_turnTracking = TurnTracking::Tracking;
    } else if (m_turnTracking == TurnTracking::Tracking) {
        const int32_t step = static_cast<int32_t>(adjusted) - m_lastEncoderCount;
        const int32_t half = static_cast<int32_t>(resolution / 2);
        if (now - m_lastMotionMs >= m_feedbackTimeoutMs || step == half || step == -half ||
            (step > half && m_roundCount == INT32_MIN) || (step < -half && m_roundCount == INT32_MAX)) {
            m_turnTracking = TurnTracking::Lost;
        } else {
            if (step > half) --m_roundCount;
            if (step < -half) ++m_roundCount;
            m_currentRevolutions = m_roundCount + (m_currentAngle - m_zeroAngle) / (2.0f * MATH_PI);
            m_lastAngle = m_currentAngle;
        }
    }
    m_lastEncoderCount = static_cast<uint16_t>(adjusted);
    m_lastMotionMs = now;
    m_hasMotionFeedback = true;
    return true;
}

MotorLKMG::MotorLKMG(uint8_t lkMotorID, Controller *controller, uint16_t encoderOffset, fp32 gearboxRatio,
                     uint8_t encoderBits, uint32_t feedbackTimeoutMs)
    : MotorLKMGBase(0x140u + lkMotorID, lkMotorID, 32, controller,
                    encoderOffset, encoderBits, gearboxRatio, feedbackTimeoutMs) {}

bool MotorLKMG::decodeCanRxMessage(const can_rx_message_t &message)
{
    switch (message.data[0]) {
        case 0x9C:
        case 0xA1: case 0xA2: case 0xA3: case 0xA4:
        case 0xA5: case 0xA6: case 0xA7: case 0xA8:
            return decodeMotionFeedback(message.data);
        case 0x8C:
            if (message.data[1] > 1) return false;
            m_brakeState = message.data[1] == 0 ? BrakeState::Engaged : BrakeState::Released;
            return true;
        default:
            return false;
    }
}

bool MotorLKMG::prepareMotionFrame(uint8_t command)
{
    const MotorIrqGuard lock;
    if (m_brakeState == BrakeState::Engaging) {
        setBrake(true); // 在收到抱闸确认前，后续控制调用不得覆盖仍待发送的抱闸命令。
        return false;
    }
    if (m_brakeState != BrakeState::Released || !isMotorConected()) {
        m_brakeState = BrakeState::Engaged; // 通信中断后重新取得松闸确认。
        setBrake(false);
        return false;
    }
    memset(m_motorControlData, 0, sizeof(m_motorControlData));
    m_motorControlData[0] = command;
    return true;
}

void MotorLKMG::convertControllerOutputToMotorControlData()
{
    if (!isConfigurationValid()) return;
    fp32 current = isfinite(m_controllerOutput) ? m_controllerOutput : 0.0f;
    GSRLMath::constrain(current, static_cast<fp32>(m_openloopLimit));
    // 零力矩可直接下发，无须先松闸。
    if (current == 0.0f) {
        const MotorIrqGuard lock;
        if (m_brakeState == BrakeState::Engaging) {
            setBrake(true);
            return;
        }
        memset(m_motorControlData, 0, sizeof(m_motorControlData));
        m_motorControlData[0] = 0xA1;
    } else if (prepareMotionFrame(0xA1)) {
        writeLE16(m_motorControlData + 4, static_cast<uint16_t>(static_cast<int16_t>(current)));
    }
}

bool MotorLKMG::hardwareAngularVelocityClosedloopControl()
{
    return hardwareAngularVelocityClosedloopControl(m_targetAngularVelocity, m_speedTorqueLimit);
}

bool MotorLKMG::hardwareAngularVelocityClosedloopControl(fp32 targetAngularVelocity)
{
    return hardwareAngularVelocityClosedloopControl(targetAngularVelocity, m_speedTorqueLimit);
}

bool MotorLKMG::hardwareAngularVelocityClosedloopControl(fp32 targetAngularVelocity, int16_t torqueLimit)
{
    if (!isConfigurationValid() || torqueLimit < -m_openloopLimit || torqueLimit > m_openloopLimit) return false;
    int32_t speed;
    if (!protocolInteger(double(targetAngularVelocity) * m_gearboxRatio * degreesPerRadian * 100.0, speed)) return false;
    m_targetAngularVelocity = targetAngularVelocity;
    m_speedTorqueLimit = torqueLimit;
    if (prepareMotionFrame(0xA2)) {
        writeLE16(m_motorControlData + 2, static_cast<uint16_t>(torqueLimit));
        writeLE32(m_motorControlData + 4, static_cast<uint32_t>(speed));
    }
    return true;
}

bool MotorLKMG::hardwareAngleClosedloopControl()
{
    return hardwareAngleClosedloopControl(m_targetAngle, m_maxVelocity, m_isMotorClockwise);
}

bool MotorLKMG::hardwareAngleClosedloopControl(fp32 angle, fp32 maxVelocity, bool clockwise)
{
    return setPositionCommand(angle, maxVelocity, true, clockwise);
}

bool MotorLKMG::hardwareRevolutionsClosedloopControl()
{
    return hardwareRevolutionsClosedloopControl(m_targetAngle, m_maxVelocity);
}

bool MotorLKMG::hardwareRevolutionsClosedloopControl(fp32 angle, fp32 maxVelocity)
{
    return setPositionCommand(angle, maxVelocity, false, m_isMotorClockwise);
}

bool MotorLKMG::setPositionCommand(fp32 angle, fp32 maxVelocity, bool singleTurn, bool clockwise)
{
    if (!isConfigurationValid() || (singleTurn && (angle < 0.0f || angle >= 2.0f * MATH_PI))) return false;
    uint16_t speedLimit;
    if (!protocolInteger(double(maxVelocity) * m_gearboxRatio * degreesPerRadian, speedLimit)) return false;
    const double centidegrees = double(angle) * m_gearboxRatio * degreesPerRadian * 100.0;
    uint32_t position;
    if (singleTurn) {
        if (!protocolInteger(centidegrees, position)) return false;
    } else {
        int32_t signedPosition;
        if (!protocolInteger(centidegrees, signedPosition)) return false;
        position = static_cast<uint32_t>(signedPosition);
    }

    m_targetAngle = angle;
    m_maxVelocity = maxVelocity;
    m_isMotorClockwise = clockwise;
    if (prepareMotionFrame(singleTurn ? 0xA6 : 0xA4)) {
        m_motorControlData[1] = singleTurn && !clockwise ? 1u : 0u;
        writeLE16(m_motorControlData + 2, speedLimit);
        writeLE32(m_motorControlData + 4, position);
    }
    return true;
}

bool MotorLKMG::setBrake(bool isBraked)
{
    if (!isConfigurationValid()) return false;
    const MotorIrqGuard lock;
    if (isBraked) m_brakeState = BrakeState::Engaging;
    else if (m_brakeState == BrakeState::Engaging) m_brakeState = BrakeState::Engaged;
    memset(m_motorControlData, 0, sizeof(m_motorControlData));
    m_motorControlData[0] = 0x8C;
    m_motorControlData[1] = isBraked ? 0u : 1u;
    return true;
}

bool MotorLKMG::requestMotionFeedback()
{
    if (!isConfigurationValid()) return false;
    memset(m_motorControlData, 0, sizeof(m_motorControlData));
    m_motorControlData[0] = 0x9C;
    return true;
}

/******************************************************************************
 *                   瓴控MG广播固件电机类实现（V2.35控制，V2.36反馈）
 ******************************************************************************/
/**
 * @brief 初始化瓴控MG广播电机参数
 * @param motorID 电机ID，范围1~4
 * @param controller 绑定的控制器
 * @param encoderOffset 编码器零位偏移量，单位为编码器计数
 * @param encoderBits 编码器位数，范围14~16，单圈分辨率为2的encoderBits次方
 * @param gearboxRatio 电机减速比，须为有限正数；速度下发时乘此值，反馈时除以此值
 * @note 初始命令为力矩控制；收到相应回复前，反馈状态与电机、错误状态均未确认。
 */
MotorLKMGmulti::MotorLKMGmulti(uint8_t motorID, Controller *controller, uint16_t encoderOffset,
                               uint8_t encoderBits, fp32 gearboxRatio, uint32_t feedbackTimeoutMs)
    : MotorLKMGBase(0x280u, motorID, 4, controller, encoderOffset, encoderBits, gearboxRatio, feedbackTimeoutMs),
      m_errorState(0), m_motorState(0xFF), m_hasValidMotorStatus(false), m_hasValidErrorStatus(false) {}

/**
 * @brief 将控制值写入本机对应的广播槽位，其余槽位清零
 * @param command 广播控制类型，同时决定控制CAN ID
 * @param value 已换算为协议单位的控制值，非有限值按0处理
 * @param lower 协议控制值下限
 * @param upper 协议控制值上限
 * @note 配置无效时仅更新命令ID并清空数据，不写入控制值。
 */
void MotorLKMGmulti::setControlValue(Command command, fp32 value, fp32 lower, fp32 upper)
{
    memset(m_motorControlData, 0, sizeof(m_motorControlData));
    m_motorControlMessageID = static_cast<uint16_t>(command);
    m_motorControlHeader.StdId = m_motorControlMessageID;
    if (!isConfigurationValid()) return;
    if (!isfinite(value)) value = 0;
    if (value < lower) value = lower;
    if (value > upper) value = upper;
    // 按协议单位限幅并四舍五入，再编码为16位补码，以小端字节序写入本机槽位。
    const uint16_t encoded = static_cast<uint16_t>(static_cast<int16_t>(roundf(value)));
    const uint8_t offset = (m_lkMotorID - 1) * 2;
    m_motorControlData[offset] = static_cast<uint8_t>(encoded);
    m_motorControlData[offset + 1] = static_cast<uint8_t>(encoded >> 8);
}

/**
 * @brief 将控制器输出编码为广播力矩控制数据
 * @note 直接使用协议原始控制量，限幅至-2000~2000，不在此换算为安培或牛米。
 */
void MotorLKMGmulti::convertControllerOutputToMotorControlData()
{
    setControlValue(Command::Torque, m_controllerOutput, -2000.0f, 2000.0f);
}

/**
 * @brief 设置广播速度闭环目标
 * @param radiansPerSecond 输出轴目标角速度，单位rad/s
 * @return true 目标已编码；false 配置无效、输入非有限或超出16位有符号速度范围
 * @note 协议单位为1度每秒，量化前四舍五入；失败时保留原控制数据。
 */
bool MotorLKMGmulti::hardwareAngularVelocityClosedloopControl(fp32 radiansPerSecond)
{
    if (!isConfigurationValid() || !isfinite(radiansPerSecond)) return false;
    // 先以输出轴rad/s校验范围，保留协议可表示的端点；再乘减速比换算为电机轴度每秒。
    const fp32 minimum = -32768.0f * (MATH_PI / 180.0f) / m_gearboxRatio;
    const fp32 maximum = 32767.0f * (MATH_PI / 180.0f) / m_gearboxRatio;
    if (radiansPerSecond < minimum || radiansPerSecond > maximum) return false;
    const fp32 degreesPerSecond = radiansPerSecond * m_gearboxRatio * (180.0f / MATH_PI);
    if (!isfinite(degreesPerSecond)) return false;
    m_targetAngularVelocity = radiansPerSecond;
    setControlValue(Command::Speed, degreesPerSecond,
                    -32768.0f, 32767.0f);
    return true;
}

/**
 * @brief 设置广播位置闭环目标
 * @param radians 电机轴绝对目标角度，单位rad，对应-327.68~327.67度
 * @return true 目标已编码；false 配置无效、输入非有限或超出协议范围
 * @note 协议单位为0.01度，量化前四舍五入；失败时保留原控制数据。
 */
bool MotorLKMGmulti::hardwareAngleClosedloopControl(fp32 radians)
{
    if (!isConfigurationValid() || !isfinite(radians)) return false;
    // 位置沿用编码器所在电机轴的角度定义，不换算减速比；rad乘18000/PI得到0.01度。
    const fp32 minimum = -32768.0f * (MATH_PI / 18000.0f);
    const fp32 maximum = 32767.0f * (MATH_PI / 18000.0f);
    if (radians < minimum || radians > maximum) return false;
    const fp32 centidegrees = radians * (18000.0f / MATH_PI);
    if (!isfinite(centidegrees)) return false;
    m_targetAngle = radians;
    setControlValue(Command::Position, centidegrees, -32768.0f, 32767.0f);
    return true;
}

/**
 * @brief 设置本机的广播管理命令
 * @param command 状态查询、清错、关闭、使能或停机命令
 * @return true 命令已编码；false 命令不支持或配置无效，保留原控制数据
 * @note 命令写入本机槽位低字节，高字节为0；清错与使能仍须在线才能组帧。
 */
bool MotorLKMGmulti::setManagementCommand(Management command)
{
    switch (command) {
        case Management::ReadStatus1: case Management::ClearError: case Management::ReadStatus2:
        case Management::Disable: case Management::Enable: case Management::Stop: break;
        default: return false;
    }
    if (!isConfigurationValid()) return false;
    setControlValue(Command::Management, static_cast<uint8_t>(command), 0, 255);
    return true;
}

bool MotorLKMGmulti::composeControlFrame(MotorLKMGmulti *const *motors, uint8_t count,
                                        CAN_TxHeaderTypeDef &header, uint8_t (&data)[8])
{
    return composeControlFrame(motors, count, header, data, 0, nullptr);
}

bool MotorLKMGmulti::composeControlFrame(MotorLKMGmulti *const *motors, uint8_t count,
                                        CAN_TxHeaderTypeDef &header, uint8_t (&data)[8],
                                        uint8_t expectedMotorMask, ComposeResult *result)
{
    // 失败原因依次覆盖：参数非法、配置无效、ID重复、命令非法、控制类型混用、集合不符、反馈不可用。
    // 所有检查通过后才写入输出报文头和数据，调用方只能在返回true时发送。
    const auto fail = [result](ComposeResult reason) {
        if (result) *result = reason;
        return false;
    };
    if (!motors || count == 0 || count > 4 || (expectedMotorMask & 0xF0))
        return fail(ComposeResult::InvalidArgument);
    uint8_t mask = 0;
    uint32_t id = 0;
    for (uint8_t i = 0; i < count; ++i) {
        if (!motors[i]) return fail(ComposeResult::InvalidArgument);
        if (!motors[i]->isConfigurationValid()) return fail(ComposeResult::InvalidConfiguration);
        const uint8_t bit = static_cast<uint8_t>(1u << (motors[i]->m_lkMotorID - 1));
        if (mask & bit) return fail(ComposeResult::DuplicateMotorID);
        mask |= bit;
        const uint32_t current = motors[i]->getMotorControlMessageID();
        if (current != 0x280 && current != 0x281 && current != 0x282 && current != 0x288)
            return fail(ComposeResult::InvalidCommand);
        if (i == 0) id = current;
        if (current != id) return fail(ComposeResult::MixedCommands);
        // 管理帧允许各电机使用不同管理命令，但每个槽位的高字节须为0，低字节须为支持的命令。
        if (current == 0x288) {
            const uint8_t offset = (motors[i]->m_lkMotorID - 1) * 2;
            const uint8_t command = motors[i]->m_motorControlData[offset];
            if (motors[i]->m_motorControlData[offset + 1] != 0 ||
                (command != 0x9A && command != 0x9B && command != 0x9C &&
                 command != 0x80 && command != 0x88 && command != 0x81))
                return fail(ComposeResult::InvalidCommand);
        }
    }
    if (expectedMotorMask != 0 && mask != expectedMotorMask)
        return fail(ComposeResult::IncompleteMotorSet);

    uint8_t merged[8] = {};
    for (uint8_t i = 0; i < count; ++i) {
        const uint8_t *own = motors[i]->m_motorControlData;
        const uint8_t offset = (motors[i]->m_lkMotorID - 1) * 2;
        // 查询、关闭和停止允许在断线时发送；运动控制另外要求近期有运动采样。
        const bool recoveryCommand = id == 0x288 &&
            (own[offset] == 0x9A || own[offset] == 0x9C ||
             own[offset] == 0x80 || own[offset] == 0x81);
        if (!recoveryCommand) {
            const bool ready = id == 0x288 ? motors[i]->isMotorConected() : motors[i]->hasFreshMotionFeedback();
            if (!ready) return fail(ComposeResult::FeedbackUnavailable);
        }
        merged[offset] = own[offset];
        merged[offset + 1] = own[offset + 1];
    }

    CAN_TxHeaderTypeDef composedHeader{};
    composedHeader.StdId = id;
    composedHeader.IDE = CAN_ID_STD;
    composedHeader.RTR = CAN_RTR_DATA;
    composedHeader.DLC = 8;
    header = composedHeader;
    memcpy(data, merged, sizeof(merged));
    if (result) *result = ComposeResult::Success;
    return true;
}

bool MotorLKMGmulti::decodeCanRxMessage(const can_rx_message_t &message)
{
    const uint8_t *d = message.data;
    switch (d[0]) {
        case 0x9C: case 0xA1: case 0xA2: case 0xA3:
            return decodeMotionFeedback(message.data);
        case 0x9A: case 0x9B:
            m_temperature = signedTemperature(d[1]);
            m_motorState = d[6];
            m_errorState = d[7];
            m_hasValidMotorStatus = true;
            m_hasValidErrorStatus = true;
            return true;
        case 0x80: case 0x88: case 0x81:
            // V2.36 将管理命令原样回显，D[1..7] 为保留的零字节。
            for (uint8_t i = 1; i < 8; ++i) {
                if (d[i] != 0) return false;
            }
            if (d[0] != 0x81) {
                m_motorState = d[0] == 0x80 ? 0x10 : 0x00;
                m_hasValidMotorStatus = true;
            }
            return true;
        default:
            return false;
    }
}
