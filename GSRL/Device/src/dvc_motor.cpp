/* 版权所有（c）2026 GMaster。保留所有权利。 */

#include "dvc_motor.hpp"
#include "alg_general.hpp"
#include "std_typedef.h"
#include <math.h>

/**
 * Motor电机基类实现
 */

uint32_t Motor::getMotorControlMessageID() const
{
    return m_motorControlMessageID;
}

uint32_t Motor::getMotorFeedbackMessageID() const
{
    return m_motorFeedbackMessageID;
}

const CAN_TxHeaderTypeDef *Motor::getMotorControlHeader() const
{
    return &m_motorControlHeader;
}

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
 * @note 从后往前解析, 缓冲数组中数据应从旧到新排列
 * @note Size可小于等于缓冲数组大小，推荐设置为接收到的CAN包数量
 */
bool Motor::decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t Size)
{
    for (uint8_t i = Size; i > 0; i--) {
        if (decodeCanRxMessage(rxMessage[i - 1])) {
            ++m_motorFeedbackSequence; // 只在收到有效队列反馈后更新序号。
            clearMotorFeedbackErrorCount();
            updateCurrentRevolutions();
            return true;
        }
    }
    increaseMotorFeedbackErrorCount(); // 未解析到数据增加错误计数
    return false;
}

/**
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

const uint8_t *Motor::getMotorFeedbackData() const
{
    return m_motorFeedbackData;
}

/**
 * @return fp32 电机当前角度 单位rad [0, 2PI)
 */
fp32 Motor::getCurrentAngle() const
{
    return m_currentAngle;
}

/**
 * @return fp32 电机当前角速度 单位rad/s
 */
fp32 Motor::getCurrentAngularVelocity() const
{
    return m_currentAngularVelocity;
}

void Motor::resetCurrentRevolutionsToZero()
{
    m_currentRevolutions = 0.0f;
    m_roundCount         = 0;
    m_zeroAngle          = m_currentAngle;
    m_lastAngle          = m_currentAngle;
}

fp32 Motor::getCurrentRevolutions() const
{
    return m_currentRevolutions;
}

int16_t Motor::getCurrentTorqueCurrent() const
{
    return m_currentTorqueCurrent;
}

/**
 * @return uint8_t 电机当前温度 单位℃
 */
int8_t Motor::getTemperature() const
{
    return m_temperature;
}

bool Motor::isMotorConected() const
{
    return m_isMotorConnected;
}

/**
 * @param targetAngle 电机目标角度 单位rad [0, 2PI)
 */
void Motor::setTargetAngle(fp32 targetAngle)
{
    m_targetAngle = targetAngle;
}

/**
 * @param targetAngularVelocity 电机目标角速度 单位rad/s
 */
void Motor::setTargetAngularVelocity(fp32 targetAngularVelocity)
{
    m_targetAngularVelocity = targetAngularVelocity;
}

void Motor::setTargetRevolutions(fp32 targetRevolutions)
{
    m_targetRevolutions = targetRevolutions;
}

void Motor::setTargetTorqueCurrent(int16_t targetTorqueCurrent)
{
    m_targetTorqueCurrent = targetTorqueCurrent;
}

void Motor::setController(Controller *controller)
{
    m_controller = controller;
}

void Motor::setControllerOutputPolarity(bool polarity)
{
    m_controllerOutputPolarity = polarity;
}

void Motor::openloopControl(fp32 controlValue)
{
    m_controllerOutput = controlValue;
    convertControllerOutputToMotorControlData();
}

/**
 * @note 请在每次控制电机前调用，电机控制方法被调用后会自动清零前馈值
 * @note 只对闭环方法生效，对硬件控制无效
 * @note 前馈在极性反转后叠加
 */
void Motor::setFeedForwardOnce(fp32 feedForward)
{
    m_feedForward = feedForward;
}

/**
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
 * @param targetAngle 目标角度 单位rad [0, 2PI)
 */
fp32 Motor::angleClosedloopControl(fp32 targetAngle)
{
    m_targetAngle = targetAngle;
    return angleClosedloopControl();
}

fp32 Motor::angularVelocityClosedloopControl()
{
    fp32 feedBackData[1] = {m_currentAngularVelocity};
    m_controllerOutput   = m_controller->controllerCalculate(m_targetAngularVelocity, feedBackData, 1);
    applyControllerOutputPostProcess();
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

/**
 * @param targetAngularVelocity 目标角速度 单位rad/s
 */
fp32 Motor::angularVelocityClosedloopControl(fp32 targetAngularVelocity)
{
    m_targetAngularVelocity = targetAngularVelocity;
    return angularVelocityClosedloopControl();
}

/**
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

fp32 Motor::revolutionsClosedloopControl(fp32 targetRevolutions)
{
    m_targetRevolutions = targetRevolutions;
    return revolutionsClosedloopControl();
}

int16_t Motor::torqueCurrentClosedloopControl()
{
    fp32 feedBackData[1] = {(fp32)m_currentTorqueCurrent};
    m_controllerOutput   = m_controller->controllerCalculate((fp32)m_targetTorqueCurrent, feedBackData, 1);
    applyControllerOutputPostProcess();
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

int16_t Motor::torqueCurrentClosedloopControl(int16_t targetTorqueCurrent)
{
    m_targetTorqueCurrent = targetTorqueCurrent;
    return torqueCurrentClosedloopControl();
}

fp32 Motor::externalClosedloopControl(fp32 setPoint, const fp32 *feedBackData, uint8_t feedBackSize)
{
    m_controllerOutput = m_controller->controllerCalculate(setPoint, feedBackData, feedBackSize);
    applyControllerOutputPostProcess();
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

/**
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
 * @note 该函数仅供内部或派生类调用
 */
inline void Motor::clearMotorFeedbackErrorCount()
{
    m_motorFeedbackErrorCount = 0;
    m_isMotorConnected        = true;
}

/**
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

/**
 * GM6020电机类实现
 */

/**
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

uint8_t MotorGM6020::getDjiMotorID() const
{
    return m_djiMotorID;
}

/**
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

/**
 * M3508电机类实现
 */

/**
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

/**
 * DMJ4310电机类实现
 */

/**
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

void MotorDM4310::setMotorZeroPosition()
{
    m_setZeroPositionFlag = true;
}

/**
 * 达妙一控四固件电机类实现
 */

/**
 * @param dmMotorID 达妙电机ID, 取值范围 [1, 8]
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

    m_currentTorqueCurrent = (int16_t)((rxMessage.data[4] << 8) | rxMessage.data[5]); // 毫安
    m_temperature          = (int8_t)rxMessage.data[6];                               // ℃
    m_errorState           = rxMessage.data[7];
    return true;
}

uint8_t MotorDMmulti::getDmMotorID() const
{
    return m_dmMotorID;
}

uint8_t MotorDMmulti::getErrorState() const
{
    return m_errorState;
}

/**
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

/**
 * 瓴控MG系列电机类实现
 */

/**
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
 * @note 内部函数, 被decodeCanRxMessageFromQueue或decodeCanRxMessageFromISR调用
 * @note 目前只解析0x9C命令，其他命令可根据需要添加
 * 0x9C命令数据格式参考MG系列电机协议文档，为：温度、扭矩电流、速度、原始编码器
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
 * @param targetAngularVelocity 目标角速度，单位rad/s
 * @note 正反转由符号决定
 */
void MotorLKMG::hardwareAngularVelocityClosedloopControl(fp32 targetAngularVelocity)
{
    m_targetAngularVelocity = targetAngularVelocity;
    hardwareAngularVelocityClosedloopControl();
}

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
 * @param targetAngle 目标角度，单位rad [0, 2PI)
 * @param maxVelocity 最大允许角速度，单位rad/s
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
 * @param targetAngle 目标角度，无限制，单位rad
 * @param maxVelocity 最大允许角速度，单位rad/s
 */
void MotorLKMG::hardwareRevolutionsClosedloopControl(fp32 targetAngle, fp32 maxVelocity)
{
    m_targetAngle = targetAngle;
    m_maxVelocity = maxVelocity;
    revolutionsClosedloopControl();
}

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

/* 瓴控MG广播：V2.35控制、V2.36反馈；ID1～4，两字节小端槽位。驱动不负责发送或持久化配置。 */
namespace {
// 【接收同步】恢复进入前的PRIMASK，支持在已关闭中断时嵌套调用。
struct LKMGMultiIrqGuard {
    uint32_t saved = __get_PRIMASK();
    LKMGMultiIrqGuard() { __disable_irq(); }
    ~LKMGMultiIrqGuard() { __set_PRIMASK(saved); }
};
}

MotorLKMGmulti::MotorLKMGmulti(uint8_t motorID, Controller *controller, uint16_t encoderOffset,
                             uint8_t encoderBits, fp32 gearboxRatio, uint32_t feedbackTimeoutMs,
                             FeedbackIdBase feedbackIdBase)
    : Motor(0x280u, static_cast<uint16_t>(feedbackIdBase) + motorID, controller, encoderOffset),
      m_lkMotorID(motorID), m_encoderBits(encoderBits), m_gearboxRatio(gearboxRatio),
      m_errorState(0), m_motorState(0xFF), m_hasReceivedFeedback(false),
      m_hasValidMotorStatus(false), m_hasValidErrorStatus(false), m_feedbackTimeoutMs(feedbackTimeoutMs) {}

MotorLKMGmulti::FeedbackState MotorLKMGmulti::getFeedbackState() const
{
    const LKMGMultiIrqGuard lock;
    if (!m_hasReceivedFeedback) return FeedbackState::NeverReceived;
    return HAL_GetTick() - m_lastFeedbackMs < m_feedbackTimeoutMs ? FeedbackState::Online : FeedbackState::Offline;
}

bool MotorLKMGmulti::hasFreshMotionFeedback() const
{
    const LKMGMultiIrqGuard lock;
    return m_hasMotionFeedback && HAL_GetTick() - m_lastMotionMs < m_feedbackTimeoutMs;
}

bool MotorLKMGmulti::isConfigurationValid() const
{
    return m_lkMotorID >= 1 && m_lkMotorID <= 4 &&
           (m_motorFeedbackMessageID == 0x140u + m_lkMotorID || m_motorFeedbackMessageID == 0x180u + m_lkMotorID) &&
           m_feedbackTimeoutMs > 0 &&
           m_encoderBits >= 14 && m_encoderBits <= 16 &&
           isfinite(m_gearboxRatio) && m_gearboxRatio > 0;
}

void MotorLKMGmulti::setControlValue(Command command, fp32 value, fp32 lower, fp32 upper)
{
    const LKMGMultiIrqGuard lock;
    memset(m_motorControlData, 0, sizeof(m_motorControlData));
    m_motorControlMessageID = static_cast<uint16_t>(command);
    m_motorControlHeader.StdId = m_motorControlMessageID;
    if (!isConfigurationValid()) return;
    if (!isfinite(value)) value = 0;
    if (value < lower) value = lower;
    if (value > upper) value = upper;
    // 限幅后按协议单位四舍五入，再转换为16位补码。
    const uint16_t encoded = static_cast<uint16_t>(static_cast<int16_t>(roundf(value)));
    const uint8_t offset = (m_lkMotorID - 1) * 2;
    m_motorControlData[offset] = static_cast<uint8_t>(encoded);
    m_motorControlData[offset + 1] = static_cast<uint8_t>(encoded >> 8);
}

void MotorLKMGmulti::convertControllerOutputToMotorControlData()
{
    // 【PR #77复现点一】0x280力矩编码：每台两字节、小端、原始量限幅，不换算安培。
    setControlValue(Command::Torque, m_controllerOutput, -2000.0f, 2000.0f);
}

bool MotorLKMGmulti::hardwareTorqueCurrentClosedloopControl(int16_t rawCurrent)
{
    if (!isConfigurationValid() || rawCurrent < -2000 || rawCurrent > 2000) return false;
    m_targetTorqueCurrent = rawCurrent;
    setControlValue(Command::Torque, rawCurrent, -2000.0f, 2000.0f);
    return true;
}

bool MotorLKMGmulti::hardwareAngularVelocityClosedloopControl(fp32 radiansPerSecond)
{
    if (!isConfigurationValid() || !isfinite(radiansPerSecond)) return false;
    // 输入为输出轴弧度/秒；先校验边界，再换算电机轴速度。
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

bool MotorLKMGmulti::hardwareAngleClosedloopControl(fp32 radians)
{
    if (!isConfigurationValid() || !isfinite(radians)) return false;
    // 位置沿用编码器所在电机轴坐标，不换算减速比。
    const fp32 minimum = -32768.0f * (MATH_PI / 18000.0f);
    const fp32 maximum = 32767.0f * (MATH_PI / 18000.0f);
    if (radians < minimum || radians > maximum) return false;
    const fp32 centidegrees = radians * (18000.0f / MATH_PI);
    if (!isfinite(centidegrees)) return false;
    m_targetAngle = radians;
    setControlValue(Command::Position, centidegrees, -32768.0f, 32767.0f);
    return true;
}

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
    // 【PR #77复现点二】保留完整安装集合、重复ID、命令类型及失败不改输出的检查。
    const LKMGMultiIrqGuard lock;
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
    bool feedbackAvailable = true;
    for (uint8_t i = 0; i < count; ++i) {
        // 只读本次命令缓存，不用读取次数推测在线，也不刷新反馈时间。
        const uint8_t *own = motors[i]->m_motorControlData;
        const uint8_t offset = (motors[i]->m_lkMotorID - 1) * 2;
        const bool zeroTorque = id == 0x280 && own[offset] == 0 && own[offset + 1] == 0;
        const bool recoveryCommand = zeroTorque || (id == 0x288 &&
            (own[offset] == 0x9A || own[offset] == 0x9C ||
             own[offset] == 0x80 || own[offset] == 0x81));
        if (!recoveryCommand) {
            const bool ready = id == 0x288 ? motors[i]->isMotorConected() : motors[i]->hasFreshMotionFeedback();
            if (!ready) feedbackAvailable = false;
        }
        merged[offset] = own[offset];
        merged[offset + 1] = own[offset + 1];
    }
    if (!feedbackAvailable) return fail(ComposeResult::FeedbackUnavailable);

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
    // 【PR #77复现点三】接收真实0xA1等反馈；新增可配置基址，不同时放行两套ID。
    const LKMGMultiIrqGuard lock;
    if (!isConfigurationValid() || message.header.StdId != m_motorFeedbackMessageID ||
        message.header.IDE != CAN_ID_STD || message.header.RTR != CAN_RTR_DATA ||
        message.header.DLC != 8) return false;
    const uint8_t *d = message.data;
    const auto signedWord = [](uint8_t low, uint8_t high) {
        const int32_t raw = (uint32_t(high) << 8) | low;
        return static_cast<int16_t>(raw >= 32768 ? raw - 65536 : raw);
    };
    const int8_t temperature = static_cast<int8_t>(d[1] >= 128 ? int(d[1]) - 256 : d[1]);
    switch (d[0]) {
        case 0x9C: case 0xA1: case 0xA2: case 0xA3: {
            const uint32_t resolution = 1u << m_encoderBits;
            const uint32_t raw = (uint16_t(d[7]) << 8) | d[6];
            if (raw >= resolution) return false;
            m_temperature = temperature;
            m_currentTorqueCurrent = signedWord(d[2], d[3]);
            const int16_t speed = signedWord(d[4], d[5]);
            m_currentAngularVelocity = speed * (MATH_PI / 180.0f) / m_gearboxRatio;
            const uint32_t adjusted = (raw + resolution - (m_encoderOffset % resolution)) % resolution;
            m_currentAngle = adjusted * (2.0f * MATH_PI / resolution);
            m_lastMotionMs = HAL_GetTick();
            m_hasMotionFeedback = true;
        } break;
        case 0x9A: case 0x9B:
            m_temperature = temperature;
            m_motorState = d[6];
            m_errorState = d[7];
            m_hasValidMotorStatus = true;
            m_hasValidErrorStatus = true;
            break;
        case 0x80: case 0x88: case 0x81:
            for (uint8_t i = 1; i < 8; ++i) if (d[i] != 0) return false;
            if (d[0] != 0x81) {
                m_motorState = d[0] == 0x80 ? 0x10 : 0x00;
                m_hasValidMotorStatus = true;
            }
            break;
        default: return false;
    }
    m_hasReceivedFeedback = true;
    m_lastFeedbackMs = HAL_GetTick();
    memcpy(m_motorFeedbackData, d, sizeof(m_motorFeedbackData));
    return true;
}
