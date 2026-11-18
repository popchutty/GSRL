/**
 ******************************************************************************
 * @file           : dvc_motor.hpp
 * @brief          : header file for dvc_motor.cpp
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"
#include "alg_pid.hpp"
#include "drv_can.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机基类
 * @note 该类不可实例化
 * @details 包含电机通用的通信、状态、控制器相关变量和方法
 * @details 该类的派生类需实现以下方法：
 * - convertControllerOutputToMotorControlData 将控制器输出转换为电机控制CAN数据
 * - decodeCanRxMessage 从CAN接收到的数据中解析出电机反馈数据并检查电机是否在线
 */
class Motor
{
protected:
    // 通信
    uint32_t m_motorControlMessageID;         // 电机CAN控制消息ID
    uint32_t m_motorFeedbackMessageID;        // 电机CAN反馈消息ID
    CAN_TxHeaderTypeDef m_motorControlHeader{}; // 电机CAN控制消息头
    uint8_t m_motorControlData[8];            // 电机CAN控制数据
    uint8_t m_motorFeedbackData[8];           // 电机CAN反馈数据
    uint8_t m_motorFeedbackSequence;          // 电机反馈数据序号, 用于中断接收离线判断
    uint8_t m_motorLastFeedbackSequence;      // 上一次反馈数据序号
    // 当前状态
    fp32 m_currentAngle;           // rad [0, 2PI)
    fp32 m_lastAngle;              // rad [0, 2PI)
    fp32 m_currentAngularVelocity; // rad/s
    int32_t m_roundCount;          // 整圈数，用于计算小数圈数
    fp32 m_zeroAngle;              // 零位角度，用于计算小数圈数
    fp32 m_currentRevolutions;     // 圈数（1.0 表示一整圈）
    int16_t m_currentTorqueCurrent;
    int8_t m_temperature; // ℃
    uint8_t m_motorFeedbackErrorCount;
    bool m_isMotorConnected;
    // 目标状态
    fp32 m_targetAngle;           // rad [0, 2PI)
    fp32 m_targetAngularVelocity; // rad/s
    fp32 m_targetRevolutions;     // 圈数
    int16_t m_targetTorqueCurrent;
    // 控制器
    Controller *m_controller; // 符合 Controller 接口的控制器
    fp32 m_controllerOutput;
    bool m_controllerOutputPolarity;
    fp32 m_feedForward; // 一次性前馈, 任一闭环方法调用后自动清零
    uint16_t m_encoderOffset;
    
public:
    virtual ~Motor() = default;
    // 通信相关
    uint32_t getMotorControlMessageID() const;  // 发给电机的控制报文 ID
    uint32_t getMotorFeedbackMessageID() const; // 电机回复的反馈报文 ID
    const CAN_TxHeaderTypeDef *getMotorControlHeader() const;
    virtual const uint8_t *getMotorControlData();
    virtual bool decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t Size);
    virtual bool decodeCanRxMessageFromISR(const can_rx_message_t *rxMessage);
    const uint8_t *getMotorFeedbackData() const;
    // 获取当前状态相关
    fp32 getCurrentAngle() const;
    fp32 getCurrentAngularVelocity() const;
    virtual void resetCurrentRevolutionsToZero();
    fp32 getCurrentRevolutions() const;
    int16_t getCurrentTorqueCurrent() const;
    int8_t getTemperature() const;
    virtual bool isMotorConected() const;
    // 设置目标状态相关
    void setTargetAngle(fp32 targetAngle);
    void setTargetAngularVelocity(fp32 targetAngularVelocity);
    void setTargetRevolutions(fp32 targetRevolutions);
    void setTargetTorqueCurrent(int16_t targetTorqueCurrent);
    // 控制器相关
    void setController(Controller *controller);
    void setControllerOutputPolarity(bool polarity);
    void setFeedForwardOnce(fp32 feedForward);
    void openloopControl(fp32 controlValue);
    fp32 angleClosedloopControl();
    fp32 angleClosedloopControl(fp32 targetAngle);
    fp32 angularVelocityClosedloopControl();
    fp32 angularVelocityClosedloopControl(fp32 targetAngularVelocity);
    fp32 revolutionsClosedloopControl();
    fp32 revolutionsClosedloopControl(fp32 targetRevolutions);
    int16_t torqueCurrentClosedloopControl();
    int16_t torqueCurrentClosedloopControl(int16_t targetTorqueCurrent);
    fp32 externalClosedloopControl(fp32 setPoint, const fp32 *feedBackData, uint8_t feedBackSize);

protected:
    struct MotionFeedback {
        fp32 angle;                    // 编码器轴 rad
        fp32 angularVelocity;          // 速度环反馈，rad/s
        fp32 positionAngularVelocity;  // 与角度和圈数同轴的 rad/s，用于串级位置环
        fp32 revolutions;              // 圈
        int16_t torqueCurrent;         // 协议原始值
        bool revolutionsValid;
    };
    virtual bool readMotionFeedback(MotionFeedback &feedback) const;
    fp32 runController(fp32 target, const fp32 *feedback, uint8_t size);
    fp32 zeroControllerOutput();
    Motor(uint32_t canControlID, uint32_t canFeedbackID, Controller *controller, uint16_t encoderOffset = 0);
    virtual bool decodeCanRxMessage(const can_rx_message_t &rxMessage) = 0;
    virtual void convertControllerOutputToMotorControlData()           = 0;
    fp32 updateCurrentRevolutions();
    inline void increaseMotorFeedbackErrorCount();
    inline void clearMotorFeedbackErrorCount();
    inline void applyControllerOutputPostProcess();
};

/**
 * @brief GM6020电机类
 * @details 该类实现了Motor类的纯虚函数，用于控制大疆6020电机
 */
class MotorGM6020 : public Motor
{
protected:
    uint8_t m_djiMotorID;         // 大疆电机ID，6020电机对应1~7
    uint16_t m_encoderHistory[2]; // 0:当前值 1:上一次值
    int16_t m_currentRPMSpeed;    // RPM
    uint8_t m_mergedData[8];      // getMergedControlData 输出缓冲区

public:
    MotorGM6020(uint8_t dji6020MotorID, Controller *controller, uint16_t encoderOffset = 0);
    uint8_t getDjiMotorID() const;
    const uint8_t *getMergedControlData(MotorGM6020 &otherMotor);

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    void convertControllerOutputToMotorControlData() override;
};

/**
 * @brief M3508电机类
 * @details 在GM6020电机类的基础上适配
 */
class MotorM3508 : public MotorGM6020
{
protected:
    fp32 m_gearboxRatio; // 减速比倒数，用于换算电机转速、角速度，电机角度、圈数不受此值影响，默认为1即不换算

public:
    MotorM3508(uint8_t dji3508MotorID, Controller *controller, uint16_t encoderOffset = 0, fp32 gearboxRatio = 1.0f);

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
};

/**
 * @brief M2006电机类型别名
 * @note 复用M3508的CAN通信协议和控制方法
 */
using MotorM2006 = MotorM3508;

/**
 * @brief 达妙DMJ4310电机类
 * @note 请使用达妙电机默认固件，本类基于MIT模式单力矩输出控制帧，其余计算(如PID)在stm32上实现
 * @note 使用前先用达妙电机调试助手查看电机驱动板的PMAX、VMAX、TMAX参数，确保控制板与构造函数中的参数一致，否则会导致数据解包错误
 * @note 请将PMAX值设置位PI(3.1415926), 请将PMAX值设置位PI(3.141593), 重要！！！
 * @note 建议合理设置电机CANTimeout, 以避免电机掉线不受控, 本类会在电机重连后自动清除错误状态并使能电机
 * @details 该类实现了Motor类的纯虚函数，用于控制达妙4310电机
 */
class MotorDM4310 : public Motor
{
public:
    enum DMMotorState : uint8_t {
        DISABLE                 = 0x00,
        ENABLE                  = 0x01,
        OVER_VOLTAGE            = 0x08,
        UNDER_VOLTAGE           = 0x09,
        OVER_CURRENT            = 0x0A,
        MOSFET_OVER_TEMPERATURE = 0x0B,
        COLI_OVER_TEMPERATURE   = 0x0C,
        COMMUNICATION_LOST      = 0x0D,
        OVERLOAD                = 0x0E
    };

protected:
    DMMotorState m_motorState;
    fp32 m_dmEncoderPosition;
    fp32 m_mosfetTemperature;
    fp32 m_coliTemperature;
    bool m_setZeroPositionFlag;
    const fp32 PMAX, VMAX, TMAX;

public:
    MotorDM4310(uint8_t dmControlID, uint8_t dmMasterID, fp32 pmax, fp32 vmax, fp32 tmax, Controller *controller);
    void setMotorZeroPosition();

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    void convertControllerOutputToMotorControlData() override;
};

/**
 * @brief 达妙DM-S2325-1EC电机类型别名
 * @note 复用DMJ4310的MIT控制协议
 * @note 使用前先用达妙电机调试助手查看电机驱动板的PMAX、VMAX、TMAX参数，确保控制板与构造函数中的参数一致，否则会导致数据解包错误
 * @note 请将PMAX值设置位PI(3.1415926), 请将PMAX值设置位PI(3.141593), 重要！！！
 * @note 建议合理设置电机CANTimeout, 以避免电机掉线不受控, 本类会在电机重连后自动清除错误状态并使能电机
 */
using MotorDM2325 = MotorDM4310;

/**
 * @brief 达妙一控四固件电机类
 * @details 该类实现了Motor类的纯虚函数，用于控制达妙"一控四"固件下的电机
 * @details 与大疆GM6020的一控多模式类似，同一条控制帧上可承载4个电机的控制电流
 * @note 控制帧: 电机ID 1~4 -> 0x3FE, 电机ID 5~8 -> 0x4FE, 数据段为小端字节序
 * @note 反馈帧: 反馈ID = 0x300 + 电机ID, 数据段为大端字节序
 * @note 控制电流为标幺值，含义参见达妙力位混控模式中的 i_des 说明
 */
class MotorDMmulti : public Motor
{
protected:
    uint8_t m_dmMotorID;          // 达妙电机ID [1,8]
    uint16_t m_encoderHistory[2]; // 0:当前值 1:上一次值
    int16_t m_currentRPMSpeed;    // 电机速度, 单位RPM(已除以100后的真实值)
    uint8_t m_errorState;         // 反馈帧D[7]错误状态字节, 具体含义详见达妙错误状态说明书
    uint8_t m_mergedData[8];      // getMergedControlData 输出缓冲区

public:
    MotorDMmulti(uint8_t dmMotorID, Controller *controller, uint16_t encoderOffset = 0);
    uint8_t getDmMotorID() const;
    uint8_t getErrorState() const;
    const uint8_t *getMergedControlData(MotorDMmulti &otherMotor);

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    void convertControllerOutputToMotorControlData() override;
};

/**
 * @brief LK CAN V2.36 反馈与圈数累计。ID 和速度比例由具体电机配置决定。
 * @note 每台电机选用中断或队列接收之一；队列按接收顺序及时交付，超时从解码时刻开始计。
 *       首个运动帧为累计零点，相邻运动帧间转角须小于半圈。采样中断时保留末次圈数，
 *       并标记为无效，直到调用 resetCurrentRevolutionsToZero() 重新建立零点。
 */
class MotorLKMGBase : public Motor
{
public:
    using Motor::MotionFeedback;
    // 按驱动器实际回复配置选择；不同时接受两个ID，避免误收其他节点的报文。
    // Id180 保留原驱动默认行为；现场MG8016E-i6 V2.36实测应显式选Id140。
    enum class FeedbackIdBase : uint16_t { Id140 = 0x140, Id180 = 0x180 };
    enum class FeedbackState : uint8_t { NeverReceived, Online, Offline };
    FeedbackState getFeedbackState() const;
    bool isConfigurationValid() const;
    bool isMotorConected() const override;
    bool hasFreshMotionFeedback() const;

    // 在同一个接收快照内读取运动量。过期时返回 false，保留调用方原值。
    bool readMotionFeedback(MotionFeedback &feedback) const override;
    void resetCurrentRevolutionsToZero() override;
    uint8_t getMotorID() const { return m_lkMotorID; }
    const uint8_t *getMotorControlData() override { return m_motorControlData; }
    bool decodeCanRxMessageFromQueue(const can_rx_message_t *messages, uint8_t size) override;
    bool decodeCanRxMessageFromISR(const can_rx_message_t *message) override;

protected:
    MotorLKMGBase(uint32_t controlID, uint8_t motorID, uint8_t maxMotorID, Controller *controller,
                  uint16_t encoderOffset, uint8_t encoderBits, fp32 gearboxRatio, uint32_t feedbackTimeoutMs,
                  FeedbackIdBase feedbackIdBase = FeedbackIdBase::Id180);
    bool decodeMotionFeedback(const uint8_t (&data)[8]);
    const uint8_t m_lkMotorID;
    const fp32 m_gearboxRatio; // 电机轴速度 / 输出轴速度

private:
    bool acceptFeedback(const can_rx_message_t &message);
    enum class TurnTracking : uint8_t { AwaitingFirst, Tracking, Lost };
    const uint8_t m_maxMotorID;
    const uint8_t m_encoderBits;
    const uint32_t m_feedbackTimeoutMs;
    uint32_t m_lastFeedbackMs = 0;
    uint32_t m_lastMotionMs = 0;
    bool m_hasReceivedFeedback = false;
    bool m_hasMotionFeedback = false;
    TurnTracking m_turnTracking = TurnTracking::AwaitingFirst;
    fp32 m_motorShaftVelocity = 0.0f;
    uint16_t m_lastEncoderCount = 0;
};


/**
 * @brief 瓴控 MG 单电机 CAN V2.36。发送0x140 + ID，接收基址可配置为0x140或0x180。
 * @note 硬件位置以驱动器零点为基准，输入输出轴 rad，速度和限速为输出轴 rad/s。
 *       软件角度反馈使用编码器轴，可额外扣除 encoderOffset；该软件偏移不改变驱动器绝对零点。
 */
class MotorLKMG : public MotorLKMGBase
{
public:
    MotorLKMG(uint8_t lkMotorID, Controller *controller, uint16_t encoderOffset = 0, fp32 gearboxRatio = 1.0f,
              uint8_t encoderBits = 16, uint32_t feedbackTimeoutMs = 100,
              FeedbackIdBase feedbackIdBase = FeedbackIdBase::Id180);


    // true 表示有帧可发。等待松闸时先生成0x8C，收到松闸回复后再次调用才生成运动命令。
    // false 表示本次输入或配置无效，调用方应跳过发送。
    bool hardwareAngularVelocityClosedloopControl();
    bool hardwareAngularVelocityClosedloopControl(fp32 targetAngularVelocity);

    // D[2..3] 为协议原始转矩电流限制，范围 -2048..2048；前两个重载使用上次设置值，初值为 0。
    bool hardwareAngularVelocityClosedloopControl(fp32 targetAngularVelocity, int16_t torqueLimit);
    bool hardwareAngleClosedloopControl();
    bool hardwareAngleClosedloopControl(fp32 targetAngle, fp32 maxVelocity, bool isMotorClockwise);
    bool hardwareRevolutionsClosedloopControl();
    bool hardwareRevolutionsClosedloopControl(fp32 targetAngle, fp32 maxVelocity);
    bool setBrake(bool isBraked);
    bool requestMotionFeedback();

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    void convertControllerOutputToMotorControlData() override;

private:
    static constexpr int16_t m_openloopLimit = 2048;
    fp32 m_maxVelocity = 0.0f;
    bool m_isMotorClockwise = true;
    enum class BrakeState : uint8_t { Engaged, Released, Engaging };
    BrakeState m_brakeState = BrakeState::Engaged;
    int16_t m_speedTorqueLimit = 0;
    bool prepareMotionFrame(uint8_t command);
    bool setPositionCommand(fp32 angle, fp32 maxVelocity, bool singleTurn, bool clockwise);
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/

/**
 * @brief 瓴控MG系列广播固件电机类
 * @details 该类实现了Motor类的纯虚函数，适配V2.35广播控制协议和V2.36反馈协议
 * @note 广播CAN ID：仅支持0x280力矩控制
 * @note 电机ID为1~4，分别占用DATA[0:1]、DATA[2:3]、DATA[4:5]、DATA[6:7]，均为低字节在前
 * @note 反馈ID为feedbackIdBase + 电机ID，默认0x180；实际回复0x140+ID的设备须显式选Id140。
 *       本类仅构造和解析报文，CAN发送及驱动配置由调用方负责。
 * @note 可先下发零力矩尝试获取A1运动反馈；A1不包含故障字，不能确认无故障。
 * @note 本类仍按V2.35/V2.36的速度坐标和运动反馈布局工作，不宣称兼容V2.40。
 * @code
 * MotorLKMGmulti motor(1, nullptr, 0, 16, 6.0f, 100,
 *                     MotorLKMGmulti::FeedbackIdBase::Id140); // ID1回复0x141
 * @endcode
 */
class MotorLKMGmulti : public MotorLKMGBase
{
public:
    MotorLKMGmulti(uint8_t motorID, Controller *controller, uint16_t encoderOffset = 0,
                  uint8_t encoderBits = 16, fp32 gearboxRatio = 1.0f, uint32_t feedbackTimeoutMs = 100,
                  FeedbackIdBase feedbackIdBase = FeedbackIdBase::Id180);
    enum class ComposeResult : uint8_t {
        Success, InvalidArgument, InvalidConfiguration, DuplicateMotorID,
        InvalidCommand, IncompleteMotorSet
    };
    // 驱动器内部电流闭环目标：协议原始值[-2000,2000]，不是A或Nm；不调用Controller。
    // false时保留原命令，调用方必须跳过发送。组帧发送不依赖运动反馈。
    bool hardwareTorqueCurrentClosedloopControl(int16_t rawCurrent);
    /**
     * @brief 将同一CAN总线上各电机的控制槽位合并为广播帧
     * @param motors 电机指针数组，应列出总线上全部已安装电机，且控制CAN ID必须一致
     * @param count 电机数量，范围1~4，电机ID不可重复
     * @param header 成功时写入标准数据帧报文头
     * @param data 成功时写入8字节广播数据，未列出的电机槽位填零
     * @return true 组帧成功，可发送；false 组帧失败，header和data保持原值
     * @note 此重载不校验已安装电机集合，也无法核实电机是否属于同一总线。
     */
    static bool composeControlFrame(MotorLKMGmulti *const *motors, uint8_t count,
                                    CAN_TxHeaderTypeDef &header, uint8_t (&data)[8]);
    /**
     * @brief 合并广播控制帧，并按位掩码校验已安装电机集合
     * @param motors 同一CAN总线上的电机指针数组，要求与上述重载一致
     * @param count 电机数量，范围1~4
     * @param header 成功时写入报文头，失败时不变
     * @param data 成功时写入8字节广播数据，失败时不变
     * @param expectedMotorMask 位0~3分别表示已安装的ID 1~4；为0时跳过集合校验，高4位必须为0
     * @param result 可选的组帧结果指针，非空时写入成功或具体失败原因
     * @return true 组帧成功；false 参数、配置、命令或电机集合不满足要求
     * @note 力矩命令可直接发送，无需在线状态或反馈。
     *       零力矩并不保证制动或失能。
     *       总线发送使用合并成功的帧。getMotorControlData() 仅提供单个对象的槽位缓存。
     */
    static bool composeControlFrame(MotorLKMGmulti *const *motors, uint8_t count,
                                    CAN_TxHeaderTypeDef &header, uint8_t (&data)[8],
                                    uint8_t expectedMotorMask, ComposeResult *result = nullptr);
protected:
    bool decodeCanRxMessage(const can_rx_message_t &message) override;
    void convertControllerOutputToMotorControlData() override;
private:
    void setControlValue(fp32 value);
};
