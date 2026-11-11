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
    CAN_TxHeaderTypeDef m_motorControlHeader; // 电机CAN控制消息头
    uint8_t m_motorControlData[8];            // 电机CAN控制数据
    uint8_t m_motorFeedbackData[8];           // 电机CAN反馈数据
    uint8_t m_motorFeedbackSequence;          // 电机反馈数据序号, 用于中断接收离线判断
    uint8_t m_motorLastFeedbackSequence;      // 上一次反馈数据序号
    // 当前状态
    fp32 m_currentAngle;           // rad [0, 2PI)
    fp32 m_lastAngle;              // rad [0, 2PI)
    fp32 m_currentAngularVelocity; // rad/s
    int16_t m_roundCount;          // 整圈数，用于计算小数圈数
    fp32 m_zeroAngle;              // 零位角度，用于计算小数圈数
    fp32 m_currentRevolutions;     // 小数圈数 n*2pi(rad)
    int16_t m_currentTorqueCurrent;
    int8_t m_temperature; // ℃
    uint8_t m_motorFeedbackErrorCount;
    bool m_isMotorConnected;
    // 目标状态
    fp32 m_targetAngle;           // rad [0, 2PI)
    fp32 m_targetAngularVelocity; // rad/s
    fp32 m_targetRevolutions;     // 圈n*2pi(rad)
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
    const uint8_t *getMotorControlData();
    bool decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t Size);
    bool decodeCanRxMessageFromISR(const can_rx_message_t *rxMessage);
    const uint8_t *getMotorFeedbackData() const;
    // 获取当前状态相关
    fp32 getCurrentAngle() const;
    fp32 getCurrentAngularVelocity() const;
    void resetCurrentRevolutionsToZero();
    fp32 getCurrentRevolutions() const;
    int16_t getCurrentTorqueCurrent() const;
    int8_t getTemperature() const;
    bool isMotorConected() const;
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
 * @brief 瓴控MG系列电机类
 * @details 该类实现了Motor类的纯虚函数，用于控制瓴控MG系列电机
 */
class MotorLKMG : public Motor
{
protected:
    uint8_t m_lkMotorID;
    static constexpr uint16_t m_encoderResolution = 65535;
    static constexpr int16_t m_openloopLimit      = 2048;
    uint16_t m_encoderRaw;
    fp32 m_gearboxRatio;     // 减速比
    fp32 m_maxVelocity;      // 最大速度，单位rad/s
    bool m_isMotorClockwise; // 是否顺时针旋转
    bool m_isBraked;         // 是否刹车

public:
    MotorLKMG(uint8_t lkMotorID, Controller *controller, uint16_t encoderOffset = 0, fp32 gearboxRatio = 1.0f);
    void hardwareAngularVelocityClosedloopControl();
    void hardwareAngularVelocityClosedloopControl(fp32 targetAngularVelocity);
    void hardwareAngleClosedloopControl();
    void hardwareAngleClosedloopControl(fp32 targetAngle, fp32 maxVelocity, bool isMotorClockwise);
    void hardwareRevolutionsClosedloopControl();
    void hardwareRevolutionsClosedloopControl(fp32 targetAngle, fp32 maxVelocity);
    void setBrake(bool isBraked);
    uint8_t getMotorID() const { return m_lkMotorID; }

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    void convertControllerOutputToMotorControlData() override;
};

/**
 * @brief 瓴控广播模式(一控四)电机类
 * @details 该类实现了Motor类的纯虚函数，用于控制开启"广播模式"的瓴控MF/MG系列电机
 * @details 广播模式以帧ID区分命令类型, 一帧同时携带4个电机的控制量, 每个电机占2字节, 低字节在前
 * @note 使用前需在瓴控上位机中开启广播模式, 波特率设置为500Kbps或1Mbps, 电机ID只能为1~4
 * @note 控制帧: 0x280力矩 / 0x281速度 / 0x282位置 / 0x288混合命令, 详见LKBroadcastFrameID
 * @note 反馈帧: 数据格式与单电机模式相同, 反馈ID = 反馈基地址 + 电机ID, 详见LKFeedbackBaseID
 * @note 总线上所有在线电机都会执行广播帧中自己槽位的值, 未写入的槽位为0:
 *       力矩帧中代表0力矩, 速度帧中代表0速度, 位置帧中代表转到0°位置, 混合命令帧中代表无命令。
 *       因此同一帧内的电机必须处于同一种控制模式, 位置模式下必须为总线上所有电机设置目标
 * @note 命令最大发送频率: 500Kbps下约600Hz, 1Mbps下约1.2KHz(4个电机)
 * @note 广播模式不支持抱闸器命令, MS系列电机(开环电压范围±850)不在本类支持范围内
 * @note 控制、合帧和发送需在同一个任务中依次完成, 合帧得到的数据与getMotorControlHeader()的帧ID才能对应
 * @note 反馈数据在CAN接收中断中更新, 与本库其他电机类相同, 任务中读取到的各状态量不保证来自同一帧
 */
class MotorLKBroadcast : public Motor
{
public:
    /**
     * @brief 广播模式控制帧ID, 帧ID即命令类型
     */
    enum LKBroadcastFrameID : uint32_t {
        TORQUE_CONTROL = 0x280, // 力矩控制, 控制量范围 -2000~2000
        SPEED_CONTROL  = 0x281, // 速度控制, 1dps/LSB
        ANGLE_CONTROL  = 0x282, // 位置控制, 0.01°/LSB
        MIXED_COMMAND  = 0x288  // 混合命令, 各电机执行各自的LKBroadcastCommand
    };

    /**
     * @brief 混合命令帧(0x288)支持的电机命令
     */
    enum LKBroadcastCommand : uint8_t {
        MOTOR_OFF    = 0x80, // 电机关闭, 清除圈数及之前接收的控制指令
        MOTOR_STOP   = 0x81, // 电机停止, 不清除运行状态
        MOTOR_ON     = 0x88, // 电机开启(上电后默认状态)
        READ_STATE_1 = 0x9A, // 读取温度、母线电压、母线电流、电机状态和错误标志
        CLEAR_ERROR  = 0x9B, // 清除错误标志, 电机状态未恢复正常时无法清除
        READ_STATE_2 = 0x9C  // 读取温度、转矩电流、转速、编码器位置
    };

    /**
     * @brief 反馈帧ID基地址, 取决于电机固件的协议版本
     */
    enum LKFeedbackBaseID : uint32_t {
        FEEDBACK_BASE_0x140 = 0x140, // 回复ID = 0x140 + ID, 与本库MotorLKMG一致
        FEEDBACK_BASE_0x180 = 0x180  // 回复ID = 0x180 + ID, 协议V2.36
    };

    /**
     * @brief 编码器位数, 取决于电机型号, 决定反馈编码器值的范围
     */
    enum LKEncoderResolution : uint8_t {
        ENCODER_14BIT = 14, // 编码器值范围 0~16383
        ENCODER_15BIT = 15, // 编码器值范围 0~32767
        ENCODER_16BIT = 16  // 编码器值范围 0~65535
    };

    /**
     * @brief 错误标志位掩码, 与getErrorState()的返回值按位与使用
     */
    enum LKErrorState : uint8_t {
        ERROR_LOW_VOLTAGE             = 0x01, // 低压保护
        ERROR_HIGH_VOLTAGE            = 0x02, // 高压保护
        ERROR_DRIVER_OVER_TEMPERATURE = 0x04, // 驱动过温
        ERROR_MOTOR_OVER_TEMPERATURE  = 0x08, // 电机过温
        ERROR_OVER_CURRENT            = 0x10, // 电机过流
        ERROR_SHORT_CIRCUIT           = 0x20, // 电机短路
        ERROR_STALL                   = 0x40, // 电机堵转
        ERROR_INPUT_SIGNAL_LOST       = 0x80  // 输入信号丢失超时
    };

protected:
    static constexpr uint8_t m_maxMotorID         = 4;          // 广播模式电机ID上限
    static constexpr int16_t m_torqueLimit        = 2000;       // MF/MG电机力矩控制量限幅
    static constexpr int16_t m_int16Limit         = 32767;      // 速度、位置控制量对称限幅
    static constexpr uint32_t m_invalidFeedbackID = 0xFFFFFFFF; // 电机ID非法时的反馈ID, 不会与任何标准帧匹配
    static constexpr uint8_t m_motorStateOn       = 0x00;       // 状态1中电机开启状态字节
    static constexpr uint8_t m_motorStateOff      = 0x10;       // 状态1中电机关闭状态字节

    uint8_t m_lkMotorID;        // 瓴控电机ID [1,4]
    bool m_isMotorIDValid;      // 电机ID是否合法, 非法时不写控制数据、不解析反馈
    uint8_t m_dataOffset;       // 本电机在广播帧数据段中的槽位起始下标 0/2/4/6
    uint16_t m_encoderRaw;      // 编码器原始值
    uint16_t m_encoderMask;     // 编码器值掩码, 一圈的计数减1
    fp32 m_gearboxRatio;        // 减速比
    fp32 m_hardwareTargetAngle; // 硬件位置闭环目标, 单位rad, 以电机自身零点为基准, 与基类m_targetAngle相互独立
    fp32 m_busVoltage;          // 母线电压, 单位V, 由状态1更新
    fp32 m_busCurrent;          // 母线电流, 单位A, 由状态1更新
    uint8_t m_motorState;       // 电机状态字节, 0x00开启 0x10关闭
    uint8_t m_errorState;       // 错误标志字节, 由状态1更新
    bool m_isLastMergeComplete; // 最近一次合帧是否写入了全部参与电机
    uint8_t m_mergedData[8];    // getMergedControlData 输出缓冲区

public:
    MotorLKBroadcast(uint8_t lkMotorID, Controller *controller, uint16_t encoderOffset = 0, fp32 gearboxRatio = 1.0f, LKFeedbackBaseID feedbackBaseID = FEEDBACK_BASE_0x140, LKEncoderResolution encoderResolution = ENCODER_16BIT);
    void hardwareAngularVelocityClosedloopControl();
    void hardwareAngularVelocityClosedloopControl(fp32 targetAngularVelocity);
    void hardwareAngleClosedloopControl(fp32 targetAngle);
    void setMotorCommand(LKBroadcastCommand command);
    const uint8_t *getMergedControlData(MotorLKBroadcast &otherMotor);
    const uint8_t *getMergedControlData(MotorLKBroadcast &otherMotor1, MotorLKBroadcast &otherMotor2);
    const uint8_t *getMergedControlData(MotorLKBroadcast &otherMotor1, MotorLKBroadcast &otherMotor2, MotorLKBroadcast &otherMotor3);
    bool isLastMergeComplete() const { return m_isLastMergeComplete; }
    uint8_t getMotorID() const { return m_lkMotorID; }
    bool isMotorIDValid() const { return m_isMotorIDValid; }
    uint8_t getErrorState() const { return m_errorState; }
    fp32 getBusVoltage() const { return m_busVoltage; }
    fp32 getBusCurrent() const { return m_busCurrent; }
    bool isMotorOn() const { return m_motorState == m_motorStateOn; }

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    void convertControllerOutputToMotorControlData() override;
    void setBroadcastControlData(LKBroadcastFrameID frameID, uint8_t lowByte, uint8_t highByte);
    const uint8_t *mergeControlData(MotorLKBroadcast *const otherMotors[], uint8_t otherMotorCount);
    static int16_t convertToControlValue(fp32 value, int16_t limit);
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
