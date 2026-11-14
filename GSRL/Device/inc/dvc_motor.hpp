/* 版权所有（c）2026 GMaster。保留所有权利。 */

#pragma once

#include "gsrl_common.h"
#include "alg_pid.hpp"
#include "drv_can.h"

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
    fp32 m_currentAngle;           // 弧度，[0,2π)
    fp32 m_lastAngle;              // 弧度，[0,2π)
    fp32 m_currentAngularVelocity; // 弧度/秒
    int16_t m_roundCount;          // 整圈数，用于计算小数圈数
    fp32 m_zeroAngle;              // 零位角度，用于计算小数圈数
    fp32 m_currentRevolutions;     // 小数圈数 n*2pi(rad)
    int16_t m_currentTorqueCurrent;
    int8_t m_temperature; // ℃
    uint8_t m_motorFeedbackErrorCount;
    bool m_isMotorConnected;
    // 目标状态
    fp32 m_targetAngle;           // 弧度，[0,2π)
    fp32 m_targetAngularVelocity; // 弧度/秒
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
 */
class MotorGM6020 : public Motor
{
protected:
    uint8_t m_djiMotorID;         // 大疆电机ID，6020电机对应1~7
    uint16_t m_encoderHistory[2]; // 0:当前值 1:上一次值
    int16_t m_currentRPMSpeed;    // 转/分钟
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

/* 瓴控MG广播：V2.35控制、V2.36反馈；ID1～4，两字节小端槽位。驱动不负责发送或持久化配置。 */
class MotorLKMGmulti : public Motor
{
public:
    // 【实机修订】原默认0x180保持兼容；现场ID1回复0x141时显式选择Id140。
    enum class FeedbackIdBase : uint16_t { Id140 = 0x140, Id180 = 0x180 };
    enum class Command : uint16_t { Torque = 0x280, Speed = 0x281, Position = 0x282, Management = 0x288 };
    enum class Management : uint8_t {
        ReadStatus1 = 0x9A, ClearError = 0x9B, ReadStatus2 = 0x9C,
        Disable = 0x80, Enable = 0x88, Stop = 0x81
    };
    MotorLKMGmulti(uint8_t motorID, Controller *controller, uint16_t encoderOffset = 0,
                  uint8_t encoderBits = 16, fp32 gearboxRatio = 1.0f, uint32_t feedbackTimeoutMs = 100,
                  FeedbackIdBase feedbackIdBase = FeedbackIdBase::Id180);
    enum class FeedbackState : uint8_t { NeverReceived, Online, Offline };
    enum class ComposeResult : uint8_t {
        Success, InvalidArgument, InvalidConfiguration, DuplicateMotorID,
        MixedCommands, InvalidCommand, IncompleteMotorSet, FeedbackUnavailable
    };
    bool isConfigurationValid() const;
    FeedbackState getFeedbackState() const;
    // 【PR复现反馈门控】状态回复不能替代运动反馈；缓存读取不刷新100ms有效期。
    bool hasFreshMotionFeedback() const;
    const uint8_t *getMotorControlData() { return m_motorControlData; }
    // 有效标志只表示曾收到对应回复，不代表数据仍新鲜。
    bool hasValidMotorStatus() const { return m_hasValidMotorStatus; }
    bool hasValidErrorStatus() const { return m_hasValidErrorStatus; }

    bool isMotorConected() const { return getFeedbackState() == FeedbackState::Online; }
    uint8_t getMotorID() const { return m_lkMotorID; }
    uint8_t getErrorState() const { return m_errorState; }
    uint8_t getMotorState() const { return m_motorState; }
    // 【PR复现电流路径】驱动器内部电流闭环，协议原始值[-2000,2000]，不是A或Nm。
    // 不使用软件PID或Controller；失败保留原命令，调用方必须跳过发送。
    bool hardwareTorqueCurrentClosedloopControl(int16_t rawCurrent);
    bool hardwareAngularVelocityClosedloopControl(fp32 radiansPerSecond);
    // 电机轴绝对位置，范围−327.68～327.67度，不做单圈回绕。
    bool hardwareAngleClosedloopControl(fp32 radians);
    bool setManagementCommand(Management command);
    // 列表须属于同一物理总线；重复电机ID或混用控制类型会被拒绝。
    // 必须列出全部已安装电机；软件不能核实物理总线归属。
    // 仅合帧成功时才能发送；失败会保留原输出，禁止误发旧帧。

    // 未列出的槽位为零，遗漏位置控制电机会向其下发零位置目标。
    static bool composeControlFrame(MotorLKMGmulti *const *motors, uint8_t count,
                                    CAN_TxHeaderTypeDef &header, uint8_t (&data)[8]);
    // 安装掩码位0～3对应ID1～4；传0沿用旧接口，不核验安装集合。
    // 【修订恢复路径】零力矩、查询、Stop/Disable可离线合帧；非零运动必须有新鲜运动反馈。
    // 零力矩不保证制动；合帧成功、硬件TXOK均不等于电机已执行，应用仍须检查回复和超时。
    static bool composeControlFrame(MotorLKMGmulti *const *motors, uint8_t count,
                                    CAN_TxHeaderTypeDef &header, uint8_t (&data)[8],
                                    uint8_t expectedMotorMask, ComposeResult *result = nullptr);
protected:
    bool decodeCanRxMessage(const can_rx_message_t &message) override;
    void convertControllerOutputToMotorControlData() override;
private:
    uint8_t m_lkMotorID;
    uint8_t m_encoderBits;
    fp32 m_gearboxRatio;
    uint8_t m_errorState;
    uint8_t m_motorState; // 初值0xFF表示状态未确认
    bool m_hasReceivedFeedback;
    bool m_hasValidMotorStatus;
    bool m_hasValidErrorStatus;
    uint32_t m_feedbackTimeoutMs;
    uint32_t m_lastFeedbackMs = 0;
    uint32_t m_lastMotionMs = 0;
    bool m_hasMotionFeedback = false;
    void setControlValue(Command command, fp32 value, fp32 lower, fp32 upper);
};
