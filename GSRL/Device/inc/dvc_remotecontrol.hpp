/**
 ******************************************************************************
 * @file           : dvc_remotecontrol.hpp
 * @brief          : 遥控器驱动头文件 (Dr16 & ET08A)
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
#include "drv_uart.h"
#include "dvc_remotecontrol_protocol.hpp"
#include <math.h>

/******************************************************************************
 *                           遥控器基类实现
 ******************************************************************************/

class RemoteControl
{
public:
    // 拨杆状态
    enum SwitchStatus : int8_t {
        SWITCH_UP = 1,
        SWITCH_DOWN,
        SWITCH_MIDDLE,
        SWITCH_ERROR = -1 // 检查m_originalRxDataPointer是否为空
    };

    // 拨杆跳变事件
    enum SwitchEvent : int8_t {
        SWITCH_NO_CHANGE,
        SWITCH_TOGGLE_UP_MIDDLE,
        SWITCH_TOGGLE_MIDDLE_UP,
        SWITCH_TOGGLE_DOWN_MIDDLE,
        SWITCH_TOGGLE_MIDDLE_DOWN,
        SWITCH_EVENT_NO_UPDATE_ERROR = -1 // 检查RemoteControl::updateEvent()函数是否提前调用
    };

    // 按键状态
    enum KeyStatus : int8_t {
        KEY_RELEASE = 0,
        KEY_PRESS,
        KEY_ERROR = -1 // 检查m_originalRxDataPointer是否为空
    };

    // 按键跳变事件
    enum KeyEvent : int8_t {
        KEY_NO_CHANGE,
        KEY_TOGGLE_PRESS_RELEASE,
        KEY_TOGGLE_RELEASE_PRESS,
        KEY_EVENT_NO_UPDATE_ERROR = -1 //防止updateEvent()函数是否提前调用
    };

protected:
    // 连接状态检测
    uint32_t m_uartRxTimestamp;
    bool m_isConnected;
    bool m_isDecodeCompleted;
    fp32 m_stickDeadZone;

    // 辅助函数
    virtual SwitchEvent judgeSwitchStatus(SwitchStatus currentStatus, SwitchStatus lastStatus);
    virtual KeyEvent judgeKeyStatus(KeyStatus currentStatus, KeyStatus lastStatus);
    fp32 applyStickDeadZone(fp32 stickValue);

public:
    RemoteControl(fp32 stickDeadZone = 0.0f);
    virtual ~RemoteControl() = default;

    virtual void receiveRxDataFromISR(const uint8_t *data) = 0;
    virtual void decodeRxData()                            = 0;
    virtual void updateEvent()                             = 0;

    bool isConnected();
};

/**
 * @brief 大疆DR16遥控器类
 */
class Dr16RemoteControl : public RemoteControl
{
public:
    using Protocol     = Dr16RemoteControlProtocol;
    using RawPacket    = Protocol::RawPacket;
    using ProtocolData = Protocol::Data;

    enum KeyboardKeyIndex : uint8_t {
        KEY_W = 0,
        KEY_S,
        KEY_A,
        KEY_D,
        KEY_SHIFT,
        KEY_CTRL,
        KEY_Q,
        KEY_E,
        KEY_R,
        KEY_F,
        KEY_G,
        KEY_Z,
        KEY_X,
        KEY_C,
        KEY_V,
        KEY_B,
        KEY_TOTAL_NUMBER // 键盘按键枚举值总数
    };

private:
    RawPacket *m_originalRxDataPointer; // DR16遥控器原始接收数据指针
    ProtocolData m_protocolData;        // 解码后的协议数据缓存
    Protocol m_protocol;

    // DR16遥控器解码数据
    fp32 m_rightStickX;
    fp32 m_rightStickY;
    fp32 m_leftStickX;
    fp32 m_leftStickY;
    fp32 m_scrollWheel;
    SwitchStatus m_rightSwitchStatus;
    SwitchStatus m_lastRightSwitchStatus; // 上一次右拨杆状态
    SwitchEvent m_rightSwitchEvent;
    SwitchStatus m_leftSwitchStatus;
    SwitchStatus m_lastLeftSwitchStatus; // 上一次左拨杆状态
    SwitchEvent m_leftSwitchEvent;
    fp32 m_mouseXSpeed;
    fp32 m_mouseYSpeed;
    fp32 m_mouseWheelSpeed;
    KeyStatus m_mouseLeftKeyStatus;
    KeyStatus m_lastMouseLeftKeyStatus; // 上一次鼠标左键状态
    KeyEvent m_mouseLeftKeyEvent;
    KeyStatus m_mouseRightKeyStatus;
    KeyStatus m_lastMouseRightKeyStatus; // 上一次鼠标右键状态
    KeyEvent m_mouseRightKeyEvent;
    KeyStatus m_keyboardKeyStatus[KEY_TOTAL_NUMBER];
    KeyStatus m_lastKeyboardKeyStatus[KEY_TOTAL_NUMBER]; // 上一次键盘按键状态
    KeyEvent m_keyboardKeyEvent[KEY_TOTAL_NUMBER];

public:
    Dr16RemoteControl(fp32 stickDeadZone = 0.0f);
    void receiveRxDataFromISR(const uint8_t *data) override;
    void decodeRxData() override;
    void updateEvent() override;

    // 获取函数
    fp32 getRightStickX()
    {
        decodeRxData();
        return applyStickDeadZone(m_rightStickX);
    }

    fp32 getRightStickY()
    {
        decodeRxData();
        return applyStickDeadZone(m_rightStickY);
    }

    fp32 getLeftStickX()
    {
        decodeRxData();
        return applyStickDeadZone(m_leftStickX);
    }

    fp32 getLeftStickY()
    {
        decodeRxData();
        return applyStickDeadZone(m_leftStickY);
    }

    SwitchStatus getRightSwitchStatus()
    {
        if (m_originalRxDataPointer == nullptr) return SWITCH_ERROR;
        return m_rightSwitchStatus = (SwitchStatus)m_originalRxDataPointer->Switch_2;
    }

    SwitchEvent getRightSwitchEvent()
    {
        return m_rightSwitchEvent;
    }

    SwitchStatus getLeftSwitchStatus()
    {
        if (m_originalRxDataPointer == nullptr) return SWITCH_ERROR;
        return m_leftSwitchStatus = (SwitchStatus)m_originalRxDataPointer->Switch_1;
    }

    SwitchEvent getLeftSwitchEvent()
    {
        return m_leftSwitchEvent;
    }

    fp32 getMouseX()
    {
        decodeRxData();
        return m_mouseXSpeed;
    }

    fp32 getMouseY()
    {
        decodeRxData();
        return m_mouseYSpeed;
    }

    fp32 getMouseWheel()
    {
        decodeRxData();
        return m_mouseWheelSpeed;
    }

    KeyStatus getMouseLeftKeyStatus()
    {
        if (m_originalRxDataPointer == nullptr) return KEY_ERROR;
        return m_mouseLeftKeyStatus = (KeyStatus)m_originalRxDataPointer->Mouse_Left_Key;
    }

    KeyEvent getMouseLeftKeyEvent()
    {
        return m_mouseLeftKeyEvent;
    }

    KeyStatus getMouseRightKeyStatus()
    {
        if (m_originalRxDataPointer == nullptr) return KEY_ERROR;
        return m_mouseRightKeyStatus = (KeyStatus)m_originalRxDataPointer->Mouse_Right_Key;
    }

    KeyEvent getMouseRightKeyEvent()
    {
        return m_mouseRightKeyEvent;
    }

    KeyStatus getKeyboardKeyStatus(KeyboardKeyIndex keyIndex)
    {
        if (m_originalRxDataPointer == nullptr) return KEY_ERROR;
        return m_keyboardKeyStatus[keyIndex] = (KeyStatus)(m_originalRxDataPointer->Keyboard_Key >> keyIndex & 0x01);
    }

    KeyEvent getKeyboardKeyEvent(KeyboardKeyIndex keyIndex)
    {
        return m_keyboardKeyEvent[keyIndex];
    }
};

/**
 * @brief 大疆ET08A遥控器类 (W.BUS)
 */
class ET08ARemoteControl : public RemoteControl
{
public:
    using Protocol     = ET08ARemoteControlProtocol;
    using RawPacket    = Protocol::RawPacket;
    using ProtocolData = Protocol::Data;
    using Config       = Protocol::Config;
    using ChannelIndex = Protocol::ChannelIndex;

private:
    RawPacket *m_originalRxDataPointer;
    ProtocolData m_protocolData; // 解码后的协议数据缓存
    Protocol m_protocol;

    // ET08A 特有控件状态
    SwitchStatus m_switchSA, m_switchSB, m_switchSC, m_switchSD;
    SwitchStatus m_lastSwitchSA, m_lastSwitchSB, m_lastSwitchSC, m_lastSwitchSD;
    SwitchEvent m_eventSA, m_eventSB, m_eventSC, m_eventSD;

    fp32 m_knobLD, m_knobRD;
    fp32 m_trimmerT1, m_trimmerT2, m_trimmerT3, m_trimmerT4;

    // 标准遥控器数据
    fp32 m_rightStickX, m_rightStickY;
    fp32 m_leftStickX, m_leftStickY;
    SwitchStatus m_rightSwitchStatus;
    SwitchEvent m_rightSwitchEvent;
    SwitchStatus m_leftSwitchStatus;
    SwitchEvent m_leftSwitchEvent;

public:
    ET08ARemoteControl(Config config = Config(), fp32 stickDeadZone = 0.0f);
    void receiveRxDataFromISR(const uint8_t *data) override;
    void decodeRxData() override;
    void updateEvent() override;

    const Config &getConfig() const
    {
        return m_protocol.config();
    }

    void setConfig(const Config &config)
    {
        m_protocol.setConfig(config);
        m_isDecodeCompleted = false;
    }

    // ET08A 特有获取函数
    SwitchStatus getSwitchSA()
    {
        return m_switchSA;
    }

    SwitchStatus getSwitchSB()
    {
        return m_switchSB;
    }

    SwitchStatus getSwitchSC()
    {
        return m_switchSC;
    }

    SwitchStatus getSwitchSD()
    {
        return m_switchSD;
    }

    SwitchEvent getSwitchEventSA()
    {
        return m_eventSA;
    }

    SwitchEvent getSwitchEventSB()
    {
        return m_eventSB;
    }

    SwitchEvent getSwitchEventSC()
    {
        return m_eventSC;
    }

    SwitchEvent getSwitchEventSD()
    {
        return m_eventSD;
    }

    fp32 getKnobLD()
    {
        decodeRxData();
        return m_knobLD;
    }

    fp32 getKnobRD()
    {
        decodeRxData();
        return m_knobRD;
    }

    fp32 getTrimmerT1()
    {
        decodeRxData();
        return m_trimmerT1;
    }

    fp32 getTrimmerT2()
    {
        decodeRxData();
        return m_trimmerT2;
    }

    fp32 getTrimmerT3()
    {
        decodeRxData();
        return m_trimmerT3;
    }

    fp32 getTrimmerT4()
    {
        decodeRxData();
        return m_trimmerT4;
    }

    // 标准获取函数
    fp32 getRightStickX()
    {
        decodeRxData();
        return applyStickDeadZone(m_rightStickX);
    }

    fp32 getRightStickY()
    {
        decodeRxData();
        return applyStickDeadZone(m_rightStickY);
    }

    fp32 getLeftStickX()
    {
        decodeRxData();
        return applyStickDeadZone(m_leftStickX);
    }

    fp32 getLeftStickY()
    {
        decodeRxData();
        return applyStickDeadZone(m_leftStickY);
    }
};
