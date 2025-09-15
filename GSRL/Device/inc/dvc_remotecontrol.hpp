/**
 ******************************************************************************
 * @file           : dvc_remotecontrol.hpp
 * @brief          : header file for dvc_remotecontrol.cpp
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
#include <math.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 大疆DR16遥控器类，用于解码DR16遥控器接收数据
 * @note 使用前需确保receiveDr16RxDataFromISR方法在对应UART接收中断服务函数中被调用
 */
class Dr16RemoteControl
{
public:
    // DR16遥控器原始数据结构体
    struct DR16OriginalUARTRxData {
        uint64_t Channel_0 : 11;
        uint64_t Channel_1 : 11;
        uint64_t Channel_2 : 11;
        uint64_t Channel_3 : 11;
        uint64_t Switch_2 : 2;
        uint64_t Switch_1 : 2;
        int16_t Mouse_X;
        int16_t Mouse_Y;
        int16_t Mouse_Z;
        uint64_t Mouse_Left_Key : 8;
        uint64_t Mouse_Right_Key : 8;
        uint64_t Keyboard_Key : 16;
        uint64_t Channel_4 : 11;
    } __attribute__((packed));

    // DR16遥控器拨杆状态
    enum SwitchStatus : int8_t {
        SWITCH_UP = 1,
        SWITCH_DOWN,
        SWITCH_MIDDLE,
        SWITCH_ERROR = -1 // 检查m_originalRxDataPointer是否为空
    };

    // DR16遥控器拨杆跳变事件
    enum SwitchEvent : int8_t {
        SWITCH_NO_CHANGE,
        SWITCH_TOGGLE_UP_MIDDLE,
        SWITCH_TOGGLE_MIDDLE_UP,
        SWITCH_TOGGLE_DOWN_MIDDLE,
        SWITCH_TOGGLE_MIDDLE_DOWN,
        SWITCH_EVENT_NO_UPDATE_ERROR = -1 // 检查Dr16RemoteControl::updateEvent()函数是否提前调用
    };

    // DR16遥控器按键状态
    enum KeyStatus : int8_t {
        KEY_RELEASE = 0,
        KEY_PRESS,
        KEY_ERROR = -1 // 检查m_originalRxDataPointer是否为空
    };

    // DR16遥控器按键跳变事件
    enum KeyEvent : int8_t {
        KEY_NO_CHANGE,
        KEY_TOGGLE_PRESS_RELEASE,
        KEY_TOGGLE_RELEASE_PRESS,
        KEY_EVENT_NO_UPDATE_ERROR = -1 // 检查Dr16RemoteControl::updateEvent()函数是否提前调用
    };

    // DR16遥控器键盘按键对应索引
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
    DR16OriginalUARTRxData *m_originalRxDataPointer; // DR16遥控器原始接收数据指针

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

    // 遥控器连接状态检测
    uint32_t m_uartRxTimestamp; // 使用毫秒级HAL_GetTick()获取, 判断遥控器连接状态
    bool m_isDr16Connected;

    bool m_isDecodeCompleted; // 解码完成标志
    fp32 m_stickDeadZone;    // 遥控器摇杆死区

public:
    Dr16RemoteControl(fp32 stickDeadZone = 0.0f);

    void receiveRxDataFromISR(const uint8_t *data);
    void decodeRxData();
    void updateEvent();

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
    bool isDr16RemoteControlConnected()
    {
        decodeRxData();
        return m_isDr16Connected;
    }

private:
    /**
     * @brief 根据当前状态和上一次状态返回拨杆的跳变事件
     * @note 本函数为内部函数, 请勿在外部调用
     */
    SwitchEvent judgeSwitchStatus(SwitchStatus currentStatus, SwitchStatus lastStatus)
    {
        switch (currentStatus - lastStatus) {
            case 0:
                return SWITCH_NO_CHANGE;
            case -2:
                return SWITCH_TOGGLE_MIDDLE_UP;
            case -1:
                return SWITCH_TOGGLE_MIDDLE_DOWN;
            case 1:
                return SWITCH_TOGGLE_DOWN_MIDDLE;
            case 2:
                return SWITCH_TOGGLE_UP_MIDDLE;
            default:
                return SWITCH_NO_CHANGE;
        }
    }

    /**
     * @brief 根据当前状态和上一次状态返回按键的跳变事件
     * @note 本函数为内部函数, 请勿在外部调用
     */
    KeyEvent judgeKeyStatus(KeyStatus currentStatus, KeyStatus lastStatus)
    {
        switch (currentStatus - lastStatus) {
            case 0:
                return KEY_NO_CHANGE;
            case -1:
                return KEY_TOGGLE_PRESS_RELEASE;
            case 1:
                return KEY_TOGGLE_RELEASE_PRESS;
            default:
                return KEY_NO_CHANGE;
        }
    }

    fp32 applyStickDeadZone(fp32 stickValue)
    {
        if (fabs(stickValue) < m_stickDeadZone) {
            return 0.0f;
        } else if (stickValue > 0.0f) {
            return (stickValue - m_stickDeadZone) / (1.0f - m_stickDeadZone);
        } else {
            return (stickValue + m_stickDeadZone) / (1.0f - m_stickDeadZone);
        }
    }
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
