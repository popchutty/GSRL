/**
 ******************************************************************************
 * @file           : dvc_remotecontrol.cpp
 * @brief          : 遥控器适配
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_remotecontrol.hpp"

RemoteControl::RemoteControl(fp32 stickDeadZone)
    : m_uartRxTimestamp(0),
      m_isConnected(false),
      m_isDecodeCompleted(false),
      m_stickDeadZone(stickDeadZone)
{
}

bool RemoteControl::isConnected()
{
    decodeRxData();
    return m_isConnected;
}

RemoteControl::SwitchEvent RemoteControl::judgeSwitchStatus(SwitchStatus currentStatus, SwitchStatus lastStatus)
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

RemoteControl::KeyEvent RemoteControl::judgeKeyStatus(KeyStatus currentStatus, KeyStatus lastStatus)
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

fp32 RemoteControl::applyStickDeadZone(fp32 stickValue)
{
    if (fabs(stickValue) < m_stickDeadZone) {
        return 0.0f;
    } else if (stickValue > 0.0f) {
        return (stickValue - m_stickDeadZone) / (1.0f - m_stickDeadZone);
    } else {
        return (stickValue + m_stickDeadZone) / (1.0f - m_stickDeadZone);
    }
}

/* Dr16RemoteControl 实现 ------------------------------------------*/

Dr16RemoteControl::Dr16RemoteControl(fp32 stickDeadZone)
    : RemoteControl(stickDeadZone),
      m_originalRxDataPointer(nullptr),
      m_protocolData{},
      m_protocol(),
      m_rightStickX(0.0f),
      m_rightStickY(0.0f),
      m_leftStickX(0.0f),
      m_leftStickY(0.0f),
      m_scrollWheel(0.0f),
      m_rightSwitchStatus(SWITCH_MIDDLE),
      m_lastRightSwitchStatus(SWITCH_MIDDLE),
      m_rightSwitchEvent(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_leftSwitchStatus(SWITCH_MIDDLE),
      m_lastLeftSwitchStatus(SWITCH_MIDDLE),
      m_leftSwitchEvent(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_mouseXSpeed(0.0f),
      m_mouseYSpeed(0.0f),
      m_mouseWheelSpeed(0.0f),
      m_mouseLeftKeyStatus(KEY_RELEASE),
      m_lastMouseLeftKeyStatus(KEY_RELEASE),
      m_mouseLeftKeyEvent(KEY_EVENT_NO_UPDATE_ERROR),
      m_mouseRightKeyStatus(KEY_RELEASE),
      m_lastMouseRightKeyStatus(KEY_RELEASE),
      m_mouseRightKeyEvent(KEY_EVENT_NO_UPDATE_ERROR)
{
    // 初始化键盘按键状态数组
    for (uint8_t i = 0; i < KEY_TOTAL_NUMBER; i++) {
        m_keyboardKeyStatus[i]     = KEY_RELEASE;
        m_lastKeyboardKeyStatus[i] = KEY_RELEASE;
        m_keyboardKeyEvent[i]      = KEY_NO_CHANGE;
    }
}

/**
 * @brief 从中断中获取DR16遥控器接收数据地址，更新相关标志位
 * @param data DR16遥控器接收数据指针
 * @note 本函数不解码遥控器数据
 */
void Dr16RemoteControl::receiveRxDataFromISR(const uint8_t *data)
{
    m_originalRxDataPointer = (RawPacket *)data;
    m_uartRxTimestamp       = HAL_GetTick();
    m_isConnected           = true;
    m_isDecodeCompleted     = false;
}

/**
 * @brief 接收DR16遥控器数据后解码数据, 判断遥控器连接状态
 * @note 本函数在使用get函数获取遥控器数据时自动调用
 */
void Dr16RemoteControl::decodeRxData()
{
    // 判断遥控器连接状态，若使用的数据过时超过100ms则认为遥控器断开
    if (HAL_GetTick() - m_uartRxTimestamp > 100 || m_originalRxDataPointer == nullptr) {
        m_isConnected = false;
        return;
    }
    // 解码遥控器数据, 每次接收数据仅解码一次
    if (m_isDecodeCompleted) return;
    m_rightStickX       = (fp32)(m_originalRxDataPointer->Channel_0 - 1024) / 660.0f;
    m_rightStickY       = (fp32)(m_originalRxDataPointer->Channel_1 - 1024) / 660.0f;
    m_leftStickX        = (fp32)(m_originalRxDataPointer->Channel_2 - 1024) / 660.0f;
    m_leftStickY        = (fp32)(m_originalRxDataPointer->Channel_3 - 1024) / 660.0f;
    m_scrollWheel       = (fp32)(m_originalRxDataPointer->Channel_4 - 1024) / 660.0f;
    m_mouseXSpeed       = (fp32)(m_originalRxDataPointer->Mouse_X / 32768.0f);
    m_mouseYSpeed       = (fp32)(m_originalRxDataPointer->Mouse_Y / 32768.0f);
    m_mouseWheelSpeed   = (fp32)(m_originalRxDataPointer->Mouse_Z / 32768.0f);
    m_isDecodeCompleted = true; // 更新解码完成标志，避免重复解码
}

/**
 * @brief 更新所有按键和拨杆的跳变事件并缓存
 * @note 使用get方法获取按键状态前请先调用本函数
 * @note 建议在每个控制循环的开头调用一次，且一个控制循环内只调用一次，以保证控制循环中获取的事件状态一致
 */
void Dr16RemoteControl::updateEvent()
{
    if (m_originalRxDataPointer == nullptr) return;
    m_lastRightSwitchStatus   = m_rightSwitchStatus;
    m_rightSwitchStatus       = (SwitchStatus)m_originalRxDataPointer->Switch_2;
    m_rightSwitchEvent        = judgeSwitchStatus(m_rightSwitchStatus, m_lastRightSwitchStatus);
    m_lastLeftSwitchStatus    = m_leftSwitchStatus;
    m_leftSwitchStatus        = (SwitchStatus)m_originalRxDataPointer->Switch_1;
    m_leftSwitchEvent         = judgeSwitchStatus(m_leftSwitchStatus, m_lastLeftSwitchStatus);
    m_lastMouseLeftKeyStatus  = m_mouseLeftKeyStatus;
    m_mouseLeftKeyStatus      = (KeyStatus)m_originalRxDataPointer->Mouse_Left_Key;
    m_mouseLeftKeyEvent       = judgeKeyStatus(m_mouseLeftKeyStatus, m_lastMouseLeftKeyStatus);
    m_lastMouseRightKeyStatus = m_mouseRightKeyStatus;
    m_mouseRightKeyStatus     = (KeyStatus)m_originalRxDataPointer->Mouse_Right_Key;
    m_mouseRightKeyEvent      = judgeKeyStatus(m_mouseRightKeyStatus, m_lastMouseRightKeyStatus);
    for (uint8_t keyIndex = 0; keyIndex < KEY_TOTAL_NUMBER; keyIndex++) {
        m_lastKeyboardKeyStatus[keyIndex] = m_keyboardKeyStatus[keyIndex];
        m_keyboardKeyStatus[keyIndex]     = (KeyStatus)(m_originalRxDataPointer->Keyboard_Key >> keyIndex & 0x01);
        m_keyboardKeyEvent[keyIndex]      = judgeKeyStatus(m_keyboardKeyStatus[keyIndex], m_lastKeyboardKeyStatus[keyIndex]);
    }
}

/* ET08ARemoteControl 实现 ------------------------------------------*/

ET08ARemoteControl::ET08ARemoteControl(Config config, fp32 stickDeadZone)
    : RemoteControl(stickDeadZone),
      m_originalRxDataPointer(nullptr),
      m_protocolData{},
      m_protocol(config),
      m_switchSA(SWITCH_MIDDLE),
      m_switchSB(SWITCH_UP),
      m_switchSC(SWITCH_UP),
      m_switchSD(SWITCH_MIDDLE),
      m_lastSwitchSA(SWITCH_MIDDLE),
      m_lastSwitchSB(SWITCH_UP),
      m_lastSwitchSC(SWITCH_UP),
      m_lastSwitchSD(SWITCH_MIDDLE),
      m_eventSA(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_eventSB(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_eventSC(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_eventSD(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_knobLD(0.0f),
      m_knobRD(0.0f),
      m_trimmerT1(0.0f),
      m_trimmerT2(0.0f),
      m_trimmerT3(0.0f),
      m_trimmerT4(0.0f),
      m_rightStickX(0.0f),
      m_rightStickY(0.0f),
      m_leftStickX(0.0f),
      m_leftStickY(0.0f),
      m_rightSwitchStatus(SWITCH_MIDDLE),
      m_rightSwitchEvent(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_leftSwitchStatus(SWITCH_MIDDLE),
      m_leftSwitchEvent(SWITCH_EVENT_NO_UPDATE_ERROR)
{
}

/**
 * @brief 从中断中获取ET08A遥控器接收数据地址，更新相关标志位
 * @param data ET08A遥控器接收数据指针
 * @note 本函数不解码遥控器数据
 */

void ET08ARemoteControl::receiveRxDataFromISR(const uint8_t *data)
{
    m_originalRxDataPointer = (RawPacket *)data;
    m_uartRxTimestamp       = HAL_GetTick();
    m_isConnected           = true;
    m_isDecodeCompleted     = false;
}

/**
 * @brief 接收ET08A遥控器数据后解码数据, 判断遥控器连接状态
 * @note 本函数在使用get函数获取遥控器数据时自动调用
 */

void ET08ARemoteControl::decodeRxData()
{
    if (HAL_GetTick() - m_uartRxTimestamp > 100 || m_originalRxDataPointer == nullptr) {
        m_isConnected = false;
        return;
    }
    if (m_isDecodeCompleted) return;

    if (m_originalRxDataPointer->startByte != 0x0F) return;

    m_protocol.parse(m_originalRxDataPointer->data, m_protocolData);

    m_rightStickX = (m_protocolData.rightStickX - 1024) / 660.0f;
    m_rightStickY = (m_protocolData.rightStickY - 1024) / 660.0f;
    m_leftStickX  = (m_protocolData.leftStickX - 1024) / 660.0f;
    m_leftStickY  = (m_protocolData.leftStickY - 1024) / 660.0f;

    m_knobLD = (m_protocolData.knobLD - 1024) / 660.0f;
    m_knobRD = (m_protocolData.knobRD - 1024) / 660.0f;

    m_trimmerT1 = (m_protocolData.trimmerT1 - 1024) / 660.0f;
    m_trimmerT2 = (m_protocolData.trimmerT2 - 1024) / 660.0f;
    m_trimmerT3 = (m_protocolData.trimmerT3 - 1024) / 660.0f;
    m_trimmerT4 = (m_protocolData.trimmerT4 - 1024) / 660.0f;

    m_isDecodeCompleted = true;
}

/**
 * @brief 更新所有按键和拨杆的跳变事件并缓存
 * @note 使用get方法获取按键状态前请先调用本函数
 * @note 建议在每个控制循环的开头调用一次，且一个控制循环内只调用一次，以保证控制循环中获取的事件状态一致
 */

void ET08ARemoteControl::updateEvent()
{
    if (m_originalRxDataPointer == nullptr) return;

    decodeRxData();

    // SA和SD只有两个挡位
    // 更新开关

    m_lastSwitchSA = m_switchSA;
    if (m_protocolData.switchSA < 1200) {
        m_switchSA = SWITCH_UP;
    } else if (m_protocolData.switchSA > 1200) {
        m_switchSA = SWITCH_DOWN;
    }
    m_eventSA      = judgeSwitchStatus(m_switchSA, m_lastSwitchSA);
    m_lastSwitchSB = m_switchSB;
    if (m_protocolData.switchSB < 1024) {
        m_switchSB = SWITCH_UP;
    } else if (m_protocolData.switchSB > 1024) {
        m_switchSB = SWITCH_DOWN;
    } else {
        m_switchSB = SWITCH_MIDDLE;
    }
    m_eventSB      = judgeSwitchStatus(m_switchSB, m_lastSwitchSB);
    m_lastSwitchSC = m_switchSC;
    if (m_protocolData.switchSC < 1024) {
        m_switchSC = SWITCH_UP;
    } else if (m_protocolData.switchSC > 1024) {
        m_switchSC = SWITCH_DOWN;
    } else {
        m_switchSC = SWITCH_MIDDLE;
    }
    m_eventSC      = judgeSwitchStatus(m_switchSC, m_lastSwitchSC);
    m_lastSwitchSD = m_switchSD;
    if (m_protocolData.switchSD < 1200) {
        m_switchSD = SWITCH_UP;
    } else if (m_protocolData.switchSD > 1200) {
        m_switchSD = SWITCH_DOWN;
    }
    m_eventSD      = judgeSwitchStatus(m_switchSD, m_lastSwitchSD);
}
