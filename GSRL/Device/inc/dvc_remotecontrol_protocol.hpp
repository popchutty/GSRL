/**
 ******************************************************************************
 * @file           : dvc_remotecontrol_protocol.hpp
 * @brief          : 遥控器协议定义
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

#pragma once

#include <cstdint>

/******************************************************************************
 *                           遥控器协议基类实现
 ******************************************************************************/

class RemoteControlProtocol
{
public:
    virtual ~RemoteControlProtocol() = default;
};

/******************************************************************************
 *                           SUBS协议基类实现
 ******************************************************************************/

class SBUSRemoteControlProtocol : public RemoteControlProtocol
{
    public:
    struct RawPacket {
        uint8_t startByte;
        uint8_t data[22];
        uint8_t stopByte;
    } __attribute__((packed));
};

/******************************************************************************
 *                           DBUS协议基类实现
 ******************************************************************************/

class DBUSRemoteControlProtocol : public RemoteControlProtocol
{
    public:
    struct RawPacket {
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
};


class Dr16RemoteControlProtocol : public DBUSRemoteControlProtocol
{
public:
    struct Data {
        uint16_t ch[4];
        uint8_t s1;
        uint8_t s2;
        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        uint8_t mouse_l;
        uint8_t mouse_r;
        uint16_t key;
        uint16_t wheel;
    };

    void parse(const std::uint8_t *buffer, Data &out) const;
};


class ET08ARemoteControlProtocol : public SBUSRemoteControlProtocol
{
public:


    struct Data {
        uint16_t rightStickX;
        uint16_t rightStickY;
        uint16_t leftStickX;
        uint16_t leftStickY;

        uint16_t switchSA;
        uint16_t switchSB;
        uint16_t switchSC;
        uint16_t switchSD;

        uint16_t knobLD;
        uint16_t knobRD;

        uint16_t trimmerT1;
        uint16_t trimmerT2;
        uint16_t trimmerT3;
        uint16_t trimmerT4;
    };
    enum ChannelIndex : std::uint8_t {
        CH_1 = 0,
        CH_2,
        CH_3,
        CH_4,
        CH_5,
        CH_6,
        CH_7,
        CH_8,
        CH_NONE = 0xFF
    };

    struct Config {
        ChannelIndex rightStickJ1X;
        ChannelIndex rightStickJ2Y;
        ChannelIndex leftStickJ3Y;
        ChannelIndex leftStickJ4X;

        ChannelIndex switchSA;
        ChannelIndex switchSB;
        ChannelIndex switchSC;
        ChannelIndex switchSD;

        ChannelIndex knobLD;
        ChannelIndex knobRD;

        ChannelIndex trimmerT1;
        ChannelIndex trimmerT2;
        ChannelIndex trimmerT3;
        ChannelIndex trimmerT4;

        Config();
    };

    explicit ET08ARemoteControlProtocol(Config config = Config());

    void parse(const std::uint8_t *buffer, Data &out) const;

    const Config &config() const;
    void setConfig(const Config &config);

private:
    Config m_config;
};
