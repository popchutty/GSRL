/**
 ******************************************************************************
 * @file           : dvc_remotecontrol_protocol.cpp
 * @brief          : 遥控器协议解析实现
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

#include "dvc_remotecontrol_protocol.hpp"
/*
    @brief 解析DR16遥控器协议数据(此函数从未被使用，因为使用该函数会导致多一步赋值操作，影响性能。
           但是仍然保留此函数标准化协议实现以备不时之需)
    @param buffer 原始协议数据缓冲区指针
*/
void Dr16RemoteControlProtocol::parse(const std::uint8_t *buffer, Data &out) const
{
    if (buffer == nullptr) return;

    const auto *raw = reinterpret_cast<const RawPacket *>(buffer);

    out.ch[0] = raw->Channel_0;
    out.ch[1] = raw->Channel_1;
    out.ch[2] = raw->Channel_2;
    out.ch[3] = raw->Channel_3;

    out.s1 = raw->Switch_1;
    out.s2 = raw->Switch_2;

    out.mouse_x = raw->Mouse_X;
    out.mouse_y = raw->Mouse_Y;
    out.mouse_z = raw->Mouse_Z;

    out.mouse_l = raw->Mouse_Left_Key;
    out.mouse_r = raw->Mouse_Right_Key;

    out.key   = raw->Keyboard_Key;
    out.wheel = raw->Channel_4;
}

/*
    @brief ET08ARemoteControlProtocol::Config 默认构造函数，设置默认通道映射
*/

ET08ARemoteControlProtocol::Config::Config()
    : rightStickJ1X(CH_1),
      rightStickJ2Y(CH_3),
      leftStickJ3Y(CH_2),
      leftStickJ4X(CH_4),
      switchSA(CH_5),
      switchSB(CH_NONE),
      switchSC(CH_NONE),
      switchSD(CH_6),
      knobLD(CH_7),
      knobRD(CH_8),
      trimmerT1(CH_NONE),
      trimmerT2(CH_NONE),
      trimmerT3(CH_NONE),
      trimmerT4(CH_NONE)
{
}

/*
    @brief ET08ARemoteControlProtocol 构造函数
    @param config 通道映射配置
*/

ET08ARemoteControlProtocol::ET08ARemoteControlProtocol(Config config)
    : m_config(config)
{
}

/*
    @brief 解析ET08A遥控器协议数据
    @param buffer 原始协议数据缓冲区指针
*/

void ET08ARemoteControlProtocol::parse(const std::uint8_t *buffer, Data &out) const
{
    if (buffer == nullptr) return;

    std::uint16_t temp_ch[16];

    temp_ch[0]  = ((buffer[0] | buffer[1] << 8) & 0x07FF);
    temp_ch[1]  = ((buffer[1] >> 3 | buffer[2] << 5) & 0x07FF);
    temp_ch[2]  = ((buffer[2] >> 6 | buffer[3] << 2 | buffer[4] << 10) & 0x07FF);
    temp_ch[3]  = ((buffer[4] >> 1 | buffer[5] << 7) & 0x07FF);
    temp_ch[4]  = ((buffer[5] >> 4 | buffer[6] << 4) & 0x07FF);
    temp_ch[5]  = ((buffer[6] >> 7 | buffer[7] << 1 | buffer[8] << 9) & 0x07FF);
    temp_ch[6]  = ((buffer[8] >> 2 | buffer[9] << 6) & 0x07FF);
    temp_ch[7]  = ((buffer[9] >> 5 | buffer[10] << 3) & 0x07FF);
    temp_ch[8]  = ((buffer[11] | buffer[12] << 8) & 0x07FF);
    temp_ch[9]  = ((buffer[12] >> 3 | buffer[13] << 5) & 0x07FF);
    temp_ch[10] = ((buffer[13] >> 6 | buffer[14] << 2 | buffer[15] << 10) & 0x07FF);
    temp_ch[11] = ((buffer[15] >> 1 | buffer[16] << 7) & 0x07FF);
    temp_ch[12] = ((buffer[16] >> 4 | buffer[17] << 4) & 0x07FF);
    temp_ch[13] = ((buffer[17] >> 7 | buffer[18] << 1 | buffer[19] << 9) & 0x07FF);
    temp_ch[14] = ((buffer[19] >> 2 | buffer[20] << 6) & 0x07FF);
    temp_ch[15] = ((buffer[20] >> 5 | buffer[21] << 3) & 0x07FF);

    auto readChannel = [&](ChannelIndex idx) -> std::uint16_t {
        if (idx == CH_NONE) return 1024;
        return temp_ch[static_cast<std::uint8_t>(idx)];
    };

    out.rightStickX = readChannel(m_config.rightStickJ1X);
    out.rightStickY = readChannel(m_config.rightStickJ2Y);
    out.leftStickX  = readChannel(m_config.leftStickJ4X);
    out.leftStickY  = readChannel(m_config.leftStickJ3Y);

    out.switchSA = readChannel(m_config.switchSA);
    out.switchSB = readChannel(m_config.switchSB);
    out.switchSC = readChannel(m_config.switchSC);
    out.switchSD = readChannel(m_config.switchSD);

    out.knobLD = readChannel(m_config.knobLD);
    out.knobRD = readChannel(m_config.knobRD);

    out.trimmerT1 = readChannel(m_config.trimmerT1);
    out.trimmerT2 = readChannel(m_config.trimmerT2);
    out.trimmerT3 = readChannel(m_config.trimmerT3);
    out.trimmerT4 = readChannel(m_config.trimmerT4);
}

/*
    @brief 获取当前通道映射配置
*/

const ET08ARemoteControlProtocol::Config &ET08ARemoteControlProtocol::config() const
{
    return m_config;
}

/*
    @brief 设置通道映射配置
    @param config 通道映射配置
*/

void ET08ARemoteControlProtocol::setConfig(const Config &config)
{
    m_config = config;
}
