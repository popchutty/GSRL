/**
 ******************************************************************************
 * @file           : dvc_referee.cpp
 * @brief          : 裁判系统通信驱动
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 * @note 基于2026高校系列赛通信协议V1.3.0
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_referee.hpp"
#include "alg_crc.hpp"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                              Referee裁判系统类实现
 ******************************************************************************/

/**
 * @brief 构造函数，初始化所有成员变量
 * @note 本函数用于机器人的裁判系统初始化
 */
Referee::Referee(UART_HandleTypeDef *huartCommon,
                 UART_HandleTypeDef *huartImageTransmission)
    : m_role(Role::ROBOT),
      m_unpackCtx{},
      m_rxHeader{},
      m_gameState{},
      m_gameResult{},
      m_gameRobotHP{},
      m_fieldEvents{},
      m_refereeWarning{},
      m_dartInfo{},
      m_robotStatus{},
      m_powerHeatData{},
      m_robotPos{},
      m_buff{},
      m_robotHurt{},
      m_shootData{},
      m_projectileAllowance{},
      m_rfidStatus{},
      m_dartClientCmd{},
      m_groundRobotPosition{},
      m_radarMark{},
      m_sentryInfo{},
      m_radarInfo{},
      m_robotInteraction{},
      m_mapCommand{},
      m_mapRobotData{},
      m_customClientData{},
      m_customInfo{},
      m_customControllerData{},
      m_uartRxTimestamp(0),
      m_isConnected(false),
      m_huartCommon(huartCommon),
      m_huartImageTransmission(huartImageTransmission),
      m_huartClient(nullptr),
      m_txBuffer{},
      m_txSeq(0)
{
    m_unpackCtx.header = &m_rxHeader;
    m_unpackCtx.step   = UnpackStep::HEADER_SOF;
    m_unpackCtx.index  = 0;
}

/**
 * @brief 构造函数，初始化所有成员变量
 * @note 本函数用于自定义控制器的裁判系统初始化
 */
Referee::Referee(Role role, UART_HandleTypeDef *huartClient)
    : Referee(nullptr, nullptr)
{
    m_role          = role;
    m_huartClient   = huartClient;
}

/**
 * @brief 从UART接收中断中逐字节送入数据，按状态机进行帧解包
 * @param data 接收到的数据指针
 * @param length 数据长度
 * @note 本函数应在UART接收中断回调中调用
 */
void Referee::receiveRxDataFromISR(const uint8_t *data, uint16_t length)
{
    m_uartRxTimestamp = HAL_GetTick();
    m_isConnected     = true;

    for (uint16_t i = 0; i < length; i++) {
        uint8_t byte = data[i];

        switch (m_unpackCtx.step) {
            case UnpackStep::HEADER_SOF: {
                if (byte == FRAME_SOF) {
                    m_unpackCtx.index                             = 0;
                    m_unpackCtx.packetBuffer[m_unpackCtx.index++] = byte;
                    m_unpackCtx.step                              = UnpackStep::LENGTH_LOW;
                }
            } break;

            case UnpackStep::LENGTH_LOW: {
                m_unpackCtx.packetBuffer[m_unpackCtx.index++] = byte;
                m_unpackCtx.step                              = UnpackStep::LENGTH_HIGH;
            } break;

            case UnpackStep::LENGTH_HIGH: {
                m_unpackCtx.packetBuffer[m_unpackCtx.index++] = byte;
                m_unpackCtx.dataLength                        = static_cast<uint16_t>(
                    m_unpackCtx.packetBuffer[1] | (m_unpackCtx.packetBuffer[2] << 8));
                if (m_unpackCtx.dataLength >= FRAME_MAX_SIZE - HEADER_SIZE - CMD_ID_SIZE - CRC16_SIZE) {
                    m_unpackCtx.step = UnpackStep::HEADER_SOF;
                    break;
                }
                m_unpackCtx.step = UnpackStep::FRAME_SEQ;
            } break;

            case UnpackStep::FRAME_SEQ: {
                m_unpackCtx.packetBuffer[m_unpackCtx.index++] = byte;
                m_unpackCtx.step                              = UnpackStep::HEADER_CRC8;
            } break;

            case UnpackStep::HEADER_CRC8: {
                m_unpackCtx.packetBuffer[m_unpackCtx.index++] = byte;
                if (CRCCalculator::verifyCRC8(m_unpackCtx.packetBuffer, HEADER_SIZE)) {
                    m_unpackCtx.step = UnpackStep::DATA_CRC16;
                } else {
                    m_unpackCtx.step = UnpackStep::HEADER_SOF;
                }
            } break;

            case UnpackStep::DATA_CRC16: {
                if (m_unpackCtx.index < FRAME_MAX_SIZE) {
                    m_unpackCtx.packetBuffer[m_unpackCtx.index++] = byte;
                }
                // 帧总长度 = 帧头(5) + 命令ID(2) + 数据(dataLength) + CRC16(2)
                uint16_t frameLength = HEADER_SIZE + CMD_ID_SIZE + m_unpackCtx.dataLength + CRC16_SIZE;
                if (m_unpackCtx.index >= frameLength) {
                    if (CRCCalculator::verifyCRC16(m_unpackCtx.packetBuffer, frameLength)) {
                        solveFrameData(m_unpackCtx.packetBuffer);
                    }
                    m_unpackCtx.step = UnpackStep::HEADER_SOF;
                }
            } break;

            default:
                m_unpackCtx.step = UnpackStep::HEADER_SOF;
                break;
        }
    }
}

/**
 * @brief 解析已校验通过的完整帧，根据命令ID分发数据
 * @param frame 帧数据指针
 */
void Referee::solveFrameData(const uint8_t *frame)
{
    uint16_t index = 0;

    memcpy(&m_rxHeader, frame, sizeof(FrameHeader));
    index += sizeof(FrameHeader);

    uint16_t cmdID = 0;
    memcpy(&cmdID, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (static_cast<CmdID>(cmdID)) {
        case CmdID::GAME_STATE:
            memcpy(&m_gameState, frame + index, sizeof(GameStateData));
            break;
        case CmdID::GAME_RESULT:
            memcpy(&m_gameResult, frame + index, sizeof(GameResultData));
            break;
        case CmdID::GAME_ROBOT_HP:
            memcpy(&m_gameRobotHP, frame + index, sizeof(GameRobotHPData));
            break;
        case CmdID::FIELD_EVENTS:
            memcpy(&m_fieldEvents, frame + index, sizeof(FieldEventsData));
            break;
        case CmdID::REFEREE_WARNING:
            memcpy(&m_refereeWarning, frame + index, sizeof(RefereeWarningData));
            break;
        case CmdID::DART_INFO:
            memcpy(&m_dartInfo, frame + index, sizeof(DartInfoData));
            break;
        case CmdID::ROBOT_STATUS:
            memcpy(&m_robotStatus, frame + index, sizeof(RobotStatusData));
            break;
        case CmdID::POWER_HEAT_DATA:
            memcpy(&m_powerHeatData, frame + index, sizeof(PowerHeatData));
            break;
        case CmdID::ROBOT_POS:
            memcpy(&m_robotPos, frame + index, sizeof(RobotPosData));
            break;
        case CmdID::BUFF:
            memcpy(&m_buff, frame + index, sizeof(BuffData));
            break;
        case CmdID::ROBOT_HURT:
            memcpy(&m_robotHurt, frame + index, sizeof(RobotHurtData));
            break;
        case CmdID::SHOOT_DATA:
            memcpy(&m_shootData, frame + index, sizeof(ShootData));
            break;
        case CmdID::PROJECTILE_ALLOWANCE:
            memcpy(&m_projectileAllowance, frame + index, sizeof(ProjectileAllowanceData));
            break;
        case CmdID::RFID_STATUS:
            memcpy(&m_rfidStatus, frame + index, sizeof(RFIDStatusData));
            break;
        case CmdID::DART_CLIENT_CMD:
            memcpy(&m_dartClientCmd, frame + index, sizeof(DartClientCmdData));
            break;
        case CmdID::GROUND_ROBOT_POSITION:
            memcpy(&m_groundRobotPosition, frame + index, sizeof(GroundRobotPositionData));
            break;
        case CmdID::RADAR_MARK_DATA:
            memcpy(&m_radarMark, frame + index, sizeof(RadarMarkData));
            break;
        case CmdID::SENTRY_INFO:
            memcpy(&m_sentryInfo, frame + index, sizeof(SentryInfoData));
            break;
        case CmdID::RADAR_INFO:
            memcpy(&m_radarInfo, frame + index, sizeof(RadarInfoData));
            break;
        case CmdID::ROBOT_INTERACTION:
            memcpy(&m_robotInteraction, frame + index, sizeof(RobotInteractionData));
            break;
        case CmdID::MAP_COMMAND:
            memcpy(&m_mapCommand, frame + index, sizeof(MapCommandData));
            break;
        case CmdID::MAP_ROBOT_DATA:
            memcpy(&m_mapRobotData, frame + index, sizeof(MapRobotData));
            break;
        //case CmdID::REMOTE_CONTROL:
        //    memcpy(&m_customClientData, frame + index, sizeof(CustomClientData));
        //    break;
        case CmdID::CUSTOM_INFO:
            memcpy(&m_customInfo, frame + index, sizeof(CustomInfoData));
            break;
        case CmdID::CUSTOM_CONTROLLER:
            memcpy(&m_customControllerData, frame + index, sizeof(CustomControllerData));
            break;  
        default:
            break;
    }
}

/******************************************************************************
 *                              发送功能实现
 ******************************************************************************/

/**
 * @brief 根据机器人ID获取对应操作手端ID
 * @param robotID 机器人ID
 * @return uint16_t 对应操作手端ID，无效ID返回0
 * @note 红方机器人ID 1~9 对应客户端ID 0x0101~0x0109
 * @note 蓝方机器人ID 101~109 对应客户端ID 0x0165~0x016D
 */
uint16_t Referee::getClientIDFromRobotID(uint8_t robotID)
{
    if (robotID >= 1 && robotID <= 9) {
        return 0x0100 + robotID;
    }
    if (robotID >= 101 && robotID <= 109) {
        return 0x0164 + (robotID - 100);
    }
    return 0;
}

/**
 * @brief 构建完整协议帧并通过指定UART发送
 * @param huart 目标UART句柄
 * @param cmdID 命令ID
 * @param data 数据段内容指针
 * @param dataLength 数据段长度
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR参数错误或UART句柄为空
 * @note 内部函数，帧格式: SOF(1) + dataLength(2) + seq(1) + CRC8(1) + cmdID(2) + data(n) + CRC16(2)
 */
HAL_StatusTypeDef Referee::sendFrame(UART_HandleTypeDef *huart, CmdID cmdID, const uint8_t *data, uint16_t dataLength)
{
    if (huart == nullptr) {
        return HAL_ERROR;
    }

    uint16_t frameLength = HEADER_SIZE + CMD_ID_SIZE + dataLength + CRC16_SIZE;
    if (frameLength > FRAME_MAX_SIZE) {
        return HAL_ERROR;
    }

    uint16_t index = 0;

    // 帧头
    m_txBuffer[index++] = FRAME_SOF;
    m_txBuffer[index++] = dataLength & 0xFF;
    m_txBuffer[index++] = (dataLength >> 8) & 0xFF;
    m_txBuffer[index++] = m_txSeq++;
    CRCCalculator::appendCRC8(m_txBuffer, HEADER_SIZE);
    index = HEADER_SIZE;

    // 命令ID
    uint16_t cmdIDValue = static_cast<uint16_t>(cmdID);
    memcpy(m_txBuffer + index, &cmdIDValue, sizeof(uint16_t));
    index += sizeof(uint16_t);

    // 数据段
    if (data != nullptr && dataLength > 0) {
        memcpy(m_txBuffer + index, data, dataLength);
        index += dataLength;
    }

    // CRC16
    CRCCalculator::appendCRC16(m_txBuffer, frameLength);

    return UART_Send_Data(huart, m_txBuffer, frameLength);
}

/**
 * @brief 封装0x0301机器人交互数据并通过普通链路发送
 * @param dataCmdID 子内容ID
 * @param receiverID 接收者ID
 * @param content 子内容数据指针
 * @param contentLength 子内容数据长度
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR数据长度超限或发送失败
 * @note 内部函数，自动填充发送者ID为本机robotID
 * @note 数据段格式: dataCmdID(2) + senderID(2) + receiverID(2) + content(n)，最大118字节
 */
HAL_StatusTypeDef Referee::sendInteraction(uint16_t dataCmdID, uint16_t receiverID, const uint8_t *content, uint16_t contentLength)
{
    static constexpr uint16_t INTERACTION_HEADER_SIZE = 6;
    uint16_t dataLength                               = INTERACTION_HEADER_SIZE + contentLength;

    if (dataLength > 118) {
        return HAL_ERROR;
    }

    uint8_t interactionBuffer[118];
    uint16_t index = 0;

    memcpy(interactionBuffer + index, &dataCmdID, sizeof(uint16_t));
    index += sizeof(uint16_t);

    uint16_t senderID = m_robotStatus.robotID;
    memcpy(interactionBuffer + index, &senderID, sizeof(uint16_t));
    index += sizeof(uint16_t);

    memcpy(interactionBuffer + index, &receiverID, sizeof(uint16_t));
    index += sizeof(uint16_t);

    if (content != nullptr && contentLength > 0) {
        memcpy(interactionBuffer + index, content, contentLength);
    }

    return sendFrame(m_huartCommon, CmdID::ROBOT_INTERACTION, interactionBuffer, dataLength);
}

/**
 * @brief 删除UI图层
 * @param deleteType 删除类型 0:空操作 1:删除指定图层 2:删除所有图层
 * @param layer 图层号 0~9
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR发送失败
 * @note 通过普通链路发送，子内容ID 0x0100
 */
HAL_StatusTypeDef Referee::sendUILayerDelete(uint8_t deleteType, uint8_t layer)
{
    InteractionLayerDelete layerDelete;
    layerDelete.deleteType = deleteType;
    layerDelete.layer      = layer;

    uint16_t clientID = getClientIDFromRobotID(m_robotStatus.robotID);
    return sendInteraction(static_cast<uint16_t>(UIContentID::LAYER_DELETE),
                           clientID,
                           reinterpret_cast<const uint8_t *>(&layerDelete),
                           sizeof(InteractionLayerDelete));
}

/**
 * @brief 绘制1个UI图形
 * @param figure 图形数据
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR发送失败
 * @note 通过普通链路发送，子内容ID 0x0101
 */
HAL_StatusTypeDef Referee::sendUIDrawFigure(const InteractionFigure &figure)
{
    uint16_t clientID = getClientIDFromRobotID(m_robotStatus.robotID);
    return sendInteraction(static_cast<uint16_t>(UIContentID::DRAW_1_FIGURE),
                           clientID,
                           reinterpret_cast<const uint8_t *>(&figure),
                           sizeof(InteractionFigure));
}

/**
 * @brief 绘制2个UI图形
 * @param figures 图形数据数组，长度为2
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR发送失败
 * @note 通过普通链路发送，子内容ID 0x0102
 */
HAL_StatusTypeDef Referee::sendUIDrawFigures2(const InteractionFigure figures[2])
{
    uint16_t clientID = getClientIDFromRobotID(m_robotStatus.robotID);
    return sendInteraction(static_cast<uint16_t>(UIContentID::DRAW_2_FIGURES),
                           clientID,
                           reinterpret_cast<const uint8_t *>(figures),
                           sizeof(InteractionFigure) * 2);
}

/**
 * @brief 绘制5个UI图形
 * @param figures 图形数据数组，长度为5
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR发送失败
 * @note 通过普通链路发送，子内容ID 0x0103
 */
HAL_StatusTypeDef Referee::sendUIDrawFigures5(const InteractionFigure figures[5])
{
    uint16_t clientID = getClientIDFromRobotID(m_robotStatus.robotID);
    return sendInteraction(static_cast<uint16_t>(UIContentID::DRAW_5_FIGURES),
                           clientID,
                           reinterpret_cast<const uint8_t *>(figures),
                           sizeof(InteractionFigure) * 5);
}

/**
 * @brief 绘制7个UI图形
 * @param figures 图形数据数组，长度为7
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR发送失败
 * @note 通过普通链路发送，子内容ID 0x0104
 */
HAL_StatusTypeDef Referee::sendUIDrawFigures7(const InteractionFigure figures[7])
{
    uint16_t clientID = getClientIDFromRobotID(m_robotStatus.robotID);
    return sendInteraction(static_cast<uint16_t>(UIContentID::DRAW_7_FIGURES),
                           clientID,
                           reinterpret_cast<const uint8_t *>(figures),
                           sizeof(InteractionFigure) * 7);
}

/**
 * @brief 绘制UI字符图形
 * @param character 字符图形数据，包含图形属性和最多30字节字符内容
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR发送失败
 * @note 通过普通链路发送，子内容ID 0x0110
 */
HAL_StatusTypeDef Referee::sendUIDrawCharacter(const InteractionCharacter &character)
{
    uint16_t clientID = getClientIDFromRobotID(m_robotStatus.robotID);
    return sendInteraction(static_cast<uint16_t>(UIContentID::DRAW_CHARACTER),
                           clientID,
                           reinterpret_cast<const uint8_t *>(&character),
                           sizeof(InteractionCharacter));
}

/**
 * @brief 发送哨兵自主决策指令
 * @param cmd 哨兵决策指令数据
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR发送失败
 * @note 通过普通链路发送至裁判服务器(0x8080)，子内容ID 0x0120
 * @note 仅哨兵机器人可调用
 */
HAL_StatusTypeDef Referee::sendSentryCmd(const SentryCmdData &cmd)
{
    return sendInteraction(static_cast<uint16_t>(UIContentID::SENTRY_CMD),
                           static_cast<uint16_t>(ClientID::REFEREE_SERVER),
                           reinterpret_cast<const uint8_t *>(&cmd),
                           sizeof(SentryCmdData));
}

/**
 * @brief 发送雷达自主决策指令
 * @param cmd 雷达决策指令数据，包含决策指令和密钥
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR发送失败
 * @note 通过普通链路发送至裁判服务器(0x8080)，子内容ID 0x0121
 * @note 仅雷达机器人可调用
 */
HAL_StatusTypeDef Referee::sendRadarCmd(const RadarCmdData &cmd)
{
    return sendInteraction(static_cast<uint16_t>(UIContentID::RADAR_CMD),
                           static_cast<uint16_t>(ClientID::REFEREE_SERVER),
                           reinterpret_cast<const uint8_t *>(&cmd),
                           sizeof(RadarCmdData));
}

/**
 * @brief 发送机器人间交互数据
 * @param dataCmdID 子内容ID，范围0x0200~0x02FF
 * @param receiverID 接收者机器人ID
 * @param data 数据指针
 * @param dataLength 数据长度，最大112字节
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR数据超限或发送失败
 * @note 通过普通链路发送，命令ID 0x0301
 */
HAL_StatusTypeDef Referee::sendRobotToRobotData(uint16_t dataCmdID, uint16_t receiverID,
                                                const uint8_t *data, uint16_t dataLength)
{
    return sendInteraction(dataCmdID, receiverID, data, dataLength);
}

/**
 * @brief 发送小地图路径数据
 * @param pathData 路径数据，包含起点坐标和增量数组
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR发送失败
 * @note 通过普通链路发送，命令ID 0x0307，频率上限1Hz
 */
HAL_StatusTypeDef Referee::sendMapPath(const MapPathData &pathData)
{
    return sendFrame(m_huartCommon,
                     CmdID::MAP_DATA,
                     reinterpret_cast<const uint8_t *>(&pathData),
                     sizeof(MapPathData));
}

/**
 * @brief 发送自定义消息
 * @param receiverID 接收者ID
 * @param data UTF-16编码数据指针
 * @param dataLength 数据长度，最大30字节
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR数据超限或发送失败
 * @note 通过普通链路发送，命令ID 0x0308，频率上限3Hz
 */
HAL_StatusTypeDef Referee::sendCustomInfo(uint16_t receiverID, const uint8_t *data, uint16_t dataLength)
{
    if (dataLength > 30) {
        return HAL_ERROR;
    }

    CustomInfoData info{};
    info.senderID   = m_robotStatus.robotID;
    info.receiverID = receiverID;
    if (data != nullptr && dataLength > 0) {
        memcpy(info.userData, data, dataLength);
    }

    return sendFrame(m_huartCommon,
                     CmdID::CUSTOM_INFO,
                     reinterpret_cast<const uint8_t *>(&info),
                     sizeof(CustomInfoData));
}

/**
 * @brief 发送自定义机器人到客户端数据
 * @param data 数据指针
 * @param dataLength 数据长度
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR发送失败
 * @note 通过图传链路发送，命令ID 0x0311，频率上限75Hz
 */
HAL_StatusTypeDef Referee::sendCustomRobotToClient(const uint8_t *data, uint16_t dataLength)
{
    return sendFrame(m_huartImageTransmission, CmdID::CUSTOM_ROBOT_TO_CLIENT, data, dataLength);
}

/**
 * @brief 发送自定义控制器数据
 * @param data 数据指针
 * @return HAL_StatusTypeDef HAL_OK发送成功，HAL_ERROR发送失败
 * @note 从自定义控制器发送到选手端，命令ID 0x0302，帧率上线为30Hz
 */
HAL_StatusTypeDef Referee::sendCustomControllerData(const CustomControllerData &data)
{
    static_assert(sizeof(CustomControllerData) == 30, "0x0302 payload must be 30 bytes");

    if (m_role != Role::CONTROLLER) {
        return HAL_ERROR;
    }

    return sendFrame(m_huartClient,
                     CmdID::CUSTOM_CONTROLLER,
                     reinterpret_cast<const uint8_t *>(&data),
                     sizeof(data));
}

/**
 * @brief 发送自定义控制器键鼠数据到选手端
 * @param data 自定义控制器发送到选手端的键鼠数据
 * @return HAL_StatusTypeDef HAL_OK发送成功， HAL_ERROR发送失败
 * @note 从自定义控制器发送到选手端，命令ID 0x306
 */
HAL_StatusTypeDef Referee::sendCustomClientData(const CustomClientData &data)
{
    static_assert(sizeof(CustomClientData) == 8, "0x0306 payload must be 8 bytes");

    if (m_role != Role::CONTROLLER) {
        return HAL_ERROR;
    }

    CustomClientData txData = data;
    txData.reserved         = 0;

    return sendFrame(m_huartClient,
                     CmdID::REMOTE_CONTROL,
                     reinterpret_cast<const uint8_t *>(&txData),
                     sizeof(txData));
}