/**
 ******************************************************************************
 * @file           : dvc_referee.hpp
 * @brief          : header file for dvc_referee.cpp
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

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"
#include "drv_uart.h"
#include <cstdint>
#include <cstring>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 裁判系统通信类，用于解析裁判系统通过UART发送的比赛数据
 * @note 使用前需确保receiveRxDataFromISR方法在对应UART接收中断服务函数中被调用
 * @details 实现了RoboMaster裁判系统协议V1.2.0的帧解包和数据解析
 */
class Referee
{
public:
    /* ======================== 协议常量定义 ======================== */
    static constexpr uint8_t FRAME_SOF             = 0xA5;
    static constexpr uint16_t FRAME_MAX_SIZE       = 512; // 适配图传链路大数据帧
    static constexpr uint16_t HEADER_SIZE          = 5;   // SOF(1) + data_length(2) + seq(1) + CRC8(1)
    static constexpr uint16_t CMD_ID_SIZE          = 2;
    static constexpr uint16_t CRC16_SIZE           = 2;
    static constexpr uint16_t HEADER_CRC8_CMD_SIZE = HEADER_SIZE + CMD_ID_SIZE;
    static constexpr uint32_t RX_TIMEOUT_MS        = 500; // 裁判系统掉线超时阈值(ms)

    /* ======================== 初始化设备枚举 ======================*/
    enum class Role : uint8_t {
        ROBOT       = 0,
        CONTROLLER  = 1,
    };

    /* ======================== 命令ID枚举 ======================== */
    enum class CmdID : uint16_t {
        // 比赛状态类
        GAME_STATE    = 0x0001, // 比赛状态 1Hz
        GAME_RESULT   = 0x0002, // 比赛结果 结束触发
        GAME_ROBOT_HP = 0x0003, // 机器人血量 3Hz
        // 场地交互类
        FIELD_EVENTS    = 0x0101, // 场地事件 1Hz
        REFEREE_WARNING = 0x0104, // 裁判警告 触发/1Hz
        DART_INFO       = 0x0105, // 飞镖相关 1Hz
        // 机器人状态类
        ROBOT_STATUS          = 0x0201, // 机器人性能体系 10Hz
        POWER_HEAT_DATA       = 0x0202, // 功率热量 10Hz
        ROBOT_POS             = 0x0203, // 机器人位置 1Hz
        BUFF                  = 0x0204, // 增益和基础能量 3Hz
        ROBOT_HURT            = 0x0206, // 伤害状态 触发
        SHOOT_DATA            = 0x0207, // 射击数据 触发
        PROJECTILE_ALLOWANCE  = 0x0208, // 允许发弹量 10Hz
        RFID_STATUS           = 0x0209, // RFID状态 3Hz
        DART_CLIENT_CMD       = 0x020A, // 飞镖闸门指令 3Hz
        GROUND_ROBOT_POSITION = 0x020B, // 地面机器人位置 1Hz
        RADAR_MARK_DATA       = 0x020C, // 雷达标记进度 1Hz
        SENTRY_INFO           = 0x020D, // 哨兵自主决策信息 1Hz
        RADAR_INFO            = 0x020E, // 雷达自主决策信息 1Hz
        // 交互数据类
        ROBOT_INTERACTION      = 0x0301, // 机器人交互数据 ≤30Hz
        CUSTOM_CONTROLLER      = 0x0302, // 自定义控制器 ≤30Hz (图传链路)
        MAP_COMMAND            = 0x0303, // 小地图下发 触发
        REMOTE_CONTROL         = 0x0306, // 键鼠遥操作 ≤30Hz
        MAP_ROBOT_DATA         = 0x0305, // 小地图接收雷达数据 ≤5Hz
        MAP_DATA               = 0x0307, // 小地图路径 ≤1Hz
        CUSTOM_INFO            = 0x0308, // 自定义消息 ≤3Hz
        CUSTOM_ROBOT_DATA      = 0x0309, // 自定义控制器接收 ≤10Hz (图传链路)
        CUSTOM_CLIENT_TO_ROBOT = 0x0310, // 自定义客户端→机器人 ≤50Hz (图传链路)
        CUSTOM_ROBOT_TO_CLIENT = 0x0311, // 自定义客户端←机器人 ≤75Hz (图传链路)
    };

    /* ======================== 机器人ID枚举 ======================== */
    enum class RobotID : uint8_t {
        RED_HERO        = 1,
        RED_ENGINEER    = 2,
        RED_STANDARD_3  = 3,
        RED_STANDARD_4  = 4,
        RED_STANDARD_5  = 5,
        RED_AERIAL      = 6,
        RED_SENTRY      = 7,
        RED_DART        = 8,
        RED_RADAR       = 9,
        RED_OUTPOST     = 10,
        RED_BASE        = 11,
        BLUE_HERO       = 101,
        BLUE_ENGINEER   = 102,
        BLUE_STANDARD_3 = 103,
        BLUE_STANDARD_4 = 104,
        BLUE_STANDARD_5 = 105,
        BLUE_AERIAL     = 106,
        BLUE_SENTRY     = 107,
        BLUE_DART       = 108,
        BLUE_RADAR      = 109,
        BLUE_OUTPOST    = 110,
        BLUE_BASE       = 111,
    };

    /* ======================== 操作手端ID枚举 ======================== */
    enum class ClientID : uint16_t {
        RED_HERO_CLIENT        = 0x0101,
        RED_ENGINEER_CLIENT    = 0x0102,
        RED_STANDARD_3_CLIENT  = 0x0103,
        RED_STANDARD_4_CLIENT  = 0x0104,
        RED_STANDARD_5_CLIENT  = 0x0105,
        RED_AERIAL_CLIENT      = 0x0106,
        BLUE_HERO_CLIENT       = 0x0165,
        BLUE_ENGINEER_CLIENT   = 0x0166,
        BLUE_STANDARD_3_CLIENT = 0x0167,
        BLUE_STANDARD_4_CLIENT = 0x0168,
        BLUE_STANDARD_5_CLIENT = 0x0169,
        BLUE_AERIAL_CLIENT     = 0x016A,
        REFEREE_SERVER         = 0x8080, // 用于哨兵和雷达自主决策指令
    };

    /* ======================== 比赛阶段枚举 ======================== */
    enum class GameProgress : uint8_t {
        NOT_STARTED  = 0,
        PREPARING    = 1,
        SELF_CHECK   = 2,
        COUNTDOWN_5S = 3,
        BATTLE       = 4,
        CALCULATING  = 5,
    };

    /* ======================== 帧解包状态机 ======================== */
    enum class UnpackStep : uint8_t {
        HEADER_SOF  = 0,
        LENGTH_LOW  = 1,
        LENGTH_HIGH = 2,
        FRAME_SEQ   = 3,
        HEADER_CRC8 = 4,
        DATA_CRC16  = 5,
    };

    /* ======================== 协议数据结构体 ======================== */
#pragma pack(push, 1)

    struct FrameHeader {
        uint8_t sof;
        uint16_t dataLength;
        uint8_t seq;
        uint8_t crc8;
    };

    // 0x0001 比赛状态 (11字节)
    struct GameStateData {
        uint8_t gameType : 4;
        uint8_t gameProgress : 4;
        uint16_t stageRemainTime; // 当前阶段剩余时间，单位秒
        uint64_t syncTimeStamp;   // UNIX时间戳，连接NTP后生效
    };

    // 0x0002 比赛结果 (1字节)
    struct GameResultData {
        uint8_t winner; // 0:平局 1:红方 2:蓝方
    };

    // 0x0003 己方机器人血量 (16字节)
    struct GameRobotHPData {
        uint16_t ally1HP; // 己方1号英雄
        uint16_t ally2HP; // 己方2号工程
        uint16_t ally3HP; // 己方3号步兵
        uint16_t ally4HP; // 己方4号步兵
        uint16_t reserved;
        uint16_t ally7HP;       // 己方7号哨兵
        uint16_t allyOutpostHP; // 己方前哨站
        uint16_t allyBaseHP;    // 己方基地
    };

    // 0x0101 场地事件 (4字节)
    // bit 0:   己方补给区占领状态
    // bit 1:   保留
    // bit 2:   己方补给区占领状态(仅RMUL)
    // bit 3-4: 己方小能量机关激活状态 (0:未激活 1:已激活 2:正在激活)
    // bit 5-6: 己方大能量机关激活状态 (0:未激活 1:已激活 2:正在激活)
    // bit 7-8: 己方中央高地占领状态 (1:被己方占领 2:被对方占领)
    // bit 9-10: 己方梯形高地占领状态 (1:已占领)
    // bit 11-19: 对方飞镖最后击中己方前哨站或基地的时间(0-420s)
    // bit 20-22: 对方飞镖最后击中目标 (0:默认 1:前哨站 2:基地固定 3:基地随机固定 4:基地随机移动 5:基地末端移动)
    // bit 23-24: 中心增益点占领状态 (0:未占领 1:己方 2:对方 3:双方)(仅RMUL)
    // bit 25-26: 己方堡垒增益点占领状态 (0:未占领 1:己方 2:对方 3:双方)
    // bit 27-28: 己方前哨站增益点占领状态 (0:未占领 1:己方 2:对方)
    // bit 29:   己方基地增益点占领状态 (1:已占领)
    // bit 30-31: 保留
    struct FieldEventsData {
        uint32_t eventData;
    };

    // 0x0104 裁判警告 (3字节)
    struct RefereeWarningData {
        uint8_t level;            // 1:双方黄牌 2:黄牌 3:红牌 4:判负
        uint8_t offendingRobotID; // 犯规机器人ID，判负和双方黄牌时为0
        uint8_t count;            // 违规次数
    };

    // 0x0105 飞镖发射口相关 (3字节)
    struct DartInfoData {
        uint8_t dartRemainingTime; // 己方飞镖发射口剩余时间（秒）
        uint16_t dartInfo;         // 飞镖信息位域
    };

    // 0x0201 机器人性能体系 (13字节)
    struct RobotStatusData {
        uint8_t robotID;
        uint8_t robotLevel;
        uint16_t currentHP;
        uint16_t maximumHP;
        uint16_t shooterBarrelCoolingValue; // 枪口每秒冷却值
        uint16_t shooterBarrelHeatLimit;    // 枪口热量上限
        uint16_t chassisPowerLimit;         // 底盘功率上限
        uint8_t gimbalOutput : 1;           // gimbal口24V输出
        uint8_t chassisOutput : 1;          // chassis口24V输出
        uint8_t shooterOutput : 1;          // shooter口24V输出
    };

    // 0x0202 实时底盘缓冲能量和枪口热量 (14字节)
    struct PowerHeatData {
        uint16_t reserved1;
        uint16_t reserved2;
        float reserved3;
        uint16_t bufferEnergy;          // 缓冲能量，单位J
        uint16_t shooter17mmBarrelHeat; // 17mm枪口热量
        uint16_t shooter42mmBarrelHeat; // 42mm枪口热量
    };

    // 0x0203 机器人位置 (12字节)
    struct RobotPosData {
        float x;     // 位置x坐标，单位m
        float y;     // 位置y坐标，单位m
        float angle; // 测速模块朝向，单位度，正北为0
    };

    // 0x0204 机器人增益和基础能量 (8字节)
    struct BuffData {
        uint8_t recoveryBuff;      // 回血增益百分比
        uint16_t coolingBuff;      // 枪口冷却增益直接值
        uint8_t defenceBuff;       // 防御增益百分比
        uint8_t vulnerabilityBuff; // 负防御增益百分比
        uint16_t attackBuff;       // 攻击增益百分比
        uint8_t remainingEnergy;   // 剩余能量反馈位域
    };

    // 0x0206 伤害状态 (1字节)
    struct RobotHurtData {
        uint8_t armorID : 4;           // 装甲板/测速模块ID
        uint8_t hpDeductionReason : 4; // 0:弹丸攻击 1:离线 5:碰撞
    };

    // 0x0207 实时射击数据 (7字节)
    struct ShootData {
        uint8_t bulletType;         // bit1:17mm bit2:42mm
        uint8_t shooterNumber;      // 1:17mm枪口 3:42mm枪口
        uint8_t launchingFrequency; // 弹频 Hz
        float initialSpeed;         // 弹速 m/s
    };

    // 0x0208 允许发弹量 (8字节)
    struct ProjectileAllowanceData {
        uint16_t projectileAllowance17mm;     // 17mm允许发弹量
        uint16_t projectileAllowance42mm;     // 42mm允许发弹量
        uint16_t remainingGoldCoin;           // 剩余金币数量
        uint16_t projectileAllowanceFortress; // 堡垒增益点额外17mm发弹量
    };

    // 0x0209 RFID模块状态 (5字节)
    struct RFIDStatusData {
        uint32_t rfidStatus; // 各增益点RFID卡检测位域
        uint8_t rfidStatus2; // 扩展RFID位域
    };

    // 0x020A 飞镖闸门操作手端指令 (6字节)
    struct DartClientCmdData {
        uint8_t dartLaunchOpeningStatus; // 0:已开启 1:关闭 2:正在开启/关闭中
        uint8_t reserved;
        uint16_t targetChangeTime;    // 切换击打目标时的比赛剩余时间（秒）
        uint16_t latestLaunchCmdTime; // 最后一次操作手确认发射时的剩余时间（秒）
    };

    // 0x020B 己方地面机器人位置 (40字节)
    struct GroundRobotPositionData {
        float heroX;
        float heroY;
        float engineerX;
        float engineerY;
        float standard3X;
        float standard3Y;
        float standard4X;
        float standard4Y;
        float reserved1;
        float reserved2;
    };

    // 0x020C 雷达标记进度 (2字节)
    struct RadarMarkData {
        uint16_t markProgress; // 各机器人标记进度位域
    };

    // 0x020D 哨兵自主决策信息同步 (6字节)
    struct SentryInfoData {
        uint32_t sentryInfo;  // 哨兵决策信息位域
        uint16_t sentryInfo2; // 扩展信息位域
    };

    // 0x020E 雷达自主决策信息同步 (1字节)
    struct RadarInfoData {
        uint8_t radarInfo; // 雷达决策信息位域
    };

    // 0x0301 机器人交互数据帧头 (6字节头 + 最大112字节内容)
    struct RobotInteractionData {
        uint16_t dataCmdID;
        uint16_t senderID;
        uint16_t receiverID;
        uint8_t userData[112];
    };

    // 0x0302 自定义控制器交互数据 （最大30字节内容）
    struct CustomControllerData {
        uint8_t data[30];
    };

    // 0x0303 小地图下发数据 (12字节)
    struct MapCommandData {
        float targetPositionX; // 目标x坐标(m)，发送目标ID时为0
        float targetPositionY; // 目标y坐标(m)，发送目标ID时为0
        uint8_t cmdKeyboard;   // 键盘按键
        uint8_t targetRobotID; // 对方机器人ID，发送坐标时为0
        uint16_t cmdSource;    // 信息来源ID
    };

    // 0x0305 小地图接收雷达数据 (48字节)
    struct MapRobotData {
        // 对方机器人坐标，单位：cm；x/y均为0时视为未发送此机器人坐标
        uint16_t opponentHeroPositionX;
        uint16_t opponentHeroPositionY;
        uint16_t opponentEngineerPositionX;
        uint16_t opponentEngineerPositionY;
        uint16_t opponentInfantry3PositionX;
        uint16_t opponentInfantry3PositionY;
        uint16_t opponentInfantry4PositionX;
        uint16_t opponentInfantry4PositionY;
        uint16_t opponentAerialPositionX;
        uint16_t opponentAerialPositionY;
        uint16_t opponentSentryPositionX;
        uint16_t opponentSentryPositionY;
        // 己方机器人坐标，单位：cm；x/y均为0时视为未发送此机器人坐标
        uint16_t allyHeroPositionX;
        uint16_t allyHeroPositionY;
        uint16_t allyEngineerPositionX;
        uint16_t allyEngineerPositionY;
        uint16_t allyInfantry3PositionX;
        uint16_t allyInfantry3PositionY;
        uint16_t allyInfantry4PositionX;
        uint16_t allyInfantry4PositionY;
        uint16_t allyAerialPositionX;
        uint16_t allyAerialPositionY;
        uint16_t allySentryPositionX;
        uint16_t allySentryPositionY;
    };

    // 0x0306 键鼠遥操作数据 (8字节)
    struct CustomClientData {
        uint16_t keyValue;
        uint16_t xPosition : 12;
        uint16_t mouseLeft : 4;
        uint16_t yPosition : 12;
        uint16_t mouseRight : 4;
        uint16_t reserved;
    };

    // 0x0308 自定义消息 (34字节)
    struct CustomInfoData {
        uint16_t senderID;
        uint16_t receiverID;
        uint8_t userData[30]; // UTF-16编码
    };

    // UI绘图相关结构体 (用于0x0301子内容)

    // 图层删除操作
    struct InteractionLayerDelete {
        uint8_t deleteType; // 0:空操作 1:删除图层 2:删除所有
        uint8_t layer;      // 图层号 0~9
    };

    // 图形数据 (15字节)
    struct InteractionFigure {
        uint8_t figureName[3];
        uint32_t operateType : 3; // 0:空 1:增加 2:修改 3:删除
        uint32_t figureType : 3;  // 0:直线 1:矩形 2:正圆 3:椭圆 4:圆弧 5:浮点数 6:整数 7:字符
        uint32_t layer : 4;       // 图层号 0~9
        uint32_t color : 4;       // 颜色
        uint32_t detailsA : 9;
        uint32_t detailsB : 9;
        uint32_t width : 10;
        uint32_t startX : 11;
        uint32_t startY : 11;
        uint32_t detailsC : 10;
        uint32_t detailsD : 11;
        uint32_t detailsE : 11;
    };

    // 哨兵自主决策指令 (子内容ID 0x0120, 4字节)
    struct SentryCmdData {
        uint32_t sentryCmd;
    };

    // 雷达自主决策指令 (子内容ID 0x0121, 8字节)
    struct RadarCmdData {
        uint8_t radarCmd;
        uint8_t passwordCmd[7]; // 密钥更新或验证指令
    };

    // 字符图形数据 (子内容ID 0x0110, 15+30字节)
    struct InteractionCharacter {
        InteractionFigure figure;
        uint8_t data[30];
    };

    // 0x0307 小地图路径数据 (103字节)
    struct MapPathData {
        uint8_t intention;       // 1:攻击 2:防守 3:移动
        uint16_t startPositionX; // 起点x坐标 (dm)
        uint16_t startPositionY; // 起点y坐标 (dm)
        int8_t deltaX[49];       // x轴增量数组
        int8_t deltaY[49];       // y轴增量数组
        uint16_t senderID;
    };

#pragma pack(pop)

    /* ======================== UI子内容ID枚举 ======================== */
    enum class UIContentID : uint16_t {
        LAYER_DELETE   = 0x0100,
        DRAW_1_FIGURE  = 0x0101,
        DRAW_2_FIGURES = 0x0102,
        DRAW_5_FIGURES = 0x0103,
        DRAW_7_FIGURES = 0x0104,
        DRAW_CHARACTER = 0x0110,
        SENTRY_CMD     = 0x0120,
        RADAR_CMD      = 0x0121,
    };

    /* ======================== UI图形颜色枚举 ======================== */
    enum class UIColor : uint8_t {
        TEAM_COLOR = 0, // 己方颜色(红/蓝)
        YELLOW     = 1,
        GREEN      = 2,
        ORANGE     = 3,
        PURPLE     = 4,
        PINK       = 5,
        CYAN       = 6,
        BLACK      = 7,
        WHITE      = 8,
    };

    /* ======================== UI图形类型枚举 ======================== */
    enum class UIFigureType : uint8_t {
        LINE      = 0,
        RECTANGLE = 1,
        CIRCLE    = 2,
        ELLIPSE   = 3,
        ARC       = 4,
        FLOAT     = 5,
        INTEGER   = 6,
        STRING    = 7,
    };

    /* ======================== UI操作类型枚举 ======================== */
    enum class UIOperateType : uint8_t {
        NONE   = 0,
        ADD    = 1,
        MODIFY = 2,
        DELETE = 3,
    };

private:
    /* ========================== 角色 ========================= */
    Role m_role;

    /* ======================== 解包状态 ======================== */
    struct UnpackContext {
        FrameHeader *header;
        uint16_t dataLength;
        uint8_t packetBuffer[FRAME_MAX_SIZE];
        UnpackStep step;
        uint16_t index;
    };

    UnpackContext m_unpackCtx;
    FrameHeader m_rxHeader;

    /* ======================== 解码后的裁判系统数据 ======================== */
    GameStateData m_gameState;
    GameResultData m_gameResult;
    GameRobotHPData m_gameRobotHP;
    FieldEventsData m_fieldEvents;
    RefereeWarningData m_refereeWarning;
    DartInfoData m_dartInfo;
    RobotStatusData m_robotStatus;
    PowerHeatData m_powerHeatData;
    RobotPosData m_robotPos;
    BuffData m_buff;
    RobotHurtData m_robotHurt;
    ShootData m_shootData;
    ProjectileAllowanceData m_projectileAllowance;
    RFIDStatusData m_rfidStatus;
    DartClientCmdData m_dartClientCmd;
    GroundRobotPositionData m_groundRobotPosition;
    RadarMarkData m_radarMark;
    SentryInfoData m_sentryInfo;
    RadarInfoData m_radarInfo;
    RobotInteractionData m_robotInteraction;
    MapCommandData m_mapCommand;
    MapRobotData m_mapRobotData;
    CustomClientData m_customClientData;
    CustomInfoData m_customInfo;
    CustomControllerData m_customControllerData;

    /* ======================== 连接状态 ======================== */
    uint32_t m_uartRxTimestamp;
    bool m_isConnected;

    /* ======================== 发送相关 ======================== */
    UART_HandleTypeDef *m_huartCommon;            // 普通链路UART句柄 (115200)
    UART_HandleTypeDef *m_huartImageTransmission; // 图传链路UART句柄 (921600)
    UART_HandleTypeDef *m_huartClient;            // 自定义控制器→选手端UART句柄 (115200)
    uint8_t m_txBuffer[FRAME_MAX_SIZE];
    uint8_t m_txSeq;

    // 内部方法
    void solveFrameData(const uint8_t *frame);
    HAL_StatusTypeDef sendFrame(UART_HandleTypeDef *huart, CmdID cmdID, const uint8_t *data, uint16_t dataLength);
    HAL_StatusTypeDef sendInteraction(uint16_t dataCmdID, uint16_t receiverID, const uint8_t *content, uint16_t contentLength);

public:
    /**
     * @brief 构造函数
     * @param huartCommon 普通链路UART句柄，默认nullptr
     * @param huartImageTransmission 图传链路UART句柄，默认nullptr
     */
    Referee(UART_HandleTypeDef *huartCommon = nullptr, UART_HandleTypeDef *huartImageTransmission = nullptr);

    /**
     * @brief 构造函数
     * @param role 自定义控制器角色，必须传 Role::CONTROLLER
     * @param huartClient 自定义控制器→选手端UART句柄
     * @note role参数同时起到重载标签的作用，避免 Referee(&huart) 被误解析成机器人
     */
    Referee(Role role, UART_HandleTypeDef *huartClient);

    /**
     * @brief 从UART接收中断中逐字节送入数据进行解包
     * @param data 接收到的数据指针
     * @param length 数据长度
     * @note 本函数应在UART接收中断回调中调用
     */
    void receiveRxDataFromISR(const uint8_t *data, uint16_t length);

    /**
     * @brief 检查裁判系统是否连接
     * @note 基于UART接收时间戳进行超时检测
     */
    bool isConnected()
    {
        if (HAL_GetTick() - m_uartRxTimestamp > RX_TIMEOUT_MS) {
            m_isConnected = false;
        }
        return m_isConnected;
    }

    /* ======================== 数据访问接口 ======================== */

    const GameStateData &getGameState() const { return m_gameState; }
    const GameResultData &getGameResult() const { return m_gameResult; }
    const GameRobotHPData &getGameRobotHP() const { return m_gameRobotHP; }
    const FieldEventsData &getFieldEvents() const { return m_fieldEvents; }
    const RefereeWarningData &getRefereeWarning() const { return m_refereeWarning; }
    const DartInfoData &getDartInfo() const { return m_dartInfo; }
    const RobotStatusData &getRobotStatus() const { return m_robotStatus; }
    const PowerHeatData &getPowerHeatData() const { return m_powerHeatData; }
    const RobotPosData &getRobotPos() const { return m_robotPos; }
    const BuffData &getBuff() const { return m_buff; }
    const RobotHurtData &getRobotHurt() const { return m_robotHurt; }
    const ShootData &getShootData() const { return m_shootData; }
    const ProjectileAllowanceData &getProjectileAllowance() const { return m_projectileAllowance; }
    const RFIDStatusData &getRFIDStatus() const { return m_rfidStatus; }
    const DartClientCmdData &getDartClientCmd() const { return m_dartClientCmd; }
    const GroundRobotPositionData &getGroundRobotPosition() const { return m_groundRobotPosition; }
    const RadarMarkData &getRadarMark() const { return m_radarMark; }
    const SentryInfoData &getSentryInfo() const { return m_sentryInfo; }
    const RadarInfoData &getRadarInfo() const { return m_radarInfo; }
    const RobotInteractionData &getRobotInteraction() const { return m_robotInteraction; }
    const MapCommandData &getMapCommand() const { return m_mapCommand; }
    const MapRobotData &getMapRobotData() const { return m_mapRobotData; }
    //const CustomClientData &getCustomClientData() const { return m_customClientData; }
    const CustomInfoData &getCustomInfo() const { return m_customInfo; }
    const CustomControllerData &getCustomControllerData() const { return m_customControllerData; }
    /* ======================== 常用便捷接口 ======================== */

    /**
     * @brief 获取本机器人ID
     */
    uint8_t getRobotID() const { return m_robotStatus.robotID; }

    /**
     * @brief 获取缓冲能量
     * @return 缓冲能量，单位J
     */
    uint16_t getBufferEnergy() const { return m_powerHeatData.bufferEnergy; }

    /**
     * @brief 获取底盘功率上限
     */
    uint16_t getChassisPowerLimit() const { return m_robotStatus.chassisPowerLimit; }

    /**
     * @brief 获取枪口热量上限和当前17mm热量
     */
    void getShooterHeatLimitAndHeat17mm(uint16_t &limit, uint16_t &heat) const
    {
        limit = m_robotStatus.shooterBarrelHeatLimit;
        heat  = m_powerHeatData.shooter17mmBarrelHeat;
    }

    /**
     * @brief 获取当前42mm枪口热量
     */
    uint16_t getShooterHeat42mm() const { return m_powerHeatData.shooter42mmBarrelHeat; }

    /**
     * @brief 获取17mm允许发弹量
     */
    uint16_t getProjectileAllowance17mm() const { return m_projectileAllowance.projectileAllowance17mm; }

    /**
     * @brief 获取42mm允许发弹量
     */
    uint16_t getProjectileAllowance42mm() const { return m_projectileAllowance.projectileAllowance42mm; }

    // 工具方法
    static uint16_t getClientIDFromRobotID(uint8_t robotID);
    // 发送接口（普通链路）
    HAL_StatusTypeDef sendUILayerDelete(uint8_t deleteType, uint8_t layer);
    HAL_StatusTypeDef sendUIDrawFigure(const InteractionFigure &figure);
    HAL_StatusTypeDef sendUIDrawFigures2(const InteractionFigure figures[2]);
    HAL_StatusTypeDef sendUIDrawFigures5(const InteractionFigure figures[5]);
    HAL_StatusTypeDef sendUIDrawFigures7(const InteractionFigure figures[7]);
    HAL_StatusTypeDef sendUIDrawCharacter(const InteractionCharacter &character);
    HAL_StatusTypeDef sendSentryCmd(const SentryCmdData &cmd);
    HAL_StatusTypeDef sendRadarCmd(const RadarCmdData &cmd);
    HAL_StatusTypeDef sendMapPath(const MapPathData &pathData);
    HAL_StatusTypeDef sendCustomInfo(uint16_t receiverID, const uint8_t *data, uint16_t dataLength);
    HAL_StatusTypeDef sendRobotToRobotData(uint16_t dataCmdID, uint16_t receiverID,
                                           const uint8_t *data, uint16_t dataLength);
    // 发送接口（图传链路）
    HAL_StatusTypeDef sendCustomRobotToClient(const uint8_t *data, uint16_t dataLength);
    // 发送接口（自定义控制器→选手端）
    HAL_StatusTypeDef sendCustomControllerData(const CustomControllerData &data);
    HAL_StatusTypeDef sendCustomClientData(const CustomClientData &data);
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
