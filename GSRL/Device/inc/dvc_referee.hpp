/**
 ******************************************************************************
 * @file           : dvc_referee.hpp
 * @brief          : 裁判系统解析头文件
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DVC_REFEREE_HPP
#define __DVC_REFEREE_HPP

/* Includes ------------------------------------------------------------------*/
#include <cstdint>
#include <cstring>

/* Exported types ------------------------------------------------------------*/
// ================= 协议结构体定义 =================

#pragma pack(push, 1)

// 0x0001 比赛状态
struct GameStatus
{
    uint8_t  game_type : 4;
    uint8_t  game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t sync_timestamp;
};

// 0x0002 比赛结果
struct GameResult
{
    uint8_t winner;
};

// 0x0003 比赛机器人血量数据
struct GameRobotHP
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
};

// 0x0201 机器人性能体系数据
struct RobotStatus
{
    uint8_t  robot_id;
    uint8_t  robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t  power_management_gimbal_output  : 1;
    uint8_t  power_management_chassis_output : 1;
    uint8_t  power_management_shooter_output : 1;
};

// 0x0202 能量 & 热量数据
struct PowerHeatData
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float    chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
};

// 0x0203 机器人位置
struct RobotPos
{
    float x;
    float y;
    float angle; // 度，正北 = 0
};

// 0x0204 增益 & 能量反馈
struct Buff
{
    uint8_t  recovery_buff;
    uint16_t cooling_buff;
    uint8_t  defence_buff;
    uint8_t  vulnerability_buff;
    uint16_t attack_buff;
    uint8_t  remaining_energy;
};

// 0x0206 受伤信息
struct HurtData
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
};

// 0x0207 射击信息
struct ShootData
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float   initial_speed;
};

// 0x0208 允许发弹量
struct ProjectileAllowance
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
};

// 0x0209 RFID 状态
struct RfidStatus
{
    uint32_t rfid_status;
    uint8_t  rfid_status_2;
};

// 0x0303 小地图点击
struct MapCommand
{
    float    target_position_x;
    float    target_position_y;
    uint8_t  cmd_keyboard;
    uint8_t  target_robot_id;
    uint16_t cmd_source;
};

// 0x0305 雷达 → 小地图机器人坐标
struct MapRobotData
{
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
};

// 0x0307 路径数据
struct MapPathData
{
    uint8_t  intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t   delta_x[49];
    int8_t   delta_y[49];
    uint16_t sender_id;
};

// 0x0308 自定义文字
struct CustomInfo
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t  user_data[30]; // UTF-16 原始字节
};

// 0x0304 键鼠数据（图传链路）
struct RemoteControl
{
    int16_t  mouse_x;
    int16_t  mouse_y;
    int16_t  mouse_z;
    int8_t   left_button_down;
    int8_t   right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
};

#pragma pack(pop)

// ================= 全部信息聚合 =================

struct RefereeInfo
{
    // 比赛 & 队伍
    GameStatus  gameStatus{};
    GameResult  gameResult{};
    GameRobotHP gameRobotHP{};

    // 本机
    RobotStatus         robotStatus{};
    PowerHeatData       powerHeat{};
    RobotPos            robotPos{};
    Buff                buff{};
    HurtData            hurt{};
    ShootData           shoot{};
    ProjectileAllowance projectileAllowance{};
    RfidStatus          rfidStatus{};

    // 小地图 / 文本
    MapCommand   mapCommand{};
    MapRobotData mapRobotData{};
    MapPathData  mapPathData{};
    CustomInfo   customInfo{};

    // 图传键鼠
    RemoteControl remoteControl{};

    // 简单状态标志
    bool hasGameStatus    = false;
    bool hasRobotStatus   = false;
    bool hasPowerHeat     = false;
    bool hasRobotPos      = false;
    bool hasBuff          = false;
    bool hasRemoteControl = false;
};

// 链路类型：常规链路 or 图传链路
enum class RefereeLink : uint8_t
{
    Main,   // 常规链路（电源管理 User 串口）
    Vision  // 图传链路（图传 UART）
};

// ================= 主解析类 =================

class RefereeParser
{
public:
    explicit RefereeParser(RefereeLink link);

    // 串口每收到一个字节，就调用一次
    void onByteReceived(uint8_t byte);

    // 获取解析结果
    const RefereeInfo& getInfo() const { return info_; }

    // 可选：重置状态机
    void reset();

    // CRC
    uint8_t  calcCRC8(const uint8_t* msg, uint32_t len, uint8_t crc) const;
    uint16_t calcCRC16(const uint8_t* msg, uint32_t len, uint16_t crc) const;

private:
    // 状态机阶段
    enum class Step : uint8_t
    {
        WaitSOF = 0,
        ReadHeader,
        ReadBody
    };

    struct Context
    {
        Step    step      = Step::WaitSOF;
        uint16_t dataLen  = 0;
        uint16_t index    = 0;
        uint16_t frameLen = 0;
        uint8_t  buffer[256] = {0};
    };

    void parseByte(uint8_t byte);
    bool checkHeaderCRC() const;
    bool checkFrameCRC() const;
    void handleCompleteFrame();

    void dispatchCmd(uint16_t cmd_id, const uint8_t* data, uint16_t len);

    bool     verifyCRC8(const uint8_t* msg, uint32_t len) const;
    bool     verifyCRC16(const uint8_t* msg, uint32_t len) const;

private:
    RefereeLink link_;
    Context     ctx_;
    RefereeInfo info_;
};

#endif /* __DVC_REFEREE_HPP */
