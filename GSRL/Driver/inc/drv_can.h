/**
 ******************************************************************************
 * @file           : drv_can.h
 * @brief          : Header for drv_can.c file.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"
#include "board_config.h"
#include "cmsis_os.h"

/* Exported types ------------------------------------------------------------*/

// --- CAN 类型抽象 ---
#if defined(STM32H723xx)
    typedef FDCAN_HandleTypeDef CAN_Handle_t;
    typedef FDCAN_TxHeaderTypeDef CAN_TxHeader_t;
    typedef FDCAN_RxHeaderTypeDef CAN_RxHeader_t;
    // H7 FDCAN 使用 Identifier 字段
    #define CAN_GET_STD_ID(header)      ((header).Identifier)
    #define CAN_SET_STD_ID(header, id)  ((header).Identifier = (id))
#elif defined(STM32F407xx)
    typedef CAN_HandleTypeDef CAN_Handle_t;
    typedef CAN_TxHeaderTypeDef CAN_TxHeader_t;
    typedef CAN_RxHeaderTypeDef CAN_RxHeader_t;
    // F4 CAN 使用 StdId 字段
    #define CAN_GET_STD_ID(header)      ((header).StdId)
    #define CAN_SET_STD_ID(header, id)  ((header).StdId = (id))
#else
    // 默认 H7
    typedef FDCAN_HandleTypeDef CAN_Handle_t;
    typedef FDCAN_TxHeaderTypeDef CAN_TxHeader_t;
    typedef FDCAN_RxHeaderTypeDef CAN_RxHeader_t;
    #define CAN_GET_STD_ID(header)      ((header).Identifier)
    #define CAN_SET_STD_ID(header, id)  ((header).Identifier = (id))
#endif

/**
 * @brief CAN接收消息结构体
 */
typedef struct
{
    CAN_RxHeader_t header;
    uint8_t data[8];
} can_rx_message_t;

/**
 * @brief CAN通信接收处理回调函数类型定义
 * @param pRxMsg 接收数据结构体指针
 */
typedef void (*CAN_Call_Back)(can_rx_message_t *pRxMsg);

/**
 * @brief CAN通信处理结构体
 */
typedef struct
{
    CAN_Handle_t *hcan;
    CAN_Call_Back rxCallbackFunction;
} CAN_Manage_Object_t;

/* Exported constants --------------------------------------------------------*/
extern osMessageQueueId_t canRxQueueHandle;
extern CAN_Manage_Object_t s_can_manage_objects[3]; // CAN管理对象

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void CAN_Init(CAN_Handle_t *hcan, CAN_Call_Back rxCallbackFunction);
HAL_StatusTypeDef CAN_Send_Data(CAN_Handle_t *hcan, CAN_TxHeader_t *pTxHeader, uint8_t *pTxData);

/* Defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CAN_H */
