/**
 ******************************************************************************
 * @file           : drv_can.h
 * @brief          : Header for drv_can.c file.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
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
#include "can.h"
#include "cmsis_os.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN接收消息结构体
 */
typedef struct
{
    CAN_RxHeaderTypeDef header;
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
    CAN_HandleTypeDef *hcan;
    CAN_Call_Back rxCallbackFunction;
} CAN_Manage_Object_t;

/* Exported constants --------------------------------------------------------*/
extern osMessageQueueId_t canRxQueueHandle;
extern CAN_Manage_Object_t s_can_manage_objects[2]; // CAN管理对象

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back rxCallbackFunction);
HAL_StatusTypeDef CAN_Send_Data(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData);

/* Defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CAN_H */
