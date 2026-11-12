/**
 ******************************************************************************
 * @file           : drv_uart.h
 * @brief          : header file for drv_uart.c
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_UART_H
#define __DRV_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"
#include "usart.h"

/* Exported macro ------------------------------------------------------------*/
#define UART_BUFFER_SIZE 512 // UART接收缓冲区最大长度

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收处理回调函数类型定义
 * @param pRxData 接收数据指针
 * @param rxDataLength 接收数据长度
 */
typedef void (*UART_Call_Back)(uint8_t *pRxData, uint16_t rxDataLength);

/**
 * @brief UART通信处理结构体
 */
typedef struct
{
    UART_HandleTypeDef *huart;              // 串口句柄指针，记住当前管理的是哪个串口
    uint8_t rxBuffer[UART_BUFFER_SIZE];     // DMA接收缓冲区（由HAL直接写入）。串口收到的数据先落到这里，相当于一个"水池"
    uint16_t rxDataLimit;                   // 单次接收长度上限，即"水池"最多装多少水

    /* 环形缓冲区（Ring Buffer）：字节流缓存，供UART_Read按字节取用 */
    uint8_t rxRingBuffer[UART_BUFFER_SIZE]; // 环形缓冲区存储区
    volatile uint16_t rx_head;              // 写指针：中断/DMA回调往里写一个，就往前进一步
    volatile uint16_t rx_tail;              // 读指针：应用层读一个，就往前进一步

    UART_Call_Back rxCallbackFunction;      // 接收处理回调函数
} UART_Manage_Object_t;

/* Exported variables --------------------------------------------------------*/
extern UART_Manage_Object_t s_uart_manage_objects[8]; // UART管理对象

/* Exported functions prototypes ---------------------------------------------*/
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back rxCallbackFunction, uint16_t rxDataLimit);
void UART_Reinit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *pTxData, uint16_t txDataLength);

/**
 * @brief 从环形缓冲区读取数据（非阻塞）
 * @param huart UART句柄
 * @param pData 存放读取数据的指针
 * @param len 期望读取的数据长度
 * @return 实际读到的数据长度
 * @note 该函数不改变原有对外接口，上层可自行决定何时调用读取
 */
uint16_t UART_Read(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len);

/* Defines -------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __DRV_UART_H */
