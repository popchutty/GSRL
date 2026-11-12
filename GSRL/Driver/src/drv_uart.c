/**
 ******************************************************************************
 * @file           : drv_uart.c
 * @brief          : UART驱动
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
UART_Manage_Object_t s_uart_manage_objects[8] = {0}; // UART管理对象

/**
 * @brief UART管理实例数组
 * @note 根据board_config.h中的宏定义配置决定使用的UART实例
 */
static USART_TypeDef *const uartInstances[8] = {
#ifdef USE_USART1
    USART1,
#endif
#ifdef USE_USART2
    USART2,
#endif
#ifdef USE_USART3
    USART3,
#endif
#ifdef USE_UART4
    UART4,
#endif
#ifdef USE_UART5
    UART5,
#endif
#ifdef USE_USART6
    USART6,
#endif
#ifdef USE_UART7
    UART7,
#endif
#ifdef USE_UART8
    UART8
#endif
};

/* User code -----------------------------------------------------------------*/

/**
 * @brief 获取UART对象
 * @param huart UART句柄
 * @return UART管理对象指针
 */
static UART_Manage_Object_t *UART_Get_Object(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < 8; i++) {
        if (huart->Instance == uartInstances[i]) {
            return &s_uart_manage_objects[i];
        }
    }
    return NULL;
}

/**
 * @brief 判断环形缓冲区是否为空
 * @param uart_obj UART管理对象指针
 * @return 1为空，0为非空
 * @note 读写指针相等即表示缓冲区为空
 */
static inline uint8_t UART_RingBuffer_IsEmpty(UART_Manage_Object_t *uart_obj) // 环形缓冲区是不是空的
{
    return (uart_obj->rx_head == uart_obj->rx_tail); // 读写指针指向同一处就是空，不同处就是非空
}

/**
 * @brief 计算环形缓冲区中已缓存的数据长度
 * @param uart_obj UART管理对象指针
 * @return 已缓存数据长度（字节）
 */
static inline uint16_t UART_RingBuffer_GetLength(UART_Manage_Object_t *uart_obj) // 环形缓冲区里已经存了多少数据
{
    return (uint16_t)(uart_obj->rx_head - uart_obj->rx_tail); // 写指针减读指针，得到已有数据量
}

/**
 * @brief 环形缓冲区剩余可用空间
 * @param uart_obj UART管理对象指针
 * @return 剩余可用空间（字节）
 * @note 保留一个字节的空位，用于区分"空"和"满"两种状态
 */
static inline uint16_t UART_RingBuffer_GetFree(UART_Manage_Object_t *uart_obj) // 缓冲区还剩多少空位
{
    return (uint16_t)(UART_BUFFER_SIZE - 1 - UART_RingBuffer_GetLength(uart_obj)); // 总容量减1再减已有数据量，即剩余空间
}

/**
 * @brief 初始化UART，启动DMA接收
 * @param huart UART句柄
 * @param rxCallbackFunction 处理回调函数
 * @param rxDataLimit 空闲中断最大接收数据长度，不得超过drv_uart.h中定义的缓冲区大小UART_BUFFER_SIZE
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back rxCallbackFunction, uint16_t rxDataLimit)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj != NULL) {
        uart_obj->huart              = huart;
        uart_obj->rxDataLimit        = rxDataLimit;
        uart_obj->rxCallbackFunction = rxCallbackFunction;

        /* 环形缓冲区读写指针清零，保证初始化后从空缓冲区开始 */
        uart_obj->rx_head = 0;
        uart_obj->rx_tail = 0;

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
    }
}

/**
 * @brief 重新初始化UART并重启DMA接收
 * @param huart UART句柄
 */
void UART_Reinit(UART_HandleTypeDef *huart)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj != NULL) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
    }
}

/**
 * @brief 发送数据帧
 * @param huart UART句柄
 * @param pTxData 被发送的数据指针
 * @param txDataLength 发送数据长度
 * @retval HAL status
 */
HAL_StatusTypeDef UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *pTxData, uint16_t txDataLength)
{
    return (HAL_UART_Transmit_DMA(huart, pTxData, txDataLength));
}

/**
 * @brief 从环形缓冲区读取数据（非阻塞）
 * @param huart UART句柄
 * @param pData 存放读取数据的指针
 * @param len 期望读取的数据长度
 * @return 实际读到的数据长度
 * @note 环形缓冲区没有数据时立即返回0，不会阻塞等待
 */
uint16_t UART_Read(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    uint16_t read_len = 0;
    uint16_t i;

    if (uart_obj == NULL || pData == NULL || len == 0) {
        return 0;
    }

    /* 实际可读长度取"期望长度"和"缓冲区已有数据长度"的较小值 */
    read_len = UART_RingBuffer_GetLength(uart_obj);
    if (read_len > len) {
        read_len = len;
    }

    /* 从读指针位置依次取出数据 */
    for (i = 0; i < read_len; i++) {
        pData[i] = uart_obj->rxRingBuffer[uart_obj->rx_tail];
        /* 读指针前进，并自动回绕 */
        uart_obj->rx_tail = (uint16_t)((uart_obj->rx_tail + 1) % UART_BUFFER_SIZE);
    }

    return read_len;
}

/**
 * @brief HAL库UART接收空闲中断
 * @param huart UART编号
 * @param Size 本次接收到的数据长度
 * @details 将DMA接收到的数据循环写入环形缓冲区（满则丢弃），
 *          再调用原有回调函数（对上层透明），最后重启DMA接收
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) // 哪个串口触发的，以及这次收到了多少数据
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj != NULL) {
        uint16_t i;
        /* 将本次DMA收到的数据逐字节写入环形缓冲区 */
        for (i = 0; i < Size; i++) {
            /* 缓冲区满（可用空间为0）时直接丢弃新数据 */
            if (UART_RingBuffer_GetFree(uart_obj) == 0) {
                break;
            }
            uart_obj->rxRingBuffer[uart_obj->rx_head] = uart_obj->rxBuffer[i];
            /* 写指针前进，并自动回绕 */
            uart_obj->rx_head = (uint16_t)((uart_obj->rx_head + 1) % UART_BUFFER_SIZE);
        }

        /* 保留原有回调调用，保证对上层透明 */
        if (uart_obj->rxCallbackFunction != NULL) {
            uart_obj->rxCallbackFunction(uart_obj->rxBuffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
    }
}

/**
 * @brief HAL库UART接收错误中断
 * @param huart UART编号
 * @details 发生错误重启DMA接收
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj != NULL) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
        if (uart_obj->rxCallbackFunction != NULL) {
            uart_obj->rxCallbackFunction(NULL, 0);
        }
    }
}
