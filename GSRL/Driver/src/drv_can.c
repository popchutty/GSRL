/**
 ******************************************************************************
 * @file           : drv_can.c
 * @brief          : CAN driver (FDCAN/bxCAN adaptation)
 *                   Contains CAN initialization related functions,
 *                   CAN receive interrupt callback function
 *                   Use queue to save received data when using FreeRTOS,
 *                   otherwise use global variables
 *                   CAN驱动
 *                   包含CAN初始化相关函数，CAN接收中断回调函数
 *                   使用FreeRTOS时使用队列保存接收数据，否则使用全局变量
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"
#include "queue.h"

/* Typedef -----------------------------------------------------------*/

/* Define ------------------------------------------------------------*/
osMessageQueueId_t canRxQueueHandle;
const osMessageQueueAttr_t canRxQueue_attributes = {
    .name = "canRxQueue"};

/* Macro -------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
CAN_Manage_Object_t s_can_manage_objects[3] = {0};   // CAN管理对象
static bool_t is_queue_initialized          = false; // 队列初始化标志

/**
 * @brief CAN管理实例数组
 * @note 根据board_config.h中的宏定义配置决定使用的CAN实例
 */
#if defined(STM32H723xx)
static FDCAN_GlobalTypeDef *const canInstances[3] = {
    #ifdef FDCAN1
    FDCAN1,
    #else
    NULL,
    #endif
    #ifdef FDCAN2
    FDCAN2,
    #else
    NULL,
    #endif
    #ifdef FDCAN3
    FDCAN3
    #else
    NULL
    #endif
};
#elif defined(STM32F407xx)
static CAN_TypeDef *const canInstances[3] = {
    #ifdef CAN1
    CAN1,
    #else
    NULL,
    #endif
    #ifdef CAN2
    CAN2,
    #else
    NULL,
    #endif
    #ifdef CAN3
    CAN3
    #else
    NULL
    #endif
};
#else
// Default H7
static FDCAN_GlobalTypeDef *const canInstances[3] = {
    #ifdef FDCAN1
    FDCAN1,
    #else
    NULL,
    #endif
    #ifdef FDCAN2
    FDCAN2,
    #else
    NULL,
    #endif
    #ifdef FDCAN3
    FDCAN3
    #else
    NULL
    #endif
};
#endif

/* Function prototypes -----------------------------------------------*/
static void can_all_pass_filter_init(CAN_Handle_t *hcan);

/* User code ---------------------------------------------------------*/

/**
 * @brief 获取CAN对象
 * @param hcan CAN句柄
 * @return CAN管理对象指针
 */
static CAN_Manage_Object_t *CAN_Get_Object(CAN_Handle_t *hcan)
{
    for (int i = 0; i < 3; i++) {
        if (canInstances[i] != NULL && hcan->Instance == canInstances[i]) {
            return &s_can_manage_objects[i];
        }
    }
    return NULL;
}

/**
 * @brief 初始化CAN，注册回调函数，并启动CAN
 * @param hcan CAN句柄
 * @param rxCallbackFunction 处理回调函数
 */
void CAN_Init(CAN_Handle_t *hcan, CAN_Call_Back rxCallbackFunction)
{
    // 找到对应的CAN管理对象并设置参数
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    if (can_obj == NULL) return;

    can_obj->hcan               = hcan;
    can_obj->rxCallbackFunction = rxCallbackFunction;

    // 初始化滤波器
    can_all_pass_filter_init(hcan);

    // 启动CAN
#if defined(STM32H723xx)
    HAL_FDCAN_Start(hcan);
    HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
#elif defined(STM32F407xx)
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
#else
    HAL_FDCAN_Start(hcan);
    HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
#endif

    // 确保队列只初始化一次
    if (is_queue_initialized == false) {
        canRxQueueHandle     = osMessageQueueNew(16, sizeof(can_rx_message_t), &canRxQueue_attributes);
        is_queue_initialized = true;
    }
}

/**
 * @brief CAN发送消息
 * @param hcan CAN句柄
 * @param pTxHeader 发送帧头指针
 * @param pTxData 发送数据指针
 * @return halsatus
 */
HAL_StatusTypeDef CAN_Send_Data(CAN_Handle_t *hcan, CAN_TxHeader_t *pTxHeader, uint8_t *pTxData)
{
#if defined(STM32H723xx)
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, pTxHeader, pTxData);
#elif defined(STM32F407xx)
    uint32_t txMailbox;
    return HAL_CAN_AddTxMessage(hcan, pTxHeader, pTxData, &txMailbox);
#else
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, pTxHeader, pTxData);
#endif
}

/**
 * @brief 初始化CAN全通过滤器
 *        Initalize CAN all-pass filter
 * @param None
 * @retval None
 */
static void can_all_pass_filter_init(CAN_Handle_t *hcan)
{
#if defined(STM32H723xx)
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;

    HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
#elif defined(STM32F407xx)
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0; // 注意：如果使用双CAN，CAN2的FilterBank需要偏移
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
#else
    // Default H7
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;

    HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
#endif
}

/**
 * @brief HAL库CAN中断回调函数
 *        HAL library CAN interrupt callback function
 * @param hcan CAN句柄
 * @retval None
 */
#if defined(STM32H723xx)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        can_rx_message_t s_rx_msg;
        if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &s_rx_msg.header, s_rx_msg.data) == HAL_OK) {

            // 1. 使用队列保存接收数据（保持原有功能）
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (xQueueIsQueueFullFromISR(canRxQueueHandle)) // 队列满,移除最早的数据
            {
                can_rx_message_t s_dummy_msg;
                xQueueReceiveFromISR(canRxQueueHandle, &s_dummy_msg, &xHigherPriorityTaskWoken);
            }
            // 写入新数据
            xQueueSendToBackFromISR(canRxQueueHandle, &s_rx_msg, &xHigherPriorityTaskWoken);

            // 2. 调用注册的回调函数（新增功能）
            CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
            if (can_obj != NULL && can_obj->rxCallbackFunction != NULL) {
                can_obj->rxCallbackFunction(&s_rx_msg);
            }

            // 重新发起调度
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
#elif defined(STM32F407xx)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_rx_message_t s_rx_msg;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &s_rx_msg.header, s_rx_msg.data) == HAL_OK) {

        // 1. 使用队列保存接收数据（保持原有功能）
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (xQueueIsQueueFullFromISR(canRxQueueHandle)) // 队列满,移除最早的数据
        {
            can_rx_message_t s_dummy_msg;
            xQueueReceiveFromISR(canRxQueueHandle, &s_dummy_msg, &xHigherPriorityTaskWoken);
        }
        // 写入新数据
        xQueueSendToBackFromISR(canRxQueueHandle, &s_rx_msg, &xHigherPriorityTaskWoken);

        // 2. 调用注册的回调函数（新增功能）
        CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
        if (can_obj != NULL && can_obj->rxCallbackFunction != NULL) {
            can_obj->rxCallbackFunction(&s_rx_msg);
        }

        // 重新发起调度
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
#else
// Default H7
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        can_rx_message_t s_rx_msg;
        if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &s_rx_msg.header, s_rx_msg.data) == HAL_OK) {

            // 1. 使用队列保存接收数据（保持原有功能）
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (xQueueIsQueueFullFromISR(canRxQueueHandle)) // 队列满,移除最早的数据
            {
                can_rx_message_t s_dummy_msg;
                xQueueReceiveFromISR(canRxQueueHandle, &s_dummy_msg, &xHigherPriorityTaskWoken);
            }
            // 写入新数据
            xQueueSendToBackFromISR(canRxQueueHandle, &s_rx_msg, &xHigherPriorityTaskWoken);

            // 2. 调用注册的回调函数（新增功能）
            CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
            if (can_obj != NULL && can_obj->rxCallbackFunction != NULL) {
                can_obj->rxCallbackFunction(&s_rx_msg);
            }

            // 重新发起调度
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
#endif
