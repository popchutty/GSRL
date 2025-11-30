/**
 ******************************************************************************
 * @file           : drv_can.c
 * @brief          : CAN driver
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
CAN_Manage_Object_t s_can_manage_objects[2] = {0};   // CAN管理对象
static bool_t is_queue_initialized          = false; // 队列初始化标志

/**
 * @brief CAN管理实例数组
 * @note 根据board_config.h中的宏定义配置决定使用的CAN实例
 */
static CAN_TypeDef *const canInstances[2] = {
#ifdef USE_CAN1
    CAN1,
#endif
#ifdef USE_CAN2
    CAN2
#endif
};

/* Function prototypes -----------------------------------------------*/
static void can_all_pass_filter_init(CAN_HandleTypeDef *hcan);

/* User code ---------------------------------------------------------*/

/**
 * @brief 获取CAN对象
 * @param hcan CAN句柄
 * @return CAN管理对象指针
 */
static CAN_Manage_Object_t *CAN_Get_Object(CAN_HandleTypeDef *hcan)
{
    for (int i = 0; i < 2; i++) {
        if (hcan->Instance == canInstances[i]) {
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
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back rxCallbackFunction)
{
    // 找到对应的CAN管理对象并设置参数
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    if (can_obj == NULL) return;

    can_obj->hcan               = hcan;
    can_obj->rxCallbackFunction = rxCallbackFunction;

    // 初始化滤波器
    can_all_pass_filter_init(hcan);

    // 启动CAN
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

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
HAL_StatusTypeDef CAN_Send_Data(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData)
{
    return HAL_CAN_AddTxMessage(hcan, pTxHeader, pTxData, NULL);
}

/**
 * @brief 初始化CAN全通过滤器
 *        Initalize CAN all-pass filter
 * @param None
 * @retval None
 */
static void can_all_pass_filter_init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef can_filter_st; // 全通过滤器
    can_filter_st.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh         = 0x0000;
    can_filter_st.FilterIdLow          = 0x0000;
    can_filter_st.FilterMaskIdHigh     = 0x0000;
    can_filter_st.FilterMaskIdLow      = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_FilterFIFO0;
    can_filter_st.FilterActivation     = CAN_FILTER_ENABLE;

    if (hcan->Instance == CAN1) {
        can_filter_st.FilterBank = 0;
        HAL_CAN_ConfigFilter(hcan, &can_filter_st);
    }

    if (hcan->Instance == CAN2) {
        can_filter_st.SlaveStartFilterBank = 14;
        can_filter_st.FilterBank           = 14;
        HAL_CAN_ConfigFilter(hcan, &can_filter_st);
    }
}

/**
 * @brief HAL库CAN中断回调函数
 *        HAL library CAN interrupt callback function
 * @param hcan CAN句柄
 * @retval None
 * @note 这里其实有两层回调，第一层是HAL_CAN_RxFifo0MsgPendingCallback，语法是HAL库规定的，在CAN收到数据后会自动调用
 *       第二层是手动注册的回调函数rxCallbackFunction，在CAN_Init中
 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_rx_message_t s_rx_msg;//定义接受消息的结构体
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
