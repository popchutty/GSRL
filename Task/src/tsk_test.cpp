/**
 *******************************************************************************
 * @file           : tsk_test.cpp
 * @brief          : MG电机驱动测试任务
 *******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_motor.hpp"
#include "dvc_remotecontrol.hpp"
#include "alg_general.hpp"

/* Define --------------------------------------------------------------------*/
// PID 控制器参数
SimplePID::PIDParam param = {
    100,  // Kp
    0,   // Ki
    100, // Kd
    10000.0f,  // outputLimit
    11110.0f    // integralLimit
};

// 创建 PID 控制器实例
SimplePID myPID(SimplePID::PID_POSITION, param);

// MG 电机实例（1 为电机 ID，myPID 为控制器，0 为编码器偏移量）
MotorMG8016EI6 mgMotor(1, &myPID, 0);

/* Variables -----------------------------------------------------------------*/
GSRLMath::Vector3f eulerAngle;

/* Function prototypes -------------------------------------------------------*/
extern "C" void can1RxCallback(can_rx_message_t *pRxMsg);
inline void transmitMotorsControlData();

/* User code -----------------------------------------------------------------*/

/**
 * @brief 测试任务
 * @param argument 任务参数
 */
extern "C" void test_task(void *argument)
{
    // 初始化 CAN1，并绑定回调函数
    CAN_Init(&hcan1, can1RxCallback);
    
    // 获取任务开始时间
    TickType_t taskLastWakeTime = xTaskGetTickCount();

  
    while (1) {
        // mgMotor.openloopControl(-2048); // 开环控制，控制值为 count2               
        mgMotor.hardwareConvertAngularVelocityToMotorContorlData(-1); // 角速度闭环控制
        // mgMotor.angularVelocityClosedloopControl(-10);
        // mgMotor.angleClosedloopControl(90*3.14159f/180);
        // mgMotor.convertSingleCircleAngleToMotorControlData(0, 1.0f, false); // 单圈角度闭环控制
        // mgMotor.convertMutipleCircleAngleToMotorControlData(0, 1);
        // 传输电机控制数据
        transmitMotorsControlData();

        // 让任务延时直到下次周期 (1ms)
        vTaskDelayUntil(&taskLastWakeTime, 1);
    }
}

/**
 * @brief CAN1 接收回调函数
 * @param pRxMsg 接收到的 CAN 消息
 */
extern "C" void can1RxCallback(can_rx_message_t *pRxMsg)
{
    mgMotor.decodeCanRxMessageFromISR(pRxMsg); // 调用 MG 电机的解析函数
}


/**
 * @brief 传输电机控制数据
 */
inline void transmitMotorsControlData()
{
    // 这里可以将电机的控制数据通过 CAN 总线发送出去
    // 例如发送控制数据帧至 CAN 总线，具体实现依据你的硬件和 CAN 库
    uint8_t controlData[8]; // 控制数据（此处为示例，可以根据实际需要修改）
    // 获取电机控制数据
    memcpy(controlData, mgMotor.getMotorControlData(), sizeof(controlData));
    uint32_t mailbox;
    // 将数据发送到 CAN 总线
    HAL_CAN_AddTxMessage(&hcan1, mgMotor.getMotorControlHeader(), controlData, &mailbox);

}
