/**
 ******************************************************************************
 * @file           : board_config.h
 * @brief          : Board hardware abstraction layer configuration
 *                   板级硬件抽象层配置
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// 根据定义的芯片宏包含对应的HAL库头文件
#if defined(STM32H723xx)
    #include "stm32h7xx_hal.h"
    #include "fdcan.h"
    #include "usart.h"
    #include "spi.h"
    #include "tim.h"
    #include "gpio.h"
#elif defined(STM32F407xx)
    #include "stm32f4xx_hal.h"
    #include "can.h"
    #include "usart.h"
    #include "spi.h"
    #include "tim.h"
    #include "gpio.h"
#else
    // 默认包含H7头文件以避免IDE报错，实际编译时应由CMake定义宏
    #include "stm32h7xx_hal.h"
    #include "fdcan.h"
    #include "usart.h"
    #include "spi.h"
    #include "tim.h"
    #include "gpio.h"
#endif

/* Exported constants --------------------------------------------------------*/

// --- CAN/FDCAN 映射 ---
#if defined(STM32H723xx)
    // H7 使用 FDCAN
    #define MOTOR_CAN_HANDLE        hfdcan3
    // 如果有其他CAN总线，继续定义
    // #define CHASSIS_CAN_HANDLE      hfdcan1
#elif defined(STM32F407xx)
    // F4 使用 CAN
    #define MOTOR_CAN_HANDLE        hcan1
    // #define CHASSIS_CAN_HANDLE      hcan2
#else
    // 默认定义
    #define MOTOR_CAN_HANDLE        hfdcan3
#endif

// --- UART/USART 映射 ---
#if defined(STM32H723xx)
    #define RC_UART_HANDLE          huart3
    #define H30_UART_HANDLE         huart1
#elif defined(STM32F407xx)
    #define RC_UART_HANDLE          huart3
    #define H30_UART_HANDLE         huart1
#else
    #define RC_UART_HANDLE          huart3
    #define H30_UART_HANDLE         huart1
#endif

// --- SPI 映射 ---
#if defined(STM32H723xx)
    #define IMU_SPI_HANDLE          hspi3
#elif defined(STM32F407xx)
    #define IMU_SPI_HANDLE          hspi1
#else
    #define IMU_SPI_HANDLE          hspi3
#endif

// --- GPIO 映射 ---
#if defined(STM32H723xx)
    #define IMU_CS_ACCEL_GPIO_PORT  GPIOA
    #define IMU_CS_ACCEL_PIN        GPIO_PIN_4
    #define IMU_CS_GYRO_GPIO_PORT   GPIOB
    #define IMU_CS_GYRO_PIN         GPIO_PIN_0
#elif defined(STM32F407xx)
    #define IMU_CS_ACCEL_GPIO_PORT  GPIOA
    #define IMU_CS_ACCEL_PIN        GPIO_PIN_4
    #define IMU_CS_GYRO_GPIO_PORT   GPIOB
    #define IMU_CS_GYRO_PIN         GPIO_PIN_0
#else
    #define IMU_CS_ACCEL_GPIO_PORT  GPIOA
    #define IMU_CS_ACCEL_PIN        GPIO_PIN_4
    #define IMU_CS_GYRO_GPIO_PORT   GPIOB
    #define IMU_CS_GYRO_PIN         GPIO_PIN_0
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_CONFIG_H__ */
