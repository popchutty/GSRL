/**
 ******************************************************************************
 * @file           : drv_misc.c
 * @brief          : 杂项驱动
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_misc.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                            延迟类函数
 ******************************************************************************/

/**
 * @brief 微妙级延时函数
 * @param us 延时的微秒数
 */
void Delay_us(uint16_t us)
{
    // 计算每微秒的系统时钟周期数
    uint32_t cycles_per_us = SystemCoreClock / 1000000;

    // 计算需要延时的总周期数
    uint32_t wait_cycles = us * cycles_per_us;

    // 对于非常短的延时使用NOP指令以避免计算开销超过延时本身
    if (wait_cycles < 10) {
        for (uint32_t i = 0; i < wait_cycles * 4; i++) {
            __NOP();
        }
        return;
    }

    // 获取当前SysTick计数值
    uint32_t start = SysTick->VAL;
    uint32_t current;
    uint32_t elapsed = 0;

    // 获取SysTick的重载值
    uint32_t load = SysTick->LOAD;

    // 等待直到经过的周期数达到需要延时的周期数
    do {
        current = SysTick->VAL;

        // 计算经过的周期数，需要考虑计数器溢出的情况
        if (current <= start) {
            // 未溢出情况
            elapsed = start - current;
        } else {
            // 发生溢出情况
            elapsed = (start + load + 1) - current;
        }
    } while (elapsed < wait_cycles);
}

/**
 * @brief 毫秒级延时函数
 * @param ms 延时的毫秒数
 */
void Delay_ms(uint16_t ms)
{
    // 对于短时间延时直接调用微秒延时
    if (ms <= 10) {
        Delay_us(ms * 1000);
        return;
    }

    // 计算每毫秒的系统时钟周期数
    uint32_t cycles_per_ms = SystemCoreClock / 1000;

    // 对于较长时间延时，分段处理以避免计数器溢出
    for (uint16_t i = 0; i < ms; i++) {
        // 获取当前SysTick计数值
        uint32_t start = SysTick->VAL;
        uint32_t current;
        uint32_t elapsed = 0;

        // 获取SysTick的重载值
        uint32_t load = SysTick->LOAD;

        // 延时一毫秒
        do {
            current = SysTick->VAL;

            // 计算经过的周期数，需要考虑计数器溢出的情况
            if (current <= start) {
                // 未溢出情况
                elapsed = start - current;
            } else {
                // 发生溢出情况
                elapsed = (start + load + 1) - current;
            }
        } while (elapsed < cycles_per_ms);
    }
}

/******************************************************************************
 *                            DWT计时器
 ******************************************************************************/

DWT_Time_t SysTime;                                          // 系统时间
static uint32_t CYCCNT_RountCount;                           // 计数器溢出次数
static uint32_t CYCCNT_LAST;                                 // 上一次的计数值
static uint64_t CYCCNT64;                                    // 64位计数值
static bool_t isDWTInit = false;                             // DWT初始化标志

static void DWT_CNT_Update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++;
    CYCCNT_LAST = cnt_now;
}

/**
 * @brief 初始化DWT计时器
 */
void DWT_Init(void)
{
    if (isDWTInit)
        return;
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;
    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    CYCCNT_RountCount = 0;
    isDWTInit         = true;
}

/**
 * @brief 检测DWT是否初始化
 * @return true 已初始化
 * @return false 未初始化
 */
bool DWT_IsInit(void)
{
    return isDWTInit;
}

/**
 * @brief 获取DWT高精度时间戳
 * @return 32位时间戳
 */
uint32_t DWT_GetTimestamp(void)
{
    return DWT->CYCCNT;
}

/**
 * @brief 获取经过时间
 * @param cnt_last 上一次的计时器计数值, 函数内自动更新
 * @return fp32经过时间(秒)
 */
fp32 DWT_GetDeltaTime(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    fp32 dt                  = ((uint32_t)(cnt_now - *cnt_last)) / ((fp32)(SystemCoreClock));
    *cnt_last                 = cnt_now;
    DWT_CNT_Update();
    return dt;
}

/**
 * @brief 获取经过时间
 * @param cnt_last 上一次的计时器计数值
 * @return fp64经过时间(秒)
 */
fp64 DWT_GetDeltaTime64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    fp64 dt                 = ((uint32_t)(cnt_now - *cnt_last)) / ((fp64)(SystemCoreClock));
    *cnt_last                 = cnt_now;
    DWT_CNT_Update();
    return dt;
}

/**
 * @brief 更新系统时间
 */
void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;
    DWT_CNT_Update();
    CYCCNT64   = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1  = CYCCNT64 / SystemCoreClock;
    CNT_TEMP2  = CYCCNT64 - CNT_TEMP1 * SystemCoreClock;
    SysTime.s  = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / (SystemCoreClock / 1000);
    CNT_TEMP3  = CNT_TEMP2 - SysTime.ms * (SystemCoreClock / 1000);
    SysTime.us = CNT_TEMP3 / (SystemCoreClock / 1000000);
}

/**
 * @brief 获取系统时间轴(秒)
 * @return 时间轴(秒)
 */
fp32 DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();
    fp32 DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;
    return DWT_Timelinef32;
}

/**
 * @brief 获取系统时间轴(毫秒)
 * @return 时间轴(毫秒)
 */
fp32 DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();
    fp32 DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;
    return DWT_Timelinef32;
}

/**
 * @brief 获取系统时间轴(微秒)
 * @return 时间轴(微秒)
 */
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();
    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;
    return DWT_Timelinef32;
}

/**
 * @brief DWT延时函数
 * @param Delay 延时时间(秒)
 */
void DWT_Delay(fp32 Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    fp32 wait         = Delay;
    while ((DWT->CYCCNT - tickstart) < wait * (fp32)SystemCoreClock);
}
