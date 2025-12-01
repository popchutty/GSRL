/**
 ******************************************************************************
 * @file           : alg_ahrs.hpp
 * @brief          : header file for alg_ahrs.cpp
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"
#include "alg_general.hpp"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief AHRS姿态解算算法接口
 */
class AHRS
{
protected:
    using Vector3f = GSRLMath::Vector3f;
    Vector3f m_gyro;
    Vector3f m_accel;
    Vector3f m_magnet;
    Vector3f m_eulerAngle;
    fp32 m_quaternion[4];
    bool m_isAhrsInited; // AHRS初始化完成标志

public:
    virtual ~AHRS() = default;
    virtual void reset();
    virtual void init() = 0; // AHRS初始化纯虚函数，在第一次update被调用时执行
    const Vector3f &update(const Vector3f &gyro, const Vector3f &accel, const Vector3f &magnet = Vector3f());
    virtual const Vector3f &getGyro() const;
    virtual const Vector3f &getAccel() const;
    virtual const Vector3f &getMotionAccelBodyFrame() const = 0;
    virtual const Vector3f &getMotionAccelEarthFrame() const = 0;
    const fp32 *getQuaternion() const;
    const Vector3f &getEulerAngle() const;

protected:
    AHRS();
    virtual void dataProcess() = 0;
    void convertQuaternionToEulerAngle();
};

class Mahony : public AHRS
{
private:
    // 采样频率相关
    fp32 m_sampleFreq;              // sample frequency in Hz
    fp32 m_deltaTime;               // sample period in seconds
    uint32_t m_lastUpdateTimestamp; // DWT counter value of the last update
    // PI补偿器相关
    fp32 m_integralFBx, m_integralFBy, m_integralFBz;
    fp32 m_twoKi, m_twoKp;
    // 加速度低通滤波相关
    Vector3f m_accelFilterHistory[3]; // 0: oldest, 2: newest
    Vector3f m_accelFiltered;
    Vector3f m_accelFilterNum;
    // 运动加速度相关
    Vector3f m_motionAccelBodyFrame;  // 机体坐标系下的运动加速度
    Vector3f m_motionAccelEarthFrame; // 大地坐标系下的运动加速度

public:
    Mahony(fp32 sampleFreq = 0.0f, Vector3f accelFilterNum = 0, fp32 Kp = 0.5f, fp32 Ki = 0.0f);
    void reset() override;
    void init() override;
    const Vector3f &getAccel() const override;
    const Vector3f &getMotionAccelBodyFrame() const override;
    const Vector3f &getMotionAccelEarthFrame() const override;

private:
    void dataProcess() override;
    void sixAxisProcess(fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az);
    void nineAxisProcess(fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az, fp32 mx, fp32 my, fp32 mz);
    void filterAccel();
    void calculateMotionAccel();
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
