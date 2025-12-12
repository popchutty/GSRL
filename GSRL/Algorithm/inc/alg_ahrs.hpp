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
#include "alg_filter.hpp"

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

/**
 * @brief 卡尔曼解算IMU类实现
 */
class Kalman_Quaternion_EKF : public AHRS
{
private:
    // 初始化相关
    using KF = KalmanFilter<fp32, 6, 3, 0>;
    KF myKalmanFilter;
    uint8_t Initialized; // 初始化完成标志：0=未初始化，1=已初始化
    // 采样频率相关
    float m_sampleFreq;             // 手动输入的固定采样频率，不使用固定采样频率则给0
    uint32_t m_lastUpdateTimestamp; // DWT计数器历史值
    fp32 m_deltaTime;               // 实际采样周期
    // 卡尔曼相关
    float q[4];     // 四元数估计值
    float Q1;       // 四元数更新过程噪声
    float Q2;       // 陀螺仪零偏过程噪声
    float R;        // 加速度计量测噪声
    Vector3f Accel; // 加速度值
    Vector3f Gyro;
    Vector3f GyroBias; // 陀螺仪零偏
    // 卡方检测及细节处理
    float ChiSquareTestThreshold; // 卡方检验阈值
    float accLPFcoef;             // 加速度计一阶低通滤波系数（0~1，越小滤得越重），默认0
    bool isCheckChiSquare;        // 用户给出，判断是否检测卡方 1检测，0不检测
    KF::StateMatrix FadingFactor; // 渐消因子矩阵
public:
    Kalman_Quaternion_EKF(float sampleFreq      = 0.0f,  // 给定刷新频率，给0则自动识别
                          float q1              = 1e-4f, // 四元数过程噪声基准
                          float q2              = 1e-6f, // 零偏过程噪声基准
                          float r               = 1e-2f, // 加计量测噪声基准
                          float accLPF          = 0.2f,  // 加速度计一阶低通滤波系数（0~1，越小滤得越重），默认0
                          bool isCheckChiSquare = 1);    // 是否启用卡方检验

    // 复位
    void reset() override;
    // 参数配置
    void setChiSquareThreshold(float Chi); // 卡方检验阈值

private:
    void dataProcess() override;
    void EKFProcess(fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az);
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
