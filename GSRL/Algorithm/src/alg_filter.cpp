/**
 ******************************************************************************
 * @file           : alg_filter.cpp
 * @brief          : 常用滤波算法
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "alg_filter.hpp"
#include <cmath>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                         KalmanFilter工厂函数
 ******************************************************************************/

// 常用的卡尔曼滤波器类型别名
// 一维卡尔曼滤波器
using KalmanFilter1D = KalmanFilter<fp32, 1, 1, 0>;

// 二维位置-速度滤波器
using KalmanFilter2D = KalmanFilter<fp32, 2, 1, 0>;

// 三维位置-速度-加速度滤波器
using KalmanFilter3D = KalmanFilter<fp32, 3, 1, 0>;

/**
 * @brief 创建二维卡尔曼滤波器（位置-速度模型）
 * @param dt 时间步长
 * @param processNoise 过程噪声方差
 * @param measNoise 测量噪声方差
 * @return 配置好的卡尔曼滤波器
 */
KalmanFilter2D createPosVelKF(fp32 dt, fp32 processNoise = 0.01f, fp32 measNoise = 0.1f)
{
    KalmanFilter2D kf;

    // 状态转移矩阵 [1 dt; 0 1]
    KalmanFilter2D::StateMatrix F;
    F << 1, dt,
        0, 1;
    kf.setTransitionMatrix(F);

    // 观测矩阵 [1 0] (只观测位置)
    KalmanFilter2D::ObsMatrix H;
    H << 1, 0;
    kf.setObservationMatrix(H);

    // 过程噪声协方差矩阵
    KalmanFilter2D::StateMatrix Q;
    fp32 dt2 = dt * dt;
    Q << processNoise * dt2 * dt2 / 4, processNoise * dt2 * dt / 2,
        processNoise * dt2 * dt / 2, processNoise * dt2;
    kf.setProcessNoise(Q);

    // 测量噪声协方差矩阵
    KalmanFilter2D::MeasMatrix R;
    R << measNoise;
    kf.setMeasurementNoise(R);

    return kf;
}

/**
 * @brief 创建三维卡尔曼滤波器（位置-速度-加速度模型）
 * @param dt 时间步长
 * @param processNoise 过程噪声方差
 * @param measNoise 测量噪声方差
 * @return 配置好的卡尔曼滤波器
 */
KalmanFilter3D createPosVelAccKF(fp32 dt, fp32 processNoise = 0.01f, fp32 measNoise = 0.1f)
{
    KalmanFilter3D kf;

    // 状态转移矩阵 [1 dt dt²/2; 0 1 dt; 0 0 1]
    KalmanFilter3D::StateMatrix F;
    fp32 dt2 = dt * dt;
    F << 1, dt, dt2 / 2,
        0, 1, dt,
        0, 0, 1;
    kf.setTransitionMatrix(F);

    // 观测矩阵 [1 0 0] (只观测位置)
    KalmanFilter3D::ObsMatrix H;
    H << 1, 0, 0;
    kf.setObservationMatrix(H);

    // 过程噪声协方差矩阵
    KalmanFilter3D::StateMatrix Q;
    fp32 dt3 = dt2 * dt;
    fp32 dt4 = dt3 * dt;
    Q << processNoise * dt4 / 4, processNoise * dt3 / 2, processNoise * dt2 / 2,
        processNoise * dt3 / 2, processNoise * dt2, processNoise * dt,
        processNoise * dt2 / 2, processNoise * dt, processNoise;
    kf.setProcessNoise(Q);

    // 测量噪声协方差矩阵
    KalmanFilter3D::MeasMatrix R;
    R << measNoise;
    kf.setMeasurementNoise(R);

    return kf;
}

/**
 * @brief 创建四维状态卡尔曼滤波器（二维位置+速度模型）
 * @param dt 时间步长
 * @param processNoise 过程噪声方差
 * @param measNoise 测量噪声方差
 * @return 4维状态的卡尔曼滤波器（x, vx, y, vy）
 */
KalmanFilter<fp32, 4, 2, 0> create2DPosVelKF(fp32 dt,
                                             fp32 processNoise = 0.01f,
                                             fp32 measNoise    = 0.1f)
{
    KalmanFilter<fp32, 4, 2, 0> kf;

    // 状态: [x, vx, y, vy]
    // 测量: [x_meas, y_meas]

    // 状态转移矩阵
    KalmanFilter<fp32, 4, 2, 0>::StateMatrix F;
    F << 1, dt, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, dt,
        0, 0, 0, 1;
    kf.setTransitionMatrix(F);

    // 观测矩阵（只观测位置）
    KalmanFilter<fp32, 4, 2, 0>::ObsMatrix H;
    H << 1, 0, 0, 0,
        0, 0, 1, 0;
    kf.setObservationMatrix(H);

    // 过程噪声
    KalmanFilter<fp32, 4, 2, 0>::StateMatrix Q;
    fp32 dt2 = dt * dt;
    Q << processNoise * dt2 * dt2 / 4, processNoise * dt2 * dt / 2, 0, 0,
        processNoise * dt2 * dt / 2, processNoise * dt2, 0, 0,
        0, 0, processNoise * dt2 * dt2 / 4, processNoise * dt2 * dt / 2,
        0, 0, processNoise * dt2 * dt / 2, processNoise * dt2;
    kf.setProcessNoise(Q);

    // 测量噪声
    KalmanFilter<fp32, 4, 2, 0>::MeasMatrix R;
    R << measNoise, 0,
        0, measNoise;
    kf.setMeasurementNoise(R);

    return kf;
}

/**
 * @brief 创建带控制输入的二维位置-速度卡尔曼滤波器
 * @param dt 时间步长
 * @param processNoise 过程噪声方差
 * @param measNoise 测量噪声方差
 * @return 4维状态2维控制的卡尔曼滤波器（x, vx, y, vy）
 */
KalmanFilter<fp32, 4, 2, 2> create2DPosVelControlKF(fp32 dt,
                                                    fp32 processNoise = 0.01f,
                                                    fp32 measNoise    = 0.1f)
{
    KalmanFilter<fp32, 4, 2, 2> kf;

    // 状态: [x, vx, y, vy]
    // 测量: [x_meas, y_meas]
    // 控制: [ax, ay] (加速度控制)

    // 状态转移矩阵 F
    KalmanFilter<fp32, 4, 2, 2>::StateMatrix F;
    F << 1, dt, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, dt,
        0, 0, 0, 1;
    kf.setTransitionMatrix(F);

    // 控制矩阵 B
    KalmanFilter<fp32, 4, 2, 2>::ControlMatrix B;
    fp32 dt2 = dt * dt;
    B << dt2 / 2, 0,
        dt, 0,
        0, dt2 / 2,
        0, dt;
    kf.setControlMatrix(B);

    // 观测矩阵 H（只观测位置）
    KalmanFilter<fp32, 4, 2, 2>::ObsMatrix H;
    H << 1, 0, 0, 0,
        0, 0, 1, 0;
    kf.setObservationMatrix(H);

    // 过程噪声 Q
    KalmanFilter<fp32, 4, 2, 2>::StateMatrix Q;
    Q << processNoise * dt2 * dt2 / 4, processNoise * dt2 * dt / 2, 0, 0,
        processNoise * dt2 * dt / 2, processNoise * dt2, 0, 0,
        0, 0, processNoise * dt2 * dt2 / 4, processNoise * dt2 * dt / 2,
        0, 0, processNoise * dt2 * dt / 2, processNoise * dt2;
    kf.setProcessNoise(Q);

    // 测量噪声 R
    KalmanFilter<fp32, 4, 2, 2>::MeasMatrix R;
    R << measNoise, 0,
        0, measNoise;
    kf.setMeasurementNoise(R);

    return kf;
}
