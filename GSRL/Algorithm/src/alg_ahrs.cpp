/**
 ******************************************************************************
 * @file           : alg_ahrs.cpp
 * @brief          : AHRS姿态解算算法
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "drv_misc.h"
#include "alg_ahrs.hpp"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                            AHRS类实现
 ******************************************************************************/

/**
 * @brief 重置AHRS姿态解算相关数据
 * @note 归零传感器缓冲数据、欧拉角, 重置四元数为[1, 0, 0, 0]
 */
void AHRS::reset()
{
    m_gyro          = 0;
    m_accel         = 0;
    m_magnet        = 0;
    m_eulerAngle    = 0;
    m_quaternion[0] = 1.0f;
    m_quaternion[1] = 0.0f;
    m_quaternion[2] = 0.0f;
    m_quaternion[3] = 0.0f;
}

/**
 * @brief AHRS姿态解算更新
 * @param gyro 陀螺仪数据
 * @param accel 加速度计数据
 * @param magnet 磁力计数据
 * @return const Vector3f& 欧拉角解算结果
 * @note 包含姿态解算完整流程, 数据处理、四元数更新、欧拉角计算
 * @note 为确保数据完整性, 在数据处理过程中禁止FreeRTOS任务切换
 */
const AHRS::Vector3f &AHRS::update(const Vector3f &gyro, const Vector3f &accel, const Vector3f &magnet)
{
    m_gyro   = gyro; // 转存传感器数据
    m_accel  = accel;
    m_magnet = magnet;
    if (!m_isAhrsInited) {
        init(); // AHRS初始化
        m_isAhrsInited = true;
    }
    taskENTER_CRITICAL(); // 进入临界区, 禁止任务切换
    dataProcess();
    convertQuaternionToEulerAngle();
    taskEXIT_CRITICAL(); // 退出临界区, 允许任务切换
    return m_eulerAngle;
}

/**
 * @brief 获取陀螺仪数据
 * @return const Vector3f& 陀螺仪数据
 */
const AHRS::Vector3f &AHRS::getGyro() const
{
    return m_gyro;
}

/**
 * @brief 获取加速度计数据
 * @return const Vector3f& 加速度计数据
 */
const AHRS::Vector3f &AHRS::getAccel() const
{
    return m_accel;
}

/**
 * @brief 获取四元数
 * @return const fp32* 四元数数组指针
 */
const fp32 *AHRS::getQuaternion() const
{
    return m_quaternion;
}

/**
 * @brief 获取欧拉角
 * @return const Vector3f& 欧拉角
 */
const AHRS::Vector3f &AHRS::getEulerAngle() const
{
    return m_eulerAngle;
}

/**
 * @brief 构造函数
 * @note 初始化成员变量
 */
AHRS::AHRS()
    : m_gyro(), m_accel(), m_magnet(), m_eulerAngle(), m_isAhrsInited(false)
{
    m_quaternion[0] = 1.0f; // 初始化四元数为[1, 0, 0, 0]
    m_quaternion[1] = 0.0f;
    m_quaternion[2] = 0.0f;
    m_quaternion[3] = 0.0f;
}

/**
 * @brief 四元数转欧拉角
 */
void AHRS::convertQuaternionToEulerAngle()
{
    // 提取四元数分量
    // 四元数顺序为 [w, x, y, z]
    fp32 q0 = m_quaternion[0];
    fp32 q1 = m_quaternion[1];
    fp32 q2 = m_quaternion[2];
    fp32 q3 = m_quaternion[3];

    // 计算欧拉角 (ZYX顺序, 即yaw-pitch-roll)

    // Roll (x轴旋转) - 绕X轴旋转的角度
    m_eulerAngle.x = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    // Pitch (y轴旋转) - 绕Y轴旋转的角度
    // 处理万向锁问题，确保结果在[-π/2, π/2]范围内
    fp32 sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1)
        m_eulerAngle.y = copysignf(MATH_PI / 2, sinp); // 使用copysignf确保正确的符号
    else
        m_eulerAngle.y = asinf(sinp);

    // Yaw (z轴旋转) - 绕Z轴旋转的角度
    m_eulerAngle.z = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
/******************************************************************************
 *                            Mahony类实现
 ******************************************************************************/

/**
 * @brief Mahony姿态解算构造函数
 * @param sampleFreq 采样频率, 默认为0表示使用不定频模式
 * @param accelFilterNum 加速度计低通滤波器系数, 默认为0表示使用大疆官方例程参数
 * @param Kp 比例增益, 默认为0.5
 * @param Ki 积分增益, 默认为0.0
 * @note sampleFreq设置为定频运行时的采样频率(单位Hz), 设置为0表示不定频使用DWT计时, 准确的采样间隔时间有利于更精确的积分计算
 * accelFilterNum为加速度计IIR低通滤波器系数, 默认为0表示使用大疆官方例程参数
 * Kp和Ki分别为比例和积分增益, 用于Mahony算法的PI补偿器
 */
Mahony::Mahony(fp32 sampleFreq, Vector3f accelFilterNum, fp32 Kp, fp32 Ki)
    : AHRS(), m_sampleFreq(sampleFreq), m_deltaTime(0), m_lastUpdateTimestamp(0), m_integralFBx(0), m_integralFBy(0), m_integralFBz(0), m_twoKi(2 * Ki), m_twoKp(2 * Kp), m_accelFilterHistory(), m_accelFiltered()
{
    DWT_Init(); // 初始化DWT计时器, 用于不定频模式下的采样周期计算
    if (accelFilterNum.dot(accelFilterNum)) {
        m_accelFilterNum = accelFilterNum; // 用户自定义滤波器参数
    } else {
        m_accelFilterNum.x = 1.929454039488895f; // 大疆官方例程滤波器参数
        m_accelFilterNum.y = -0.93178349823448126f;
        m_accelFilterNum.z = 0.002329458745586203f;
    }
}

/**
 * @brief 重置Mahony姿态解算相关数据
 * @note 重置积分项、滤波器历史数据, 调用AHRS::reset()重置四元数和欧拉角
 */
void Mahony::reset()
{
    AHRS::reset();
    m_integralFBx   = 0;
    m_integralFBy   = 0;
    m_integralFBz   = 0;
    m_accelFiltered = 0;
    m_motionAccelBodyFrame = 0;
    m_motionAccelEarthFrame = 0;
    for (int i = 0; i < 3; i++) {
        m_accelFilterHistory[i] = 0;
    }
}

/**
 * @brief 初始化AHRS姿态解算, 根据加速度计和磁力计数据初始化四元数
 * @param accel 加速度计数据
 * @param magnet 磁力计数据
 */
void Mahony::init()
{
    // 归一化加速度计数据，得到重力方向单位向量（机体Z轴在大地坐标系中的方向）
    fp32 recipNorm = GSRLMath::invSqrt(m_accel.x * m_accel.x + m_accel.y * m_accel.y + m_accel.z * m_accel.z);
    fp32 ax = m_accel.x * recipNorm;
    fp32 ay = m_accel.y * recipNorm;
    fp32 az = m_accel.z * recipNorm;

    // 归一化磁力计数据
    recipNorm = GSRLMath::invSqrt(m_magnet.x * m_magnet.x + m_magnet.y * m_magnet.y + m_magnet.z * m_magnet.z);
    fp32 mx = m_magnet.x * recipNorm;
    fp32 my = m_magnet.y * recipNorm;
    fp32 mz = m_magnet.z * recipNorm;

    // 构建旋转矩阵 (从大地坐标系到机体坐标系的转换矩阵)
    fp32 R[9]; // 按行优先存储: R[0]=R11, R[1]=R12, R[2]=R13, R[3]=R21, ...

    // 计算东向(East)向量: east = mag × accel (在水平面上，指向东方)
    fp32 ex = my * az - mz * ay;
    fp32 ey = mz * ax - mx * az;
    fp32 ez = mx * ay - my * ax;
    
    // 归一化东向向量
    recipNorm = GSRLMath::invSqrt(ex * ex + ey * ey + ez * ez);
    ex *= recipNorm;
    ey *= recipNorm;
    ez *= recipNorm;

    // 计算北向(North)向量: north = accel × east
    fp32 nx = ay * ez - az * ey;
    fp32 ny = az * ex - ax * ez;
    fp32 nz = ax * ey - ay * ex;

    // 构建旋转矩阵 (NED到Body的转换矩阵)
    // 第一行: 北向在机体坐标系中的分量
    R[0] = nx;
    R[1] = ny;
    R[2] = nz;
    // 第二行: 东向在机体坐标系中的分量
    R[3] = ex;
    R[4] = ey;
    R[5] = ez;
    // 第三行: 地向(Down)在机体坐标系中的分量
    R[6] = ax;
    R[7] = ay;
    R[8] = az;

    // ---- 使用Shepperd方法从旋转矩阵计算四元数 ----
    
    fp32 trace = R[0] + R[4] + R[8]; // 矩阵的迹
    
    if (trace > 0.0f) {
        // w是最大的分量
        fp32 s = sqrtf(trace + 1.0f);
        m_quaternion[0] = s * 0.5f;
        s = 0.5f / s;
        m_quaternion[1] = (R[7] - R[5]) * s;
        m_quaternion[2] = (R[2] - R[6]) * s;
        m_quaternion[3] = (R[3] - R[1]) * s;
    } else if ((R[0] >= R[4]) && (R[0] >= R[8])) {
        // x是最大的分量
        fp32 s = sqrtf(1.0f + R[0] - R[4] - R[8]);
        m_quaternion[1] = s * 0.5f;
        s = 0.5f / s;
        m_quaternion[0] = (R[7] - R[5]) * s;
        m_quaternion[2] = (R[1] + R[3]) * s;
        m_quaternion[3] = (R[2] + R[6]) * s;
    } else if (R[4] > R[8]) {
        // y是最大的分量
        fp32 s = sqrtf(1.0f + R[4] - R[0] - R[8]);
        m_quaternion[2] = s * 0.5f;
        s = 0.5f / s;
        m_quaternion[0] = (R[2] - R[6]) * s;
        m_quaternion[1] = (R[1] + R[3]) * s;
        m_quaternion[3] = (R[5] + R[7]) * s;
    } else {
        // z是最大的分量
        fp32 s = sqrtf(1.0f + R[8] - R[0] - R[4]);
        m_quaternion[3] = s * 0.5f;
        s = 0.5f / s;
        m_quaternion[0] = (R[3] - R[1]) * s;
        m_quaternion[1] = (R[2] + R[6]) * s;
        m_quaternion[2] = (R[5] + R[7]) * s;
    }

    // 归一化四元数，保证数值稳定性
    recipNorm = GSRLMath::invSqrt(m_quaternion[0] * m_quaternion[0] +
                                   m_quaternion[1] * m_quaternion[1] +
                                   m_quaternion[2] * m_quaternion[2] +
                                   m_quaternion[3] * m_quaternion[3]);
    m_quaternion[0] *= recipNorm;
    m_quaternion[1] *= recipNorm;
    m_quaternion[2] *= recipNorm;
    m_quaternion[3] *= recipNorm;

    // 从四元数计算欧拉角
    convertQuaternionToEulerAngle();

    // 初始化DWT计数器时间戳
    if (m_sampleFreq == 0.0f && DWT_IsInit()) {
        m_lastUpdateTimestamp = DWT_GetTimestamp();
    }
}

/**
 * @brief Mahony姿态解算数据处理
 * @note 对加速度进行滤波并计算采样周期后调用Mahony姿态解算核心函数
 */
void Mahony::dataProcess()
{
    // 加速度低通滤波
    filterAccel();
    // 计算采样周期
    if (m_sampleFreq) {
        m_deltaTime = 1.0f / m_sampleFreq; // 定频运行模式
    } else {
        m_deltaTime = DWT_GetDeltaTime(&m_lastUpdateTimestamp); // 不定频运行模式
        GSRLMath::constrain(m_deltaTime, 0.1f);   // 限制采样周期在0.1s以内
    }
    // 选择姿态解算算法, 在磁力计数据无效时使用六轴融合算法
    if ((m_magnet.x == 0.0f) && (m_magnet.y == 0.0f) && (m_magnet.z == 0.0f)) {
        Mahony::sixAxisProcess(m_gyro.x, m_gyro.y, m_gyro.z, m_accelFiltered.x, m_accelFiltered.y, m_accelFiltered.z);
    } else {
        Mahony::nineAxisProcess(m_gyro.x, m_gyro.y, m_gyro.z, m_accelFiltered.x, m_accelFiltered.y, m_accelFiltered.z, m_magnet.x, m_magnet.y, m_magnet.z);
    }
    // 计算运动加速度
    calculateMotionAccel();
}

/**
 * @brief Mahony六轴姿态解算核心函数
 * @param gx 陀螺仪x轴数据
 * @param gy 陀螺仪y轴数据
 * @param gz 陀螺仪z轴数据
 * @param ax 加速度计x轴数据
 * @param ay 加速度计y轴数据
 * @param az 加速度计z轴数据
 * @note 修改自Mahony官方算法https://x-io.co.uk/open-source-imu-and-ahrs-algorithms
 */
void Mahony::sixAxisProcess(fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az)
{
    fp32 recipNorm;
    fp32 halfvx, halfvy, halfvz;
    fp32 halfex, halfey, halfez;
    fp32 qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    // 只在加速度计数据有效时才进行运算
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        // 将加速度计得到的实际重力加速度向量v单位化
        recipNorm = GSRLMath::invSqrt(ax * ax + ay * ay + az * az); // 该函数返回平方根的倒数
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        // 通过四元数得到理论重力加速度向量g
        // 注意，这里实际上是矩阵第三列*1/2，在开头对Kp Ki的宏定义均为2*增益
        // 这样处理目的是减少乘法运算量
        halfvx = m_quaternion[1] * m_quaternion[3] - m_quaternion[0] * m_quaternion[2];
        halfvy = m_quaternion[0] * m_quaternion[1] + m_quaternion[2] * m_quaternion[3];
        halfvz = m_quaternion[0] * m_quaternion[0] - 0.5f + m_quaternion[3] * m_quaternion[3];

        // Error is sum of cross product between estimated and measured direction of gravity
        // 对实际重力加速度向量v与理论重力加速度向量g做外积
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        // 在PI补偿器中积分项使能情况下计算并应用积分项
        if (m_twoKi > 0.0f) {
            // integral error scaled by Ki
            // 积分过程
            m_integralFBx += m_twoKi * halfex * m_deltaTime;
            m_integralFBy += m_twoKi * halfey * m_deltaTime;
            m_integralFBz += m_twoKi * halfez * m_deltaTime;

            // apply integral feedback
            // 应用误差补偿中的积分项
            gx += m_integralFBx;
            gy += m_integralFBy;
            gz += m_integralFBz;
        } else {
            // prevent integral windup
            // 避免为负值的Ki时积分异常饱和
            m_integralFBx = 0.0f;
            m_integralFBy = 0.0f;
            m_integralFBz = 0.0f;
        }

        // Apply proportional feedback
        // 应用误差补偿中的比例项
        gx += m_twoKp * halfex;
        gy += m_twoKp * halfey;
        gz += m_twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    // 微分方程迭代求解
    gx *= (0.5f * m_deltaTime); // pre-multiply common factors
    gy *= (0.5f * m_deltaTime);
    gz *= (0.5f * m_deltaTime);
    qa = m_quaternion[0];
    qb = m_quaternion[1];
    qc = m_quaternion[2];
    m_quaternion[0] += (-qb * gx - qc * gy - m_quaternion[3] * gz);
    m_quaternion[1] += (qa * gx + qc * gz - m_quaternion[3] * gy);
    m_quaternion[2] += (qa * gy - qb * gz + m_quaternion[3] * gx);
    m_quaternion[3] += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    // 单位化四元数 保证四元数在迭代过程中保持单位性质
    recipNorm = GSRLMath::invSqrt(m_quaternion[0] * m_quaternion[0] + m_quaternion[1] * m_quaternion[1] + m_quaternion[2] * m_quaternion[2] + m_quaternion[3] * m_quaternion[3]);
    m_quaternion[0] *= recipNorm;
    m_quaternion[1] *= recipNorm;
    m_quaternion[2] *= recipNorm;
    m_quaternion[3] *= recipNorm;
}

/**
 * @brief Mahony九轴姿态解算核心函数
 * @param gx 陀螺仪x轴数据
 * @param gy 陀螺仪y轴数据
 * @param gz 陀螺仪z轴数据
 * @param ax 加速度计x轴数据
 * @param ay 加速度计y轴数据
 * @param az 加速度计z轴数据
 * @param mx 磁力计x轴数据
 * @param my 磁力计y轴数据
 * @param mz 磁力计z轴数据
 * @note 修改自Mahony官方算法https://x-io.co.uk/open-source-imu-and-ahrs-algorithms
 */
void Mahony::nineAxisProcess(fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az, fp32 mx, fp32 my, fp32 mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, hz, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    // 只在加速度计数据有效时才进行运算
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        // 将加速度计得到的实际重力加速度向量v单位化
        recipNorm = GSRLMath::invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        // 将磁力计得到的实际磁场向量m单位化
        recipNorm = GSRLMath::invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        // 辅助变量，以避免重复运算
        q0q0 = m_quaternion[0] * m_quaternion[0];
        q0q1 = m_quaternion[0] * m_quaternion[1];
        q0q2 = m_quaternion[0] * m_quaternion[2];
        q0q3 = m_quaternion[0] * m_quaternion[3];
        q1q1 = m_quaternion[1] * m_quaternion[1];
        q1q2 = m_quaternion[1] * m_quaternion[2];
        q1q3 = m_quaternion[1] * m_quaternion[3];
        q2q2 = m_quaternion[2] * m_quaternion[2];
        q2q3 = m_quaternion[2] * m_quaternion[3];
        q3q3 = m_quaternion[3] * m_quaternion[3];

        // Reference direction of Earth's magnetic field
        // 通过磁力计测量值与坐标转换矩阵得到大地坐标系下的理论地磁向量
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        bx = sqrt(hx * hx + hy * hy);
        bz = hz;

        // Estimated direction of gravity and magnetic field
        // 将理论重力加速度向量与理论地磁向量变换至机体坐标系
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        // 通过向量外积得到重力加速度向量和地磁向量的实际值与测量值之间误差
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        // 在PI补偿器中积分项使能情况下计算并应用积分项
        if (m_twoKi > 0.0f) {
            // integral error scaled by Ki
            // 积分过程
            m_integralFBx += m_twoKi * halfex * m_deltaTime;
            m_integralFBy += m_twoKi * halfey * m_deltaTime;
            m_integralFBz += m_twoKi * halfez * m_deltaTime;

            // apply integral feedback
            // 应用误差补偿中的积分项
            gx += m_integralFBx;
            gy += m_integralFBy;
            gz += m_integralFBz;
        } else {
            // prevent integral windup
            // 避免为负值的Ki时积分异常饱和
            m_integralFBx = 0.0f;
            m_integralFBy = 0.0f;
            m_integralFBz = 0.0f;
        }

        // Apply proportional feedback
        // 应用误差补偿中的比例项
        gx += m_twoKp * halfex;
        gy += m_twoKp * halfey;
        gz += m_twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    // 微分方程迭代求解
    gx *= (0.5f * m_deltaTime); // pre-multiply common factors
    gy *= (0.5f * m_deltaTime);
    gz *= (0.5f * m_deltaTime);
    qa = m_quaternion[0];
    qb = m_quaternion[1];
    qc = m_quaternion[2];
    m_quaternion[0] += (-qb * gx - qc * gy - m_quaternion[3] * gz);
    m_quaternion[1] += (qa * gx + qc * gz - m_quaternion[3] * gy);
    m_quaternion[2] += (qa * gy - qb * gz + m_quaternion[3] * gx);
    m_quaternion[3] += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    // 单位化四元数 保证四元数在迭代过程中保持单位性质
    recipNorm = GSRLMath::invSqrt(m_quaternion[0] * m_quaternion[0] + m_quaternion[1] * m_quaternion[1] + m_quaternion[2] * m_quaternion[2] + m_quaternion[3] * m_quaternion[3]);
    m_quaternion[0] *= recipNorm;
    m_quaternion[1] *= recipNorm;
    m_quaternion[2] *= recipNorm;
    m_quaternion[3] *= recipNorm;
}

/**
 * @brief 加速度计数据滤波
 * @note 使用IIR低通滤波器对加速度计数据进行滤波
 */
void Mahony::filterAccel()
{
    // 滚动保存滤波历史数据
    m_accelFilterHistory[0] = m_accelFilterHistory[1];
    m_accelFilterHistory[1] = m_accelFilterHistory[2];
    // x轴滤波
    m_accelFilterHistory[2].x = m_accelFilterHistory[1].x * m_accelFilterNum.x + m_accelFilterHistory[0].x * m_accelFilterNum.y + m_accel.x * m_accelFilterNum.z;
    // y轴滤波
    m_accelFilterHistory[2].y = m_accelFilterHistory[1].y * m_accelFilterNum.x + m_accelFilterHistory[0].y * m_accelFilterNum.y + m_accel.y * m_accelFilterNum.z;
    // z轴滤波
    m_accelFilterHistory[2].z = m_accelFilterHistory[1].z * m_accelFilterNum.x + m_accelFilterHistory[0].z * m_accelFilterNum.y + m_accel.z * m_accelFilterNum.z;
    // 更新滤波后的加速度计数据
    m_accelFiltered = m_accelFilterHistory[2];
}

/**
 * @brief 获取滤波后的加速度计数据
 * @return const Vector3f& 滤波后的加速度计数据
 */
const Mahony::Vector3f &Mahony::getAccel() const
{
    return m_accelFiltered;
}

/**
 * @brief 获取机体坐标系下的运动加速度
 * @return const Vector3f& 机体坐标系下的运动加速度
 */
const Mahony::Vector3f &Mahony::getMotionAccelBodyFrame() const
{
    return m_motionAccelBodyFrame;
}

/**
 * @brief 获取大地坐标系下的运动加速度
 * @return const Vector3f& 大地坐标系下的运动加速度
 */
const Mahony::Vector3f &Mahony::getMotionAccelEarthFrame() const
{
    return m_motionAccelEarthFrame;
}

/**
 * @brief 计算运动加速度
 * @note 从滤波后的加速度计数据中减去重力加速度分量,得到机体坐标系下的运动加速度
 *       然后通过四元数旋转矩阵将其转换到大地坐标系
 */
void Mahony::calculateMotionAccel()
{
    // 提取四元数分量
    fp32 q0 = m_quaternion[0];
    fp32 q1 = m_quaternion[1];
    fp32 q2 = m_quaternion[2];
    fp32 q3 = m_quaternion[3];

    // 计算重力加速度在机体坐标系下的分量(单位向量)
    // 这是通过四元数旋转矩阵的第三列得到的(Z轴方向)
    fp32 gx = 2.0f * (q1 * q3 - q0 * q2);
    fp32 gy = 2.0f * (q0 * q1 + q2 * q3);
    fp32 gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 从滤波后的加速度计数据中减去重力分量,得到机体坐标系下的运动加速度
    // 加速度计测量值包含了重力和运动加速度
    // 假设重力加速度为9.8m/s^2
    constexpr fp32 GRAVITY = 9.8f;
    m_motionAccelBodyFrame.x = m_accelFiltered.x - gx * GRAVITY;
    m_motionAccelBodyFrame.y = m_accelFiltered.y - gy * GRAVITY;
    m_motionAccelBodyFrame.z = m_accelFiltered.z - gz * GRAVITY;

    // 将机体坐标系下的运动加速度转换到大地坐标系
    // 使用四元数旋转矩阵进行坐标变换
    m_motionAccelEarthFrame.x = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * m_motionAccelBodyFrame.x +
                                 2.0f * (q1 * q2 - q0 * q3) * m_motionAccelBodyFrame.y +
                                 2.0f * (q1 * q3 + q0 * q2) * m_motionAccelBodyFrame.z;

    m_motionAccelEarthFrame.y = 2.0f * (q1 * q2 + q0 * q3) * m_motionAccelBodyFrame.x +
                                 (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * m_motionAccelBodyFrame.y +
                                 2.0f * (q2 * q3 - q0 * q1) * m_motionAccelBodyFrame.z;

    m_motionAccelEarthFrame.z = 2.0f * (q1 * q3 - q0 * q2) * m_motionAccelBodyFrame.x +
                                 2.0f * (q2 * q3 + q0 * q1) * m_motionAccelBodyFrame.y +
                                 (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * m_motionAccelBodyFrame.z;
}
