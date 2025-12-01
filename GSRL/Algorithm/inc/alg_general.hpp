/**
 ******************************************************************************
 * @file           : alg_general.hpp
 * @brief          : header file for alg_general.cpp
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
#include <math.h>
#include <string.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
namespace GSRLMath
{
    /**
     * @brief 快速平方根倒数
     * @param x 输入值
     * @return 平方根倒数
     */
    inline fp32 invSqrt(fp32 x)
    {
        fp32 halfx = 0.5f * x;
        fp32 y     = x;
        int32_t i  = *(int32_t *)&y;
        i          = 0x5f3759df - (i >> 1);
        y          = *(fp32 *)&i;
        y          = y * (1.5f - (halfx * y * y));
        return y;
    }

    /**
     * @brief 限幅, 将num限制在-limit到limit之间
     * @param num 输入值
     * @param limit 限制值（应为正数）
     * @tparam Tp 数据类型
     */
    template <typename Tp>
    constexpr inline void constrain(Tp &num, Tp limit)
    {
        if (num > limit) {
            num = limit;
        } else if (num < -limit) {
            num = -limit;
        }
    }

    /**
     * @brief 浮点数转换为无符号整数函数
     * @param x_float 待转换的浮点数
     * @param x_min 范围最小值
     * @param x_max 范围最大值
     * @param bits 目标无符号整数的位数
     * @retval 无符号整数结果
     * @details 将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
     */
    constexpr inline int convertFloatToUint(float x_float, float x_min, float x_max, int bits)
    {
        /* Converts a float to an unsigned int, given range and number of bits */
        float span   = x_max - x_min;
        float offset = x_min;
        return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
    }
    /**
    * @brief 无符号整数转换为浮点数函数
    * @param x_int 待转换的无符号整数
    * @param x_min 范围最小值
    * @param x_max 范围最大值
    * @param bits 无符号整数的位数
    * @retval 浮点数结果
    * @details 将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
    */
    constexpr inline float convertUintToFloat(int x_int, float x_min, float x_max, int bits)
    {
        /* converts unsigned int to float, given range and number of bits */
        float span   = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

    /**
     * @brief 绝对角度归一化
     * @param angle 待归一化的角度
     * @retval 归一化后的绝对角度[0, 2PI)
     */
    inline fp32 normalizeAngle(fp32 angle)
    {
        angle = fmodf(angle, 2 * MATH_PI);
        if (angle < 0.0f) {
            angle += 2 * MATH_PI;
        }
        return angle;
    }

    /**
     * @brief 角度差归一化
     * @param deltaAngle 待归一化的角度差
     * @retval 归一化后的角度差[-PI, PI)
     */
    inline fp32 normalizeDeltaAngle(fp32 deltaAngle)
    {
        deltaAngle = fmodf(deltaAngle, 2 * MATH_PI);
        if (deltaAngle >= MATH_PI) {
            deltaAngle -= 2 * MATH_PI;
        } else if (deltaAngle < -MATH_PI) {
            deltaAngle += 2 * MATH_PI;
        }
        return deltaAngle;
    }

    /**
     * @brief 三维向量
     * @tparam Tp 数据类型
     */
    template <typename Tp>
    class Vector3
    {
    public:
        // 向量元素
        Tp x, y, z;
        // 默认构造函数
        Vector3()
            : x(0), y(0), z(0)
        {
        }
        // 统一赋值构造函数
        Vector3(Tp value)
            : x(value), y(value), z(value)
        {
        }
        // 各元素初始化构造函数
        Vector3(Tp x, Tp y, Tp z)
            : x(x), y(y), z(z)
        {
        }
        // 标量统一赋值
        Vector3 &operator=(const Tp &value)
        {
            x = value;
            y = value;
            z = value;
            return *this;
        }
        // 向量加法
        Vector3 operator+(const Vector3 &otherVector) const
        {
            return Vector3(x + otherVector.x, y + otherVector.y, z + otherVector.z);
        }
        // 向量减法
        Vector3 operator-(const Vector3 &otherVector) const
        {
            return Vector3(x - otherVector.x, y - otherVector.y, z - otherVector.z);
        }
        // 数乘
        Vector3 operator*(const Tp &scalar) const
        {
            return Vector3(x * scalar, y * scalar, z * scalar);
        }
        // 数除
        Vector3 operator/(const Tp &scalar) const
        {
            return Vector3(x / scalar, y / scalar, z / scalar);
        }
        // 点积
        Tp dot(const Vector3 &otherVector) const
        {
            return x * otherVector.x + y * otherVector.y + z * otherVector.z;
        }
        // 叉积
        Vector3 cross(const Vector3 &otherVector) const
        {
            return Vector3(y * otherVector.z - z * otherVector.y,
                           z * otherVector.x - x * otherVector.z,
                           x * otherVector.y - y * otherVector.x);
        }
        // 求模长
        Tp magnitude() const
        {
            return sqrt(x * x + y * y + z * z);
        }
        // 归一化
        Vector3 normalize() const
        {
            Tp mag = magnitude();
            return (mag == 0) ? Vector3(0, 0, 0) : *this / mag;
        }
        // 数组下标访问
        Tp &operator[](int index)
        {
            switch (index) {
                case 0:
                    return x;
                case 1:
                    return y;
                case 2:
                    return z;
                default:
                    return z; // 越界访问
            }
        }
        // 数组下标访问
        const Tp &operator[](int index) const
        {
            switch (index) {
                case 0:
                    return x;
                case 1:
                    return y;
                case 2:
                    return z;
                default:
                    return z; // 越界访问
            }
        }
    };
    // 常用浮点向量定义
    typedef Vector3<fp32> Vector3f;

    /**
     * @brief 三维矩阵
     * @tparam Tp 数据类型
     */
    template <typename Tp>
    class Matrix33
    {
    public:
        // 常用矩阵类型
        enum MatrixType : uint8_t {
            ZERO = 0,
            IDENTITY,
            SCALAR,
            ROTATION
        };

    private:
        // 矩阵元素
        Tp m[3][3];

    public:
        // 默认构造函数
        Matrix33()
        {
            memset(m, 0, sizeof(m));
        }
        // 统一赋值构造函数
        Matrix33(Tp value)
        {
            if (value == 0) {
                memset(m, 0, sizeof(m));
            } else {
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        m[i][j] = value;
                    }
                }
            }
        }
        /**
         * @brief 使用数组初始化构造函数
         * @param init 使用3x3数组初始化矩阵
         */
        Matrix33(const Tp init[3][3])
        {
            memcpy(m, init, sizeof(m));
        }
        /**
         * @brief 无参数矩阵类型构造函数
         * @param type 矩阵类型
         * @note 支持矩阵类型如下:
         * ZERO     - 零矩阵
         * IDENTITY - 单位矩阵
         */
        Matrix33(MatrixType type)
        {
            switch (type) {
                case ZERO:
                    memset(m, 0, sizeof(m));
                    break;
                case IDENTITY:
                    memset(m, 0, sizeof(m));
                    m[0][0] = m[1][1] = m[2][2] = 1;
                    break;
                default:
                    memset(m, 0, sizeof(m));
                    break;
            }
        }
        /**
         * @brief 单参数矩阵类型构造函数
         * @param type 矩阵类型
         * @param para 标量参数
         * @param vec 向量参数(可选)
         * @note 支持矩阵类型及对应参数如下:
         * SCALAR - 标量矩阵 para对角线元素值 vec无效
         * ROTATION - 旋转矩阵 para旋转角度 vec旋转轴向量
         */
        Matrix33(MatrixType type, Tp para, Vector3<Tp> vec = Vector3<Tp>())
        {
            switch (type) {
                case SCALAR:
                    memset(m, 0, sizeof(m));
                    m[0][0] = m[1][1] = m[2][2] = para;
                    break;
                case ROTATION: {
                    Tp c = cos(para);
                    Tp s = sin(para);
                    Tp t = 1 - c;
                    Tp x = vec.x, y = vec.y, z = vec.z;
                    m[0][0] = t * x * x + c;
                    m[0][1] = t * x * y - s * z;
                    m[0][2] = t * x * z + s * y;
                    m[1][0] = t * x * y + s * z;
                    m[1][1] = t * y * y + c;
                    m[1][2] = t * y * z - s * x;
                    m[2][0] = t * x * z - s * y;
                    m[2][1] = t * y * z + s * x;
                    m[2][2] = t * z * z + c;
                    break;
                }
                default:
                    memset(m, 0, sizeof(m));
                    break;
            }
        }
        // 矩阵加法
        Matrix33 operator+(const Matrix33 &otherMatrix) const
        {
            Matrix33 result;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    result.m[i][j] = m[i][j] + otherMatrix.m[i][j];
                }
            }
            return result;
        }
        // 矩阵减法
        Matrix33 operator-(const Matrix33 &otherMatrix) const
        {
            Matrix33 result;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    result.m[i][j] = m[i][j] - otherMatrix.m[i][j];
                }
            }
            return result;
        }
        // 数乘
        Matrix33 operator*(const Tp &scalar) const
        {
            Matrix33 result;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    result.m[i][j] = m[i][j] * scalar;
                }
            }
            return result;
        }
        // 数除
        Matrix33 operator/(const Tp &scalar) const
        {
            Matrix33 result;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    result.m[i][j] = m[i][j] / scalar;
                }
            }
            return result;
        }
        // 矩阵乘法
        Matrix33 operator*(const Matrix33 &otherMatrix) const
        {
            Matrix33 result;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    result.m[i][j] = m[i][0] * otherMatrix.m[0][j] +
                                     m[i][1] * otherMatrix.m[1][j] +
                                     m[i][2] * otherMatrix.m[2][j];
                }
            }
            return result;
        }
        // 矩阵右乘向量
        Vector3<Tp> operator*(const Vector3<Tp> &vector) const
        {
            return Vector3<Tp>(
                m[0][0] * vector.x + m[0][1] * vector.y + m[0][2] * vector.z,
                m[1][0] * vector.x + m[1][1] * vector.y + m[1][2] * vector.z,
                m[2][0] * vector.x + m[2][1] * vector.y + m[2][2] * vector.z);
        }
        // 矩阵转置
        Matrix33 transpose() const
        {
            Matrix33 result;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    result.m[i][j] = m[j][i];
                }
            }
            return result;
        }
        // 矩阵求逆
        Matrix33 inverse() const
        {
            Matrix33 result;
            // 计算行列式
            Tp det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
            // 如果行列式为0或接近0，矩阵不可逆
            if (fabs(det) < 1e-10) {
                // 返回零矩阵表示不可逆
                return Matrix33(ZERO);
            }
            // 计算伴随矩阵的元素（代数余子式的转置）
            result[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det;
            result[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) / det;
            result[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det;
            result[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) / det;
            result[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det;
            result[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) / det;
            result[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det;
            result[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) / det;
            result[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det;
            return result;
        }
        // 数组下标访问
        Tp *operator[](int index)
        {
            return m[index];
        }
        const Tp *operator[](int index) const
        {
            return m[index];
        }
    };
    // 常用浮点矩阵定义
    typedef Matrix33<fp32> Matrix33f;
}

/* Defines -------------------------------------------------------------------*/
