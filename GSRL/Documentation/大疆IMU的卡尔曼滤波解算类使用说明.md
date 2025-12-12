# 大疆C板的IMU卡尔曼解算使用说明
#####作者：王家豪
#####最终更新时间2025.12.11

##基本说明
#### 本算法基于扩展卡尔曼滤波（EKF），以四元数作为核心姿态表示，对 BMI088 输出的六轴原始数据（陀螺仪与加速度计）进行融合处理。同时提供可选的卡方检验机制：当检测到当前周期内的加速度超出可信范围时，将自动屏蔽加速度量测输入，从而避免高动态环境下因线加速度过大而导致的观测误差，确保姿态解算过程以陀螺仪预测为主，保持计算的稳定性和精度。

#### 相比于传统Mahony算法，本EKF方法在高速运动、高加速度等复杂工况下能够提供更稳定、更准确的姿态估计。最终结果可通过读取欧拉角或四元数获得。

###代码测试环境
####大疆c型开发板，BMI088，基于hal库的GMaster代码库GSRL

###代码位置
####GSRL/Algorithm/src/alg_ahrs.cpp （上层陀螺仪适配卡尔曼算法类实现）
####GSRL/Algorithm/inc/alg_filter.hpp  (下层卡尔曼解算类实现)

###注意事项
####本算法进行了大量线性代数运算，对栈内存要求较高，使用ROTS分配task时，请务必注意当前task是否分配了足够的栈内存，GSRL库内的默认栈内存分配仅为0.5KB，这会导致栈溢出，使代码在循环中卡死。本次测试内存分配为2KB。

###典型实现代码
//初始化卡尔曼算法
Kalman_Quaternion_EKF ahrs(
    0.0f, // 给定刷新频率，给0则为自动识别频率
    10.0f,  // Q的四元数过程噪声基准
    0.001f, //Q的零偏过程噪声基准
    1e7f,   //R，线加速度计噪声基准
    0.0f,   //加速度计一阶低通滤波系数（0~1，越小滤得越重），默认0
    1      //是否使用卡方检验（卡方检验阈值请使用类内函数接口来设定）
);

//初始化IMU
BMI088::CalibrationInfo cali = {
    {0.0f, 0.0f, 0.0f}, // gyroOffset
    {0.0f, 0.0f, 0.0f}, // accelOffset
    {0.0f, 0.0f, 0.0f},  // magnetOffset
    {GSRLMath::Matrix33f::MatrixType::IDENTITY}
};

//初始化IMU
BMI088 imu(&ahrs, {&hspi1, GPIOA, GPIO_PIN_4}, {&hspi1, GPIOB, GPIO_PIN_0}, cali);

while(1)
{
GSRLMath::Vector3f eulerAngle = imu.solveAttitude();
}


###参考/学习文件
####[1]四元数EKF姿态更新算法 https://zhuanlan.zhihu.com/p/454155643
####[2]四元数解算姿态角解析 https://blog.csdn.net/guanjianhe/article/details/95608801
####[3]惯导姿态解算项目 https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example