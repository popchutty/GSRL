# MG 电机驱动模块使用说明以及实例代码

作者：王家豪
版本：v1.0.0  
最后更新日期：2025-11-30  

---

## 1：概述
   本MG电机驱动基于GSRL库，实现了速度闭环，角度闭环控制等功能

   模块基于C++封装，包含：
   -电机驱动类（MGmotor）
   -CAN通信解析与打包逻辑


## 2：功能特性
  本驱动可实现：
   -MG电机的电流开环控制
   -MG电机的自带角速度闭环控制
   -MG电机的自带单圈角度闭环控制
   -MG电机的自带多圈角度闭环控制  
   -实时获取MG电机通过CAN返回的运行状态，包括：
         -电机温度
         -转矩电流
         -输出功率
         -电机速度
         -编码器位置

  特性：
    -上述的四种控制模式在一个电机上只能同时使用一种（应该不会有人在闭环控制角速度的同时闭环控制角度吧），因为其中有参数是共用的
    -如果使用电机自带的闭环控制，会在控制量比较低的时候出现轻微的电机抖动，可通过专用上位机对电机的内置PID参数进行调整，以达到最佳控制效果
    -要想要获得更好的控制效果，建议使用GSRL库里面自带的控制器来直接控制扭矩


## 3：适用环境
   -MCU：STM32系列
   -开发环境：---
   -依赖库：
    -STM32 HAL ‘drv_can.hpp’
    -自定义库：‘dvc_motor.hpp’ ‘alg_pid.hpp’
   -通信环境：CAN


## 4：典型用法实例

    #include ..
    #include ..

    SimplePID::PIDParam param = {
    100,  // Kp
    10.0f,   // Ki
    10.0f, // Kd
    10000.0f,  // outputLimit
    10000.0f    // integralLimit };

    // 创建 PID 控制器实例
    SimplePID myPID(SimplePID::PID_POSITION, param);
    
    // MG 电机实例
    MotorMGDM4310 mgMotor(1, &myPID, 0);

    extern "C" void can1RxCallback(can_rx_message_t *pRxMsg);
    inline void transmitMotorsControlData();

    int main()
    {
      CAN_Init(&hcan1, can1RxCallback);
      while (1) {
        // mgMotor.angularVelocityClosedloopControl(1);    
        // mgMotor.hardwareConvertAngularVelocityToMotorContorlData(1); // 角速度闭环控制
        // mgMotor.hardwareConvertSingleCircleAngleToMotorControlData(0, 1.0f, false); // 单圈角度闭环控制
        // mgMotor.hardwareConvertMutipleCircleAngleToMotorControlData(0, 1);
        
        // 传输电机控制数据
        transmitMotorsControlData();}
    }


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
