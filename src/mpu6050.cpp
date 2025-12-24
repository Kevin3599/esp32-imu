#include "mpu6050.h"
#include <math.h>

// 全局对象实例化
Adafruit_MPU6050 mpu;
MPU6050_DATA mpu6050_data;

void Init_mpu6050()
{
  Serial.println("尝试初始化MPU6050...");
  
  // 尝试初始化MPU6050
  if (!mpu.begin()) {
    Serial.println("❌ Failed to find MPU6050 chip");
    Serial.println("可能的问题：");
    Serial.println("  1. MPU6050未正确连接");
    Serial.println("  2. I2C地址不正确 (应为0x68或0x69)");
    Serial.println("  3. 供电问题");
    Serial.println("  4. I2C总线接线错误");
    
    // 尝试不同的I2C地址
    Serial.println("尝试不同的I2C地址...");
    if (!mpu.begin(0x69)) {  // 尝试备用地址
      Serial.println("❌ 在0x69地址也找不到MPU6050");
      Serial.println("⚠️  系统将继续运行但没有MPU6050数据");
      return;
    } else {
      Serial.println("✅ 在地址0x69找到MPU6050！");
    }
  } else {
    Serial.println("✅ MPU6050 Found at default address 0x68!");
  }
  
  /***********************************************
    Available AccelerometerRange:
      MPU6050_RANGE_2_G
      MPU6050_RANGE_4_G
      MPU6050_RANGE_8_G
      MPU6050_RANGE_16_G
  
    API:
      getAccelerometerRange()
      setAccelerometerRange()
  ***********************************************/
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  /***********************************************
    Available GyroRange:
      MPU6050_RANGE_250_DEG
      MPU6050_RANGE_500_DEG
      MPU6050_RANGE_1000_DEG
      MPU6050_RANGE_2000_DEG
  
    API:
      getGyroRange()
      setGyroRange()
  ***********************************************/
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  /***********************************************
    Available Bandwidth:
      MPU6050_BAND_5_HZ
      MPU6050_BAND_10_HZ
      MPU6050_BAND_21_HZ
      MPU6050_BAND_44_HZ
      MPU6050_BAND_94_HZ
      MPU6050_BAND_184_HZ
      MPU6050_BAND_260_HZ

    API:
      getFilterBandwidth()
      setFilterBandwidth()
  ***********************************************/
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("MPU6050 configuration completed");
}

void ReadMPU6050()
{
  // 获取新的传感器事件和读数
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 存储温度数据
  mpu6050_data.Temperature = temp.temperature;
  
  // 存储加速度数据 (m/s^2)
  mpu6050_data.Acc_X = a.acceleration.x;
  mpu6050_data.Acc_Y = a.acceleration.y;
  mpu6050_data.Acc_Z = a.acceleration.z;

  // 存储角速度数据 (rad/s)
  mpu6050_data.Angle_Velocity_R = g.gyro.x; 
  mpu6050_data.Angle_Velocity_P = g.gyro.y; 
  mpu6050_data.Angle_Velocity_Y = g.gyro.z;
  
  // 计算姿态角
  CalculateAttitude();
}

void CalculateAttitude()
{
  // 基于加速度计数据计算Roll和Pitch角度 (单位：度)
  // Roll (绕X轴旋转)
  mpu6050_data.Roll = atan2(mpu6050_data.Acc_Y, mpu6050_data.Acc_Z) * 180.0 / PI;
  
  // Pitch (绕Y轴旋转)  
  mpu6050_data.Pitch = atan2(-mpu6050_data.Acc_X, 
                            sqrt(mpu6050_data.Acc_Y * mpu6050_data.Acc_Y + 
                                 mpu6050_data.Acc_Z * mpu6050_data.Acc_Z)) * 180.0 / PI;
  
  // Yaw角度需要磁力计或者通过陀螺仪积分计算，这里暂时设为0
  mpu6050_data.Yaw = 0.0;
}

void PrintMPU6050Data()
{
  Serial.println("=================== MPU6050 实时数据 ===================");
  
  // 加速度数据
  Serial.println("[加速度 m/s²]");
  Serial.printf("  X轴: %8.3f  |  Y轴: %8.3f  |  Z轴: %8.3f\n", 
                mpu6050_data.Acc_X, mpu6050_data.Acc_Y, mpu6050_data.Acc_Z);
  
  // 角速度数据 (转换为度/秒显示)
  Serial.println("[角速度 °/s]");
  Serial.printf("  Roll: %7.2f  | Pitch: %7.2f  |  Yaw: %7.2f\n", 
                mpu6050_data.Angle_Velocity_R * 180.0/PI, 
                mpu6050_data.Angle_Velocity_P * 180.0/PI,
                mpu6050_data.Angle_Velocity_Y * 180.0/PI);
  
  // 姿态角数据
  Serial.println("[姿态角 °]");
  Serial.printf("  Roll: %7.2f  | Pitch: %7.2f  |  Yaw: %7.2f\n", 
                mpu6050_data.Roll, mpu6050_data.Pitch, mpu6050_data.Yaw);
  
  // 温度
  Serial.printf("[温度] %.2f°C\n", mpu6050_data.Temperature);
  
  // 计算合成加速度和运动状态
  float totalAccel = sqrt(mpu6050_data.Acc_X * mpu6050_data.Acc_X + 
                         mpu6050_data.Acc_Y * mpu6050_data.Acc_Y + 
                         mpu6050_data.Acc_Z * mpu6050_data.Acc_Z);
  Serial.printf("[合成加速度] %.3f m/s²\n", totalAccel);
  
  // 运动状态判断
  if (totalAccel > 11.0) {
    Serial.println("[状态] 🔴 剧烈运动");
  } else if (totalAccel > 10.5) {
    Serial.println("[状态] 🟡 轻微运动");
  } else {
    Serial.println("[状态] 🟢 静止状态");
  }
  
  Serial.println("======================================================\n");
}

void PrintMPU6050DataBrief()
{
  // 简洁格式输出 - 一行显示主要数据
  Serial.printf("[IMU] A(%.2f,%.2f,%.2f) G(%.1f,%.1f,%.1f) R/P(%.1f,%.1f) T:%.1f°C\n",
                mpu6050_data.Acc_X, mpu6050_data.Acc_Y, mpu6050_data.Acc_Z,
                mpu6050_data.Angle_Velocity_R * 180.0/PI,
                mpu6050_data.Angle_Velocity_P * 180.0/PI, 
                mpu6050_data.Angle_Velocity_Y * 180.0/PI,
                mpu6050_data.Roll, mpu6050_data.Pitch,
                mpu6050_data.Temperature);
}