#ifndef MPU6050_H
#define MPU6050_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// 外部I2C总线声明
extern TwoWire I2C_MPU;  // 使用I2C0总线

// 全局MPU6050对象
extern Adafruit_MPU6050 mpu;

// MPU6050数据结构
struct MPU6050_DATA
{
  // 温度 摄氏度
  double Temperature = 0.0;

  // 姿态角 (这里预留，需要通过加速度计和陀螺仪数据计算)
  double Roll = 0.0;
  double Pitch = 0.0;
  double Yaw = 0.0;

  // 沿三轴的线加速度 米每二次方秒 m/s^2
  double Acc_X = 0.0;
  double Acc_Y = 0.0;
  double Acc_Z = 0.0;

  // 三轴角速度 弧度每秒 rad/s
  double Angle_Velocity_R = 0.0;  // Roll角速度
  double Angle_Velocity_P = 0.0;  // Pitch角速度
  double Angle_Velocity_Y = 0.0;  // Yaw角速度
};

// 全局MPU6050数据对象
extern MPU6050_DATA mpu6050_data;

/**
 * 初始化MPU6050传感器
 * 配置加速度计、陀螺仪范围和滤波器
 */
void Init_mpu6050();

/**
 * 读取MPU6050传感器数据
 * 包括加速度、角速度和温度
 */
void ReadMPU6050();

/**
 * 计算简单的姿态角 (Roll和Pitch)
 * 基于加速度计数据计算倾斜角度
 */
void CalculateAttitude();

/**
 * 实时打印MPU6050数据到串口
 * 包括加速度、角速度、姿态角和温度
 */
void PrintMPU6050Data();

/**
 * 打印MPU6050简要数据到串口
 * 用于快速查看数据变化
 */
void PrintMPU6050DataBrief();

#endif // MPU6050_H