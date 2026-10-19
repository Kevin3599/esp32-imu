#include "mpu6050.h"
#include <math.h>

// 全局对象实例化
Adafruit_MPU6050 mpu;
MPU6050_DATA mpu6050_data;

// 高震动滤波参数
#define FILTER_SAMPLES 10  // 滤波样本数
#define LOW_PASS_ALPHA 0.1  // 低通滤波系数 (0.1 = 强滤波)

// 滤波缓存数组
float acc_buffer_x[FILTER_SAMPLES];
float acc_buffer_y[FILTER_SAMPLES];
float acc_buffer_z[FILTER_SAMPLES];
float gyro_buffer_x[FILTER_SAMPLES];
float gyro_buffer_y[FILTER_SAMPLES];
float gyro_buffer_z[FILTER_SAMPLES];
int buffer_index = 0;
bool buffer_full = false;

// 低通滤波器变量
float acc_x_filtered_prev = 0.0;
float acc_y_filtered_prev = 0.0;
float acc_z_filtered_prev = 0.0;
float gyro_x_filtered_prev = 0.0;
float gyro_y_filtered_prev = 0.0;
float gyro_z_filtered_prev = 0.0;

void InitHighVibrationMode()
{
  // 初始化缓存数组
  for(int i = 0; i < FILTER_SAMPLES; i++) {
    acc_buffer_x[i] = 0.0;
    acc_buffer_y[i] = 0.0;
    acc_buffer_z[i] = 0.0;
    gyro_buffer_x[i] = 0.0;
    gyro_buffer_y[i] = 0.0;
    gyro_buffer_z[i] = 0.0;
  }
  
  Serial.println("⚙️ 高震动模式已启用");
  Serial.printf("📊 滤波参数: 样本数=%d, 低通系数=%.2f\n", FILTER_SAMPLES, LOW_PASS_ALPHA);
}

void Init_mpu6050()
{
  Serial.println("尝试初始化MPU6050...");
  
  // 使用I2C0总线初始化MPU6050
  if (!mpu.begin(0x68, &I2C_MPU)) {
    Serial.println("❌ Failed to find MPU6050 chip on I2C0");
    Serial.println("可能的问题：");
    Serial.println("  1. MPU6050未正确连接到I2C0 (GPIO21/22)");
    Serial.println("  2. I2C地址不正确 (应为0x68或0x69)");
    Serial.println("  3. 供电问题");
    Serial.println("  4. I2C总线接线错误");
    
    // 尝试不同的I2C地址
    Serial.println("尝试不同的I2C地址...");
    if (!mpu.begin(0x69, &I2C_MPU)) {  // 尝试备用地址
      Serial.println("❌ 在0x69地址也找不到MPU6050");
      Serial.println("⚠️  系统将继续运行但没有MPU6050数据");
      return;
    } else {
      Serial.println("✅ 在地址0x69找到MPU6050！");
    }
  } else {
    Serial.println("✅ MPU6050 Found at default address 0x68 on I2C0!");
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
      MPU6050_BAND_5_HZ   - 最强滤波 (高震动环境推荐)
      MPU6050_BAND_10_HZ  - 强滤波
      MPU6050_BAND_21_HZ  - 中等滤波
      MPU6050_BAND_44_HZ
      MPU6050_BAND_94_HZ
      MPU6050_BAND_184_HZ
      MPU6050_BAND_260_HZ

    API:
      getFilterBandwidth()
      setFilterBandwidth()
  ***********************************************/
  // Dragy模式：使用94Hz带宽，支持~200Hz采样率用于GPS-IMU融合
  // 软件滤波会处理噪声，硬件需要保持足够的响应速度
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  
  Serial.println("✅ MPU6050 configuration completed (Dragy Mode - 94Hz BW)");
  
  // 初始化高震动滤波
  InitHighVibrationMode();
}

void ReadMPU6050()
{
  // 获取新的传感器事件和读数
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 存储温度数据
  mpu6050_data.Temperature = temp.temperature;
  
  // 存储原始加速度数据 (m/s^2)
  mpu6050_data.Acc_X = a.acceleration.x;
  mpu6050_data.Acc_Y = a.acceleration.y;
  mpu6050_data.Acc_Z = a.acceleration.z;

  // 存储原始角速度数据 (rad/s)
  mpu6050_data.Angle_Velocity_R = g.gyro.x; 
  mpu6050_data.Angle_Velocity_P = g.gyro.y; 
  mpu6050_data.Angle_Velocity_Y = g.gyro.z;
  
  // 应用高震动环境滤波
  FilterDataForVibration();
  
  // 使用滤波后的数据计算姿态角
  CalculateAttitude();
}

void FilterDataForVibration()
{
  // 1. 滑动平均滤波
  acc_buffer_x[buffer_index] = mpu6050_data.Acc_X;
  acc_buffer_y[buffer_index] = mpu6050_data.Acc_Y;
  acc_buffer_z[buffer_index] = mpu6050_data.Acc_Z;
  gyro_buffer_x[buffer_index] = mpu6050_data.Angle_Velocity_R;
  gyro_buffer_y[buffer_index] = mpu6050_data.Angle_Velocity_P;
  gyro_buffer_z[buffer_index] = mpu6050_data.Angle_Velocity_Y;
  
  buffer_index = (buffer_index + 1) % FILTER_SAMPLES;
  if (buffer_index == 0) buffer_full = true;
  
  // 计算滑动平均
  int samples_to_use = buffer_full ? FILTER_SAMPLES : buffer_index;
  float acc_x_avg = 0, acc_y_avg = 0, acc_z_avg = 0;
  float gyro_x_avg = 0, gyro_y_avg = 0, gyro_z_avg = 0;
  
  for(int i = 0; i < samples_to_use; i++) {
    acc_x_avg += acc_buffer_x[i];
    acc_y_avg += acc_buffer_y[i];
    acc_z_avg += acc_buffer_z[i];
    gyro_x_avg += gyro_buffer_x[i];
    gyro_y_avg += gyro_buffer_y[i];
    gyro_z_avg += gyro_buffer_z[i];
  }
  
  acc_x_avg /= samples_to_use;
  acc_y_avg /= samples_to_use;
  acc_z_avg /= samples_to_use;
  gyro_x_avg /= samples_to_use;
  gyro_y_avg /= samples_to_use;
  gyro_z_avg /= samples_to_use;
  
  // 2. 一阶低通滤波器 (进一步平滑)
  mpu6050_data.Acc_X_Filtered = LOW_PASS_ALPHA * acc_x_avg + (1.0 - LOW_PASS_ALPHA) * acc_x_filtered_prev;
  mpu6050_data.Acc_Y_Filtered = LOW_PASS_ALPHA * acc_y_avg + (1.0 - LOW_PASS_ALPHA) * acc_y_filtered_prev;
  mpu6050_data.Acc_Z_Filtered = LOW_PASS_ALPHA * acc_z_avg + (1.0 - LOW_PASS_ALPHA) * acc_z_filtered_prev;
  
  mpu6050_data.Gyro_X_Filtered = LOW_PASS_ALPHA * gyro_x_avg + (1.0 - LOW_PASS_ALPHA) * gyro_x_filtered_prev;
  mpu6050_data.Gyro_Y_Filtered = LOW_PASS_ALPHA * gyro_y_avg + (1.0 - LOW_PASS_ALPHA) * gyro_y_filtered_prev;
  mpu6050_data.Gyro_Z_Filtered = LOW_PASS_ALPHA * gyro_z_avg + (1.0 - LOW_PASS_ALPHA) * gyro_z_filtered_prev;
  
  // 更新前一次的值
  acc_x_filtered_prev = mpu6050_data.Acc_X_Filtered;
  acc_y_filtered_prev = mpu6050_data.Acc_Y_Filtered;
  acc_z_filtered_prev = mpu6050_data.Acc_Z_Filtered;
  gyro_x_filtered_prev = mpu6050_data.Gyro_X_Filtered;
  gyro_y_filtered_prev = mpu6050_data.Gyro_Y_Filtered;
  gyro_z_filtered_prev = mpu6050_data.Gyro_Z_Filtered;
}

void CalculateAttitude()
{
  // 使用滤波后的数据计算Roll和Pitch角度 (单位：度)
  // Roll (绕X轴旋转) - 使用滤波数据
  mpu6050_data.Roll = atan2(mpu6050_data.Acc_Y, mpu6050_data.Acc_Z) * 180.0 / PI;
  mpu6050_data.Roll_Filtered = atan2(mpu6050_data.Acc_Y_Filtered, mpu6050_data.Acc_Z_Filtered) * 180.0 / PI;
  
  // Pitch (绕Y轴旋转) - 使用滤波数据
  mpu6050_data.Pitch = atan2(-mpu6050_data.Acc_X, 
                            sqrt(mpu6050_data.Acc_Y * mpu6050_data.Acc_Y + 
                                 mpu6050_data.Acc_Z * mpu6050_data.Acc_Z)) * 180.0 / PI;
  
  mpu6050_data.Pitch_Filtered = atan2(-mpu6050_data.Acc_X_Filtered, 
                                    sqrt(mpu6050_data.Acc_Y_Filtered * mpu6050_data.Acc_Y_Filtered + 
                                         mpu6050_data.Acc_Z_Filtered * mpu6050_data.Acc_Z_Filtered)) * 180.0 / PI;
  
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