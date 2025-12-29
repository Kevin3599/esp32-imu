#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Arduino.h>

// GPS模块引脚定义
#define GPS_RX_PIN 16   // ESP32 RX <- GPS TX
#define GPS_TX_PIN 17   // ESP32 TX -> GPS RX  
#define GPS_BAUD 9600   // GT-U7默认波特率9600

// GPS数据结构
struct GPS_DATA {
    // 基础位置信息
    double latitude = 0.0;      // 纬度
    double longitude = 0.0;     // 经度
    double altitude = 0.0;      // 海拔高度 (米)
    
    // 时间信息
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
    int centisecond = 0;
    
    // 速度和方向
    double speed_kmh = 0.0;     // 速度 (km/h)
    double speed_mps = 0.0;     // 速度 (m/s)
    double course = 0.0;        // 航向角度
    
    // 信号质量 - 增强
    int satellites = 0;         // 卫星数量
    double hdop = 0.0;          // 水平精度稀释
    double vdop = 0.0;          // 垂直精度稀释
    double pdop = 0.0;          // 位置精度稀释
    int fix_quality = 0;        // 定位质量 (0=无效, 1=GPS, 2=DGPS)
    
    // 高精度性能监控
    double speed_accuracy = 0.0;  // 速度精度估计
    double position_accuracy = 0.0; // 位置精度估计
    unsigned long last_fix_time = 0;  // 最后定位时间
    
    // IMU-GPS融合速度数据（Dragy模式）
    double smooth_speed_kmh = 0.0;      // 融合后的顺滑速度
    double fused_speed_ms = 0.0;        // 融合速度 (m/s) - 高精度
    double imu_speed_delta = 0.0;       // IMU积分的速度增量
    bool speed_increasing = false;      // 速度是否在增加  
    double acceleration_ms2 = 0.0;      // 当前加速度估计
    double last_update_time_ms = 0;     // 上次更新时间
    bool imu_calibrated = false;        // IMU是否已校准
    double accel_bias = 0.0;            // 加速度偏置校准值
    int calibration_count = 0;          // 校准样本计数
    double last_gps_speed_ms = 0.0;     // 上次GPS速度 (m/s)
    unsigned long last_gps_time = 0;    // 上次GPS更新时间
    double speed_ms = 0.0;              // 当前速度 (m/s)
    
    // 状态信息
    bool valid_location = false; // 位置有效性
    bool valid_date = false;     // 日期有效性
    bool valid_time = false;     // 时间有效性
    bool valid_speed = false;    // 速度有效性
    
    // 计算得出的性能数据
    double max_speed_session = 0.0;  // 本次会话最高速度
    double distance_traveled = 0.0;   // 行驶距离
    unsigned long fix_age = 0;        // GPS数据年龄(ms)
};

// 性能测量数据结构
struct PERFORMANCE_DATA {
    // 加速测试
    bool accel_test_active = false;
    unsigned long accel_start_time = 0;
    double accel_0_60_time = 0.0;      // 0-60km/h时间
    double accel_0_100_time = 0.0;     // 0-100km/h时间
    double accel_100_200_time = 0.0;   // 100-200km/h时间
    
    // 制动测试
    bool brake_test_active = false;
    unsigned long brake_start_time = 0;
    double brake_start_speed = 0.0;
    double brake_distance = 0.0;       // 制动距离
    double brake_100_0_time = 0.0;     // 100-0km/h时间
    
    // G力记录
    double max_accel_g = 0.0;          // 最大加速G力
    double max_brake_g = 0.0;          // 最大制动G力
    double max_lateral_g = 0.0;        // 最大侧向G力
    
    // 圈速 (需要设定起点/终点)
    bool lap_timing_active = false;
    unsigned long lap_start_time = 0;
    double best_lap_time = 0.0;
    double current_lap_time = 0.0;
    int lap_count = 0;
    
    // 位置记录 (起点/终点坐标)
    double start_lat = 0.0;
    double start_lon = 0.0;
    double finish_lat = 0.0;
    double finish_lon = 0.0;
    
    // 会话信息
    unsigned long session_start_time = 0;
    bool session_active = false;
};

// 全局对象声明
extern TinyGPSPlus gps;
extern HardwareSerial gpsSerial;
extern GPS_DATA gps_data;
extern PERFORMANCE_DATA performance_data;

// 函数声明
bool initGPS();
void updateGPSData();
void printGPSData();
bool isGPSDataValid();
void calculatePerformanceMetrics();
void startAccelTest();
void stopAccelTest();
void startBrakeTest();
void stopBrakeTest();
void startLapTiming();
void stopLapTiming();
void resetPerformanceData();
double calculateDistance(double lat1, double lon1, double lat2, double lon2);
String formatTime(unsigned long milliseconds);
void saveSessionData();
void loadSessionData();

// Dragy模式GPS-IMU融合函数
void fuseIMUSpeed(double accel_x, double accel_y, double dt);
void calibrateIMUBias(double accel_x, double accel_y);
double getFusedSpeedKmh();
double interpolateSpeedAtTime(double target_speed_kmh, unsigned long* crossing_time);

#endif