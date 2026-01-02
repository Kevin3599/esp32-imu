#ifndef PERFORMANCE_ANALYZER_H
#define PERFORMANCE_ANALYZER_H

#include "mpu6050.h"
#include "gps_module.h"
#include <Arduino.h>

// 数据融合配置
#define G_FORCE_THRESHOLD 0.1      // G力检测阈值
#define SPEED_CHANGE_THRESHOLD 2.0  // 速度变化阈值(km/h)
#define DATA_FUSION_INTERVAL 50     // 数据融合间隔(ms)

// 融合数据结构
struct FUSED_PERFORMANCE_DATA {
    // 当前状态
    double current_speed_kmh = 0.0;
    double current_accel_g = 0.0;        // 纵向G力 (正=加速, 负=制动)
    double current_lateral_g = 0.0;       // 横向G力
    double current_vertical_g = 0.0;      // 垂直G力
    
    // 实时性能指标
    double instantaneous_accel = 0.0;     // 瞬时加速度 (m/s²)
    double instantaneous_power_estimate = 0.0;  // 估算功率
    
    // 历史峰值
    double session_max_speed = 0.0;
    double session_max_accel_g = 0.0;
    double session_max_brake_g = 0.0;
    double session_max_lateral_g = 0.0;
    
    // 测试状态
    bool is_accelerating = false;
    bool is_braking = false;
    bool is_cornering = false;
    
    // 数据质量指标
    double gps_accuracy = 0.0;
    double imu_accuracy = 0.0;
    bool data_reliable = false;
    
    // 时间戳
    unsigned long last_update = 0;
    unsigned long session_duration = 0;
};

// ===== 摩托车模式数据结构 =====
struct MOTORCYCLE_DATA {
    // 倾角数据 (压弯角度)
    double lean_angle = 0.0;           // 当前倾斜角度 (度, 正=右倾, 负=左倾)
    double lean_angle_filtered = 0.0;  // 滤波后倾角
    double max_lean_left = 0.0;        // 会话最大左倾角
    double max_lean_right = 0.0;       // 会话最大右倾角
    
    // 加速度数据
    double forward_g = 0.0;            // 前进方向G力 (加速/减速)
    double lateral_g = 0.0;            // 侧向G力 (转弯离心力)
    double vertical_g = 0.0;           // 垂直G力 (颠簸/跳跃)
    double combined_g = 0.0;           // 合成G力
    
    // 弯道分析
    double corner_speed = 0.0;         // 入弯速度
    double corner_g = 0.0;             // 弯道G力
    double corner_radius_estimate = 0.0; // 估算弯道半径(m)
    bool in_corner = false;            // 是否在弯道中
    int corner_count = 0;              // 弯道计数
    
    // 骑行状态
    bool is_accelerating = false;      // 加速中
    bool is_braking = false;           // 刹车中
    bool is_leaning = false;           // 压弯中
    bool is_wheelie = false;           // 抬头(前轮离地)
    bool is_stoppie = false;           // 翘尾(后轮离地)
    
    // 峰值记录
    double max_forward_g = 0.0;        // 最大加速G
    double max_brake_g = 0.0;          // 最大刹车G
    double max_lateral_g = 0.0;        // 最大侧向G
    double max_combined_g = 0.0;       // 最大合成G
    double top_speed = 0.0;            // 最高时速
    
    // 翘头/翘尾检测
    double wheelie_angle = 0.0;        // 翘头角度
    double max_wheelie_angle = 0.0;    // 最大翘头角度
    int wheelie_count = 0;             // 翘头次数
    
    // 陀螺仪数据
    double roll_rate = 0.0;            // 横滚角速度 (deg/s)
    double pitch_rate = 0.0;           // 俯仰角速度 (deg/s)
    double yaw_rate = 0.0;             // 偏航角速度 (deg/s)
    
    // 数据质量
    bool data_valid = false;
    unsigned long last_update = 0;
};

// 摩托车模式全局数据
extern MOTORCYCLE_DATA moto_data;
extern bool motorcycle_mode_enabled;

// 全局融合数据对象
extern FUSED_PERFORMANCE_DATA fused_data;

// 函数声明
void initPerformanceAnalyzer();
void updateFusedData();
void analyzeDrivingBehavior();
void calculateGForces();
void detectPerformanceEvents();
void calibrateIMUForVehicle();
void resetSessionData();
void printFusedData();

// ===== 摩托车模式函数 =====
void initMotorcycleMode();
void updateMotorcycleData();
void calculateLeanAngle();
void detectWheelieStoppie();
void analyzeCorner();
void resetMotorcycleSession();

// 性能事件检测
bool detectLaunchStart();
bool detectBrakingStart();
bool detectCorneringStart();

// 数据质量评估
double assessGPSQuality();
double assessIMUQuality();
bool validateFusedData();

// 功率估算 (基于加速度和速度)
double estimatePower();
#endif