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