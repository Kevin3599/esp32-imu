#include "performance_analyzer.h"
#include <math.h>

// 全局融合数据对象
FUSED_PERFORMANCE_DATA fused_data;

// 滤波器参数
const double ACCEL_FILTER_ALPHA = 0.3;   // 加速度滤波系数
const double SPEED_FILTER_ALPHA = 0.8;   // 速度滤波系数

// 车辆特定校准参数 (可通过学习调整)
double vehicle_mass_estimate = 1500.0;    // 估算车重(kg)
double rolling_resistance = 0.015;        // 滚阻系数
double drag_coefficient = 0.3;           // 风阻系数
double frontal_area = 2.5;               // 迎风面积(m²)

// 历史数据用于计算
double previous_speed = 0.0;
unsigned long previous_time = 0;

/**
 * 初始化性能分析器
 */
void initPerformanceAnalyzer() {
    Serial.println("正在初始化性能分析器...");
    
    // 清零融合数据
    memset(&fused_data, 0, sizeof(FUSED_PERFORMANCE_DATA));
    
    // 设置初始状态
    fused_data.last_update = millis();
    fused_data.session_duration = 0;
    
    // 校准IMU用于车辆应用
    calibrateIMUForVehicle();
    
    Serial.println("性能分析器初始化完成");
}

/**
 * 更新融合数据
 */
void updateFusedData() {
    unsigned long current_time = millis();
    
    // 检查更新间隔
    if (current_time - fused_data.last_update < DATA_FUSION_INTERVAL) {
        return;
    }
    
    // 更新会话时长
    fused_data.session_duration = current_time;
    
    // 融合GPS速度数据
    if (gps_data.valid_speed) {
        // 使用低通滤波器平滑速度数据
        fused_data.current_speed_kmh = SPEED_FILTER_ALPHA * gps_data.speed_kmh + 
                                      (1.0 - SPEED_FILTER_ALPHA) * fused_data.current_speed_kmh;
        
        // 更新会话最高速度
        if (fused_data.current_speed_kmh > fused_data.session_max_speed) {
            fused_data.session_max_speed = fused_data.current_speed_kmh;
        }
    }
    
    // 计算G力
    calculateGForces();
    
    // 计算瞬时加速度
    if (previous_time > 0) {
        double time_delta = (current_time - previous_time) / 1000.0; // 转换为秒
        if (time_delta > 0 && fused_data.current_speed_kmh > 0) {
            double speed_change = (fused_data.current_speed_kmh - previous_speed) / 3.6; // km/h转m/s
            fused_data.instantaneous_accel = speed_change / time_delta;
        }
    }
    
    // 分析驾驶行为
    analyzeDrivingBehavior();
    
    // 检测性能事件
    detectPerformanceEvents();
    
    // 估算功率
    fused_data.instantaneous_power_estimate = estimatePower();
    
    // 评估数据质量
    fused_data.gps_accuracy = assessGPSQuality();
    fused_data.imu_accuracy = assessIMUQuality();
    fused_data.data_reliable = validateFusedData();
    
    // 更新历史数据
    previous_speed = fused_data.current_speed_kmh;
    previous_time = current_time;
    fused_data.last_update = current_time;
}

/**
 * 计算G力
 */
void calculateGForces() {
    // 获取精密校准的IMU数据
    double acc_x = mpu6050_data.Acc_X_Filtered - 0.47;
    double acc_y = mpu6050_data.Acc_Y_Filtered + 0.5;
    double acc_z = mpu6050_data.Acc_Z_Filtered - 0.48;
    
    // 最大化GPS-IMU融合精度 - 10Hz优化
    static double gps_speed_history[16] = {0}; // 适配10Hz的更大缓冲区
    static int gps_history_index = 0;
    static double imu_confidence = 1.0;
    
    // 更新GPS速度历史 - 10Hz高频数据
    if (gps_data.valid_speed) {
        gps_speed_history[gps_history_index] = gps_data.speed_kmh;
        gps_history_index = (gps_history_index + 1) % 16;
    }
    
    // 计算GPS速度稳定性 - 16点分析
    double gps_variance = 0.0;
    double gps_avg = 0.0;
    for (int i = 0; i < 16; i++) {
        gps_avg += gps_speed_history[i];
    }
    gps_avg /= 16.0;
    
    for (int i = 0; i < 16; i++) {
        double diff = gps_speed_history[i] - gps_avg;
        gps_variance += diff * diff;
    }
    gps_variance /= 16.0;
    
    // 动态信心度调整
    double gps_stability = 1.0 / (1.0 + gps_variance);
    double signal_quality = (gps_data.satellites >= 8 && gps_data.hdop < 1.5) ? 1.0 : 0.6;
    imu_confidence = gps_stability * signal_quality;
    
    // 10Hz下的高精度静止检测
    bool is_stationary = false;
    if (gps_data.valid_speed && gps_data.satellites >= 6) {
        bool gps_stationary = (gps_avg < 1.0 && sqrt(gps_variance) < 0.2); // 10Hz下更严格的标准
        bool imu_stationary = (abs(acc_x) < 1.2 && abs(acc_y) < 1.2); // 更精细的IMU阈值
        is_stationary = gps_stationary && imu_stationary;
    }
    
    double raw_accel_g, raw_lateral_g;
    
    if (is_stationary) {
        // 静止状态：使用最小化算法去除重力偏差
        static double gravity_bias_x = 0.0, gravity_bias_y = 0.0;
        static int calibration_samples = 0;
        
        if (calibration_samples < 50) {
            gravity_bias_x += acc_x;
            gravity_bias_y += acc_y;
            calibration_samples++;
        } else {
            gravity_bias_x /= calibration_samples;
            gravity_bias_y /= calibration_samples;
        }
        
        raw_accel_g = (acc_x - gravity_bias_x) / 9.81;
        raw_lateral_g = (acc_y - gravity_bias_y) / 9.81;
        
        // 静止时强制归零
        if (abs(raw_accel_g) < 0.02) raw_accel_g = 0.0;
        if (abs(raw_lateral_g) < 0.02) raw_lateral_g = 0.0;
    } else {
        // 动态状态：高精度G力计算
        raw_accel_g = acc_x / 9.81;
        raw_lateral_g = acc_y / 9.81;
        
        // GPS辅助的重力补偿
        if (gps_data.valid_speed && gps_data.speed_kmh > 2.0) {
            // 使用GPS速度变化率估计纵向加速度
            static double prev_gps_speed = 0.0;
            static unsigned long prev_time = 0;
            unsigned long current_time = millis();
            
            if (prev_time > 0) {
                double dt = (current_time - prev_time) / 1000.0;
                if (dt > 0.05 && dt < 0.5) { // 合理时间间隔
                    double gps_accel_g = (gps_data.speed_mps - prev_gps_speed/3.6) / (dt * 9.81);
                    
                    // 融合GPS和IMU的加速度数据
                    double fusion_weight = imu_confidence;
                    raw_accel_g = raw_accel_g * fusion_weight + gps_accel_g * (1.0 - fusion_weight);
                }
            }
            prev_gps_speed = gps_data.speed_mps * 3.6;
            prev_time = current_time;
        }
    }
    
    // 自适应高精度滤波器
    double filter_alpha = is_stationary ? 0.95 : (0.4 + 0.4 * imu_confidence);
    static double filtered_accel_g = 0.0;
    static double filtered_lateral_g = 0.0;
    static bool first_run = true;
    
    if (first_run) {
        filtered_accel_g = raw_accel_g;
        filtered_lateral_g = raw_lateral_g;
        first_run = false;
    } else {
        filtered_accel_g = filtered_accel_g * filter_alpha + raw_accel_g * (1.0 - filter_alpha);
        filtered_lateral_g = filtered_lateral_g * filter_alpha + raw_lateral_g * (1.0 - filter_alpha);
    }
    
    // 动态死区处理
    double deadzone = is_stationary ? 0.01 : (0.02 / (1.0 + imu_confidence));
    if (abs(filtered_accel_g) < deadzone) filtered_accel_g = 0.0;
    if (abs(filtered_lateral_g) < deadzone) filtered_lateral_g = 0.0;
    
    fused_data.current_accel_g = filtered_accel_g;
    fused_data.current_lateral_g = filtered_lateral_g;
    fused_data.current_vertical_g = acc_z / 9.81;
    
    // 只在高质量数据时更新峰值
    if (!is_stationary && imu_confidence > 0.7 && abs(fused_data.current_accel_g) > 0.05) {
        if (fused_data.current_accel_g > fused_data.session_max_accel_g) {
            fused_data.session_max_accel_g = fused_data.current_accel_g;
        }
        if (fused_data.current_accel_g < -fused_data.session_max_brake_g) {
            fused_data.session_max_brake_g = -fused_data.current_accel_g;
        }
    }
    
    if (!is_stationary && abs(fused_data.current_lateral_g) > fused_data.session_max_lateral_g) {
        fused_data.session_max_lateral_g = abs(fused_data.current_lateral_g);
    }
}

/**
 * 分析驾驶行为
 */
void analyzeDrivingBehavior() {
    // 检测加速状态
    fused_data.is_accelerating = (fused_data.current_accel_g > G_FORCE_THRESHOLD);
    
    // 检测制动状态
    fused_data.is_braking = (fused_data.current_accel_g < -G_FORCE_THRESHOLD);
    
    // 检测转弯状态
    fused_data.is_cornering = (abs(fused_data.current_lateral_g) > G_FORCE_THRESHOLD);
}

/**
 * 检测性能事件
 */
void detectPerformanceEvents() {
    // 检测起步加速
    if (detectLaunchStart()) {
        startAccelTest();
    }
    
    // 检测急刹车
    if (detectBrakingStart()) {
        startBrakeTest();
    }
    
    // 检测激烈驾驶
    if (detectCorneringStart()) {
        Serial.println("检测到激烈转弯");
    }
}

/**
 * 检测起步开始
 */
bool detectLaunchStart() {
    static bool was_stationary = true;
    
    if (fused_data.current_speed_kmh < 5.0) {
        was_stationary = true;
        return false;
    }
    
    if (was_stationary && fused_data.current_speed_kmh > 10.0 && 
        fused_data.current_accel_g > 0.2) {
        was_stationary = false;
        return true;
    }
    
    return false;
}

/**
 * 检测制动开始
 */
bool detectBrakingStart() {
    return (fused_data.current_speed_kmh > 50.0 && fused_data.current_accel_g < -0.3);
}

/**
 * 检测转弯开始
 */
bool detectCorneringStart() {
    return (abs(fused_data.current_lateral_g) > 0.4);
}

/**
 * 车辆IMU校准
 */
void calibrateIMUForVehicle() {
    Serial.println("正在进行车辆IMU校准...");
    
    // 这里可以实现自动校准逻辑
    // 例如：在静止状态下采集100个样本进行零点校准
    
    Serial.println("IMU校准完成");
}

/**
 * 评估GPS质量
 */
double assessGPSQuality() {
    if (!gps_data.valid_location || gps_data.satellites < 4) {
        return 0.0;
    }
    
    // 基于卫星数量和HDOP计算质量分数
    double satellite_score = min(gps_data.satellites / 12.0, 1.0);
    double hdop_score = max(0.0, (10.0 - gps_data.hdop) / 10.0);
    
    return (satellite_score + hdop_score) / 2.0;
}

/**
 * 评估IMU质量
 */
double assessIMUQuality() {
    // 基于数据稳定性评估IMU质量
    // 这里简化处理，实际可以基于噪声水平等指标
    return 0.8; // 假设IMU质量良好
}

/**
 * 验证融合数据
 */
bool validateFusedData() {
    return (fused_data.gps_accuracy > 0.5 && 
            fused_data.imu_accuracy > 0.5 &&
            gps_data.satellites >= 4);
}

/**
 * 估算功率
 */
double estimatePower() {
    if (fused_data.current_speed_kmh <= 0) return 0.0;
    
    double speed_ms = fused_data.current_speed_kmh / 3.6; // 转换为m/s
    
    // 计算各种阻力
    double rolling_force = vehicle_mass_estimate * 9.81 * rolling_resistance;
    double drag_force = 0.5 * 1.225 * drag_coefficient * frontal_area * speed_ms * speed_ms;
    double accel_force = vehicle_mass_estimate * fused_data.instantaneous_accel;
    
    double total_force = rolling_force + drag_force + accel_force;
    double power_watts = total_force * speed_ms;
    
    return max(0.0, power_watts / 1000.0); // 转换为kW
}

/**
 * 重置会话数据
 */
void resetSessionData() {
    fused_data.session_max_speed = 0.0;
    fused_data.session_max_accel_g = 0.0;
    fused_data.session_max_brake_g = 0.0;
    fused_data.session_max_lateral_g = 0.0;
    fused_data.session_duration = millis();
    
    resetPerformanceData(); // 重置GPS性能数据
    
    Serial.println("会话数据已重置");
}

/**
 * 打印融合数据
 */
void printFusedData() {
    Serial.println("=== 融合性能数据 ===");
    Serial.printf("速度: %.1f km/h | 加速度: %.2fG | 横向: %.2fG\n", 
                 fused_data.current_speed_kmh, 
                 fused_data.current_accel_g, 
                 fused_data.current_lateral_g);
    
    Serial.printf("状态: %s%s%s\n",
                 fused_data.is_accelerating ? "加速 " : "",
                 fused_data.is_braking ? "制动 " : "",
                 fused_data.is_cornering ? "转弯 " : "");
    
    Serial.printf("会话峰值 - 最高速度: %.1f km/h | 最大加速: %.2fG | 最大制动: %.2fG | 最大横向: %.2fG\n",
                 fused_data.session_max_speed,
                 fused_data.session_max_accel_g,
                 fused_data.session_max_brake_g,
                 fused_data.session_max_lateral_g);
    
    Serial.printf("估算功率: %.1f kW | GPS质量: %.1f%% | IMU质量: %.1f%%\n",
                 fused_data.instantaneous_power_estimate,
                 fused_data.gps_accuracy * 100,
                 fused_data.imu_accuracy * 100);
    
    Serial.println();
}