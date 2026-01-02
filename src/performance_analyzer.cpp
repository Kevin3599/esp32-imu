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

// ========================================
// ===== 摩托车模式实现 =====
// ========================================

// 摩托车模式全局变量
MOTORCYCLE_DATA moto_data;
bool motorcycle_mode_enabled = false;

// 摩托车模式滤波参数 - 优化响应速度
const double COMPLEMENTARY_ALPHA = 0.96;    // 互补滤波系数 (0.96=陀螺仪为主，响应快)
const double MOTO_G_FILTER_ALPHA = 0.1;     // G力滤波系数 (降低=更快响应)
const double WHEELIE_THRESHOLD = 15.0;      // 翘头检测阈值(度)
const double LEAN_THRESHOLD = 10.0;         // 压弯检测阈值(度)
const double CORNER_G_THRESHOLD = 0.3;      // 弯道G力阈值

// 互补滤波状态变量
static double gyro_lean_angle = 0.0;        // 陀螺仪积分的倾角
static double gyro_pitch_angle = 0.0;       // 陀螺仪积分的俯仰角
static unsigned long last_moto_update = 0;  // 上次更新时间

/**
 * 初始化摩托车模式
 */
void initMotorcycleMode() {
    Serial.println("正在初始化摩托车模式...");
    
    memset(&moto_data, 0, sizeof(MOTORCYCLE_DATA));
    motorcycle_mode_enabled = true;
    moto_data.last_update = millis();
    last_moto_update = millis();
    gyro_lean_angle = 0.0;
    gyro_pitch_angle = 0.0;
    
    Serial.println("摩托车模式已启用 - 互补滤波");
}

/**
 * 更新摩托车数据
 */
void updateMotorcycleData() {
    if (!motorcycle_mode_enabled) return;
    
    unsigned long current_time = millis();
    
    // 计算倾角
    calculateLeanAngle();
    
    // 计算G力 - 垂直安装坐标系映射
    // X轴向下(重力) Y轴左右 Z轴前后
    double acc_x = mpu6050_data.Acc_X_Filtered - 0.47;  // 垂直方向(重力)
    double acc_y = mpu6050_data.Acc_Y_Filtered + 0.5;   // 侧向
    double acc_z = mpu6050_data.Acc_Z_Filtered - 0.48;  // 前后方向
    
    // 摩托车坐标系(垂直安装): Z=前进, Y=侧向, X=垂直
    double raw_forward_g = -acc_z / 9.81;   // Z轴前后(取反，加速为正)
    double raw_lateral_g = acc_y / 9.81;    // Y轴侧向
    double raw_vertical_g = acc_x / 9.81;   // X轴垂直(重力约为1G)
    
    // 滤波
    static double prev_forward = 0, prev_lateral = 0, prev_vertical = 0;
    moto_data.forward_g = prev_forward * MOTO_G_FILTER_ALPHA + raw_forward_g * (1 - MOTO_G_FILTER_ALPHA);
    moto_data.lateral_g = prev_lateral * MOTO_G_FILTER_ALPHA + raw_lateral_g * (1 - MOTO_G_FILTER_ALPHA);
    moto_data.vertical_g = prev_vertical * MOTO_G_FILTER_ALPHA + raw_vertical_g * (1 - MOTO_G_FILTER_ALPHA);
    prev_forward = moto_data.forward_g;
    prev_lateral = moto_data.lateral_g;
    prev_vertical = moto_data.vertical_g;
    
    // 合成G力
    moto_data.combined_g = sqrt(moto_data.forward_g * moto_data.forward_g + 
                                moto_data.lateral_g * moto_data.lateral_g);
    
    // 陀螺仪数据 (角速度)
    moto_data.roll_rate = mpu6050_data.Gyro_Z_Filtered;   // 横滚
    moto_data.pitch_rate = mpu6050_data.Gyro_Y_Filtered;  // 俯仰
    moto_data.yaw_rate = mpu6050_data.Gyro_X_Filtered;    // 偏航
    
    // 检测翘头/翘尾
    detectWheelieStoppie();
    
    // 分析弯道
    analyzeCorner();
    
    // 更新骑行状态
    moto_data.is_accelerating = (moto_data.forward_g > 0.15);
    moto_data.is_braking = (moto_data.forward_g < -0.2);
    moto_data.is_leaning = (abs(moto_data.lean_angle_filtered) > LEAN_THRESHOLD);
    
    // 更新峰值记录
    if (moto_data.forward_g > moto_data.max_forward_g) {
        moto_data.max_forward_g = moto_data.forward_g;
    }
    if (-moto_data.forward_g > moto_data.max_brake_g) {
        moto_data.max_brake_g = -moto_data.forward_g;
    }
    if (abs(moto_data.lateral_g) > moto_data.max_lateral_g) {
        moto_data.max_lateral_g = abs(moto_data.lateral_g);
    }
    if (moto_data.combined_g > moto_data.max_combined_g) {
        moto_data.max_combined_g = moto_data.combined_g;
    }
    
    // 更新最高速度
    if (gps_data.valid_speed && gps_data.speed_kmh > moto_data.top_speed) {
        moto_data.top_speed = gps_data.speed_kmh;
    }
    
    moto_data.data_valid = true;
    moto_data.last_update = current_time;
}

/**
 * 计算倾斜角度 (压弯角度) - 互补滤波
 * 融合陀螺仪(快速响应) + 加速度计(长期稳定)
 * 垂直安装: X轴向下承受重力，Y轴左右，Z轴前后
 */
void calculateLeanAngle() {
    unsigned long current_time = millis();
    double dt = (current_time - last_moto_update) / 1000.0;  // 秒
    if (dt <= 0 || dt > 0.5) dt = 0.005;  // 异常保护
    last_moto_update = current_time;
    
    // === 加速度计计算倾角 (静态准确，动态有离心力干扰) ===
    double acc_x = mpu6050_data.Acc_X_Filtered - 0.47;  // 垂直方向(重力)
    double acc_y = mpu6050_data.Acc_Y_Filtered + 0.5;   // 侧向
    double acc_z = mpu6050_data.Acc_Z_Filtered - 0.48;  // 前后
    
    // 加速度计倾角 (Y-X平面) - 取反使右倾为正
    double accel_lean = -atan2(acc_y, abs(acc_x)) * 180.0 / M_PI;
    
    // 加速度计俯仰角 (Z-X平面) - 用于翘头检测
    double accel_pitch = atan2(-acc_z, abs(acc_x)) * 180.0 / M_PI;
    
    // === 陀螺仪积分 (动态准确，会漂移) ===
    // 垂直安装时: 绕Y轴旋转 = 左右倾斜(lean), 绕Z轴旋转 = 前后俯仰(pitch)
    double gyro_rate_lean = -mpu6050_data.Gyro_Y_Filtered;   // 左右倾斜角速度 (deg/s) - 取反
    double gyro_rate_pitch = mpu6050_data.Gyro_Z_Filtered;   // 前后俯仰角速度 (deg/s)
    
    // 陀螺仪积分
    gyro_lean_angle += gyro_rate_lean * dt;
    gyro_pitch_angle += gyro_rate_pitch * dt;
    
    // === 互补滤波 ===
    // 高通滤波陀螺仪(快速变化) + 低通滤波加速度计(长期稳定)
    // lean = alpha * (lean + gyro*dt) + (1-alpha) * accel_lean
    double fused_lean = COMPLEMENTARY_ALPHA * (gyro_lean_angle) + 
                        (1.0 - COMPLEMENTARY_ALPHA) * accel_lean;
    
    double fused_pitch = COMPLEMENTARY_ALPHA * (gyro_pitch_angle) + 
                         (1.0 - COMPLEMENTARY_ALPHA) * accel_pitch;
    
    // 更新陀螺仪积分值，防止漂移累积
    gyro_lean_angle = fused_lean;
    gyro_pitch_angle = fused_pitch;
    
    // 限制范围 -70° ~ +70°
    if (fused_lean > 70.0) fused_lean = 70.0;
    if (fused_lean < -70.0) fused_lean = -70.0;
    
    // 输出 - 直接使用融合值，不再额外滤波(已经够平滑)
    moto_data.lean_angle = fused_lean;           // 原始融合倾角
    moto_data.lean_angle_filtered = fused_lean;  // 直接使用，无额外延迟
    moto_data.wheelie_angle = fused_pitch;       // 俯仰角(翘头)
    
    // 更新最大倾角记录
    if (fused_lean < moto_data.max_lean_left) {
        moto_data.max_lean_left = fused_lean;  // 左倾是负值
    }
    if (fused_lean > moto_data.max_lean_right) {
        moto_data.max_lean_right = fused_lean; // 右倾是正值
    }
}

/**
 * 检测翘头(Wheelie)和翘尾(Stoppie)
 * 使用互补滤波计算的俯仰角 (已在calculateLeanAngle中计算)
 */
void detectWheelieStoppie() {
    // 俯仰角已经在 calculateLeanAngle() 中通过互补滤波计算
    double pitch_angle = moto_data.wheelie_angle;
    
    // 翘头检测: 俯仰角 > 阈值 且 有加速
    static bool was_wheelie = false;
    moto_data.is_wheelie = (pitch_angle > WHEELIE_THRESHOLD && moto_data.forward_g > 0.1);
    
    if (moto_data.is_wheelie && !was_wheelie) {
        moto_data.wheelie_count++;
        Serial.println("检测到翘头!");
    }
    was_wheelie = moto_data.is_wheelie;
    
    // 更新最大翘头角度
    if (pitch_angle > moto_data.max_wheelie_angle) {
        moto_data.max_wheelie_angle = pitch_angle;
    }
    
    // 翘尾检测: 俯仰角 < -阈值 且 有刹车
    moto_data.is_stoppie = (pitch_angle < -WHEELIE_THRESHOLD && moto_data.forward_g < -0.3);
}

/**
 * 弯道分析
 */
void analyzeCorner() {
    static bool was_in_corner = false;
    static double corner_entry_speed = 0.0;
    static double corner_max_g = 0.0;
    
    // 检测是否在弯道中
    bool currently_cornering = (abs(moto_data.lean_angle_filtered) > LEAN_THRESHOLD || 
                                abs(moto_data.lateral_g) > CORNER_G_THRESHOLD);
    
    // 弯道入口
    if (currently_cornering && !was_in_corner) {
        moto_data.in_corner = true;
        corner_entry_speed = gps_data.valid_speed ? gps_data.speed_kmh : 0.0;
        corner_max_g = 0.0;
        Serial.printf("进入弯道 @ %.1f km/h\n", corner_entry_speed);
    }
    
    // 弯道中
    if (moto_data.in_corner) {
        moto_data.corner_speed = corner_entry_speed;
        
        // 更新弯道最大G力
        double current_corner_g = abs(moto_data.lateral_g);
        if (current_corner_g > corner_max_g) {
            corner_max_g = current_corner_g;
        }
        moto_data.corner_g = corner_max_g;
        
        // 估算弯道半径: r = v² / (g * G力)
        // v 单位 m/s, g = 9.81
        if (gps_data.valid_speed && moto_data.lateral_g != 0) {
            double speed_ms = gps_data.speed_kmh / 3.6;
            moto_data.corner_radius_estimate = (speed_ms * speed_ms) / (9.81 * abs(moto_data.lateral_g));
        }
    }
    
    // 弯道出口
    if (!currently_cornering && was_in_corner) {
        moto_data.in_corner = false;
        moto_data.corner_count++;
        Serial.printf("弯道完成 #%d | 入弯: %.1f km/h | 最大G: %.2f | 估算半径: %.0fm\n",
                     moto_data.corner_count, corner_entry_speed, corner_max_g, moto_data.corner_radius_estimate);
    }
    
    was_in_corner = currently_cornering;
}

/**
 * 重置摩托车会话数据
 */
void resetMotorcycleSession() {
    moto_data.max_lean_left = 0.0;
    moto_data.max_lean_right = 0.0;
    moto_data.max_forward_g = 0.0;
    moto_data.max_brake_g = 0.0;
    moto_data.max_lateral_g = 0.0;
    moto_data.max_combined_g = 0.0;
    moto_data.max_wheelie_angle = 0.0;
    moto_data.wheelie_count = 0;
    moto_data.corner_count = 0;
    moto_data.top_speed = 0.0;
    
    Serial.println("摩托车会话数据已重置");
}