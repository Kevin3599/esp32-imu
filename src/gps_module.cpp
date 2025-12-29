#include "gps_module.h"
#include "mpu6050.h"  // 添加MPU6050头文件
#include <SPIFFS.h>
#include <ArduinoJson.h>

// 全局对象定义
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // 使用Serial2
GPS_DATA gps_data;
PERFORMANCE_DATA performance_data;

// 性能测试阈值
const double ACCEL_TEST_START_SPEED = 5.0;    // 5km/h开始加速测试
const double ACCEL_TEST_60_SPEED = 60.0;      // 60km/h目标
const double ACCEL_TEST_100_SPEED = 100.0;    // 100km/h目标
const double ACCEL_TEST_200_SPEED = 200.0;    // 200km/h目标

const double BRAKE_TEST_START_SPEED = 95.0;   // 95km/h以上开始制动测试
const double BRAKE_TEST_END_SPEED = 5.0;      // 5km/h以下结束制动测试

const double LAP_RADIUS = 50.0;                // 起点/终点检测半径(米)

/**
 * 初始化GPS模块 - 针对Dragy风格性能测量优化
 */
bool initGPS() {
    Serial.println("正在初始化GPS模块 (Dragy模式)...");
    
    // 初始化GPS串口 - 使用GT-U7默认的9600波特率
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    // 等待GPS模块启动
    delay(1000);
    
    // 清空串口缓冲区
    while (gpsSerial.available()) {
        gpsSerial.read();
    }
    
    Serial.println("配置GPS模块参数 (10Hz模式)...");
    
    // ===== 关键：10Hz更新频率 (100ms) =====
    // GT-U7在9600波特率下支持10Hz，但需要精简NMEA输出
    gpsSerial.println("$PMTK220,100*2F");
    delay(300);
    
    // 只输出RMC和GGA - 10Hz时必须精简，否则数据溢出
    // RMC包含：时间、位置、速度(Doppler)、航向
    // GGA包含：定位质量、卫星数、HDOP、高度
    gpsSerial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
    delay(300);
    
    // 启用SBAS增强定位精度
    gpsSerial.println("$PMTK313,1*2E");
    delay(300);
    
    // 设置DGPS模式为WAAS
    gpsSerial.println("$PMTK301,2*2E");
    delay(300);
    
    Serial.println("GPS模块初始化完成 - 10Hz Dragy模式");
    Serial.printf("GPS配置: RX=%d, TX=%d, 波特率=%d, 更新率=10Hz\n", 
                  GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
    
    // 初始化性能数据
    resetPerformanceData();
    
    return true;
}

/**
 * 更新GPS数据
 */
void updateGPSData() {
    static unsigned long last_gps_data_time = 0;
    static unsigned long last_reconnect_attempt = 0;
    static int signal_lost_count = 0;
    static bool was_connected = false;
    
    unsigned long current_time = millis();
    
    // 读取GPS数据
    bool got_new_data = false;
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            got_new_data = true;
            last_gps_data_time = current_time;
            signal_lost_count = 0;
        }
    }
    
    // GPS信号监控和恢复
    if (current_time - last_gps_data_time > 3000) { // 3秒没有数据
        signal_lost_count++;
        
        // 每10秒尝试一次恢复
        if (current_time - last_reconnect_attempt > 10000) {
            Serial.println("GPS信号丢失，尝试恢复...");
            
            // 发送热重启命令
            gpsSerial.println("$PMTK101*32");
            delay(100);
            
            // 重新配置关键参数
            gpsSerial.println("$PMTK220,200*2C"); // 5Hz更新
            delay(100);
            gpsSerial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
            delay(100);
            
            last_reconnect_attempt = current_time;
            Serial.printf("GPS恢复尝试完成，信号丢失计数: %d\n", signal_lost_count);
        }
        
        // 标记GPS数据为无效
        if (signal_lost_count > 5) {
            gps_data.valid_location = false;
            gps_data.valid_speed = false;
            gps_data.valid_time = false;
            gps_data.valid_date = false;
        }
    }
    
    // 处理有效的GPS数据
    if (got_new_data) {
        // 更新基础位置信息
        if (gps.location.isValid()) {
            double new_lat = gps.location.lat();
            double new_lon = gps.location.lng();
            
            // 数据合理性检验
            if (abs(new_lat) <= 90.0 && abs(new_lon) <= 180.0) {
                // 简单的位置跳变检测 (仅在已定位后)
                if (gps_data.valid_location) {
                    double distance = calculateDistance(gps_data.latitude, gps_data.longitude, new_lat, new_lon);
                    if (distance < 1000) { // 1km以内的变化才接受
                        gps_data.latitude = new_lat;
                        gps_data.longitude = new_lon;
                    }
                } else {
                    gps_data.latitude = new_lat;
                    gps_data.longitude = new_lon;
                }
                gps_data.valid_location = true;
                gps_data.fix_age = gps.location.age();
            }
        } else {
            gps_data.valid_location = false;
        }
        
        // 更新高度
        if (gps.altitude.isValid()) {
            double new_alt = gps.altitude.meters();
            if (new_alt > -1000 && new_alt < 10000) { // 合理高度范围
                gps_data.altitude = new_alt;
            }
        }
        
        // 更新时间
        if (gps.date.isValid() && gps.time.isValid()) {
            gps_data.year = gps.date.year();
            gps_data.month = gps.date.month();
            gps_data.day = gps.date.day();
            gps_data.hour = gps.time.hour();
            gps_data.minute = gps.time.minute();
            gps_data.second = gps.time.second();
            gps_data.centisecond = gps.time.centisecond();
            gps_data.valid_date = gps.date.isValid();
            gps_data.valid_time = gps.time.isValid();
        }
        
        // 更新速度和方向 - 极限优化版本
        if (gps.speed.isValid()) {
            double new_speed = gps.speed.kmph();
            if (new_speed >= 0 && new_speed < 400) {
                
                // 极限精度的静止检测 - 适配10Hz高频数据
                static double speed_buffer[20] = {0}; // 增加缓冲区适配10Hz
                static int buffer_index = 0;
                static double speed_sum = 0.0;
                static int stable_count = 0;
                
                // 更新循环缓冲区
                speed_sum -= speed_buffer[buffer_index];
                speed_buffer[buffer_index] = new_speed;
                speed_sum += new_speed;
                buffer_index = (buffer_index + 1) % 20;
                
                double avg_speed = speed_sum / 20.0;
                
                // 计算标准差和变化率 - 10Hz高频分析
                double variance = 0.0;
                for (int i = 0; i < 20; i++) {
                    double diff = speed_buffer[i] - avg_speed;
                    variance += diff * diff;
                }
                variance /= 20.0;
                double std_dev = sqrt(variance);
                
                // 10Hz下的动态阈值 - 更精细的控制
                double signal_quality = (gps_data.satellites >= 8) ? 1.0 : 0.7;
                double base_threshold = (gps_data.hdop < 1.2) ? 1.0 : 1.6; // 更低阈值
                double dynamic_threshold = base_threshold / signal_quality;
                
                // 10Hz下的稳定性检测 - 更严格标准
                bool is_speed_stable = (std_dev < 0.2) && (avg_speed < dynamic_threshold); // 更低噪声阈值
                bool is_acceleration_stable = abs(new_speed - gps_data.speed_kmh) < 0.3; // 更灵敏的加速度检测
                
                if (is_speed_stable && is_acceleration_stable) {
                    stable_count++;
                    if (stable_count >= 15) { // 10Hz下需要15次稳定(1.5秒)
                        avg_speed = 0.0;
                        stable_count = 15; // 防止溢出
                    }
                } else if (avg_speed > 2.5) { // 更低的重置阈值
                    stable_count = 0;
                }
                
                // 10Hz高频自适应滤波 - 更精细的控制
                double filter_strength;
                if (avg_speed < 0.5) {
                    filter_strength = 0.92; // 10Hz下可以使用更强滤波
                } else if (avg_speed < 1.5) {
                    filter_strength = 0.80 * signal_quality;
                } else if (avg_speed < 4.0) {
                    filter_strength = 0.60 * signal_quality;
                } else {
                    filter_strength = 0.35 * signal_quality; // 高速时更快响应
                }
                
                // 应用滤波
                if (gps_data.valid_speed) {
                    gps_data.speed_kmh = gps_data.speed_kmh * filter_strength + avg_speed * (1.0 - filter_strength);
                } else {
                    gps_data.speed_kmh = avg_speed;
                }
                
                // 10Hz下的最终精密死区处理
                double final_threshold = (gps_data.satellites >= 8 && gps_data.hdop < 1.0) ? 0.8 : 1.2;
                if (gps_data.speed_kmh < final_threshold) {
                    gps_data.speed_kmh = 0.0;
                }
                
                gps_data.speed_mps = gps_data.speed_kmh / 3.6;
                gps_data.valid_speed = true;
                
                // 更新最高速度
                if (gps_data.speed_kmh > 1.5 && gps_data.speed_kmh > gps_data.max_speed_session) {
                    gps_data.max_speed_session = gps_data.speed_kmh;
                }
            } else {
                gps_data.valid_speed = false;
            }
        } else {
            gps_data.valid_speed = false;
        }
        
        if (gps.course.isValid()) {
            double new_course = gps.course.deg();
            if (new_course >= 0 && new_course <= 360) {
                gps_data.course = new_course;
            }
        }
        
        // 更新卫星信息 - 增强版
        if (gps.satellites.isValid()) {
            gps_data.satellites = gps.satellites.value();
            
            // 记录卫星数量历史，用于信号质量分析
            static int sat_history[5] = {0};
            static int sat_index = 0;
            sat_history[sat_index] = gps_data.satellites;
            sat_index = (sat_index + 1) % 5;
            
            // 计算平均卫星数量
            int sat_sum = 0;
            for (int i = 0; i < 5; i++) {
                sat_sum += sat_history[i];
            }
            int avg_sats = sat_sum / 5;
            
            // 信号质量评估
            if (avg_sats >= 10) {
                gps_data.fix_quality = 2; // 优秀
            } else if (avg_sats >= 6) {
                gps_data.fix_quality = 1; // 良好
            } else {
                gps_data.fix_quality = 0; // 较差
            }
        }
        
        // 更新HDOP信息 - 精密化
        if (gps.hdop.isValid()) {
            double new_hdop = gps.hdop.hdop();
            if (new_hdop > 0 && new_hdop < 20) {
                // HDOP滤波减少噪声
                static double hdop_filtered = 0.0;
                static bool hdop_init = false;
                
                if (!hdop_init) {
                    hdop_filtered = new_hdop;
                    hdop_init = true;
                } else {
                    hdop_filtered = hdop_filtered * 0.8 + new_hdop * 0.2;
                }
                
                gps_data.hdop = hdop_filtered;
                
                // 精度估计 (基于HDOP的经验公式)
                gps_data.position_accuracy = hdop_filtered * 2.5; // CEP50估计
                gps_data.speed_accuracy = hdop_filtered * 0.1;     // 速度精度估计
            }
        }
    } // 闭合got_new_data的if语句
    
    // 计算性能指标
    calculatePerformanceMetrics();
    
    // IMU-GPS融合将在后续实现
}

/**
 * 计算性能指标
 */
void calculatePerformanceMetrics() {
    if (!gps_data.valid_speed || !gps_data.valid_location) return;
    
    unsigned long current_time = millis();
    double current_speed = gps_data.speed_kmh;
    
    // 加速测试逻辑
    if (!performance_data.accel_test_active && 
        current_speed >= ACCEL_TEST_START_SPEED && 
        current_speed < ACCEL_TEST_60_SPEED) {
        startAccelTest();
    }
    
    if (performance_data.accel_test_active) {
        unsigned long elapsed = current_time - performance_data.accel_start_time;
        
        // 记录0-60km/h时间
        if (performance_data.accel_0_60_time == 0.0 && current_speed >= ACCEL_TEST_60_SPEED) {
            performance_data.accel_0_60_time = elapsed / 1000.0;
            Serial.printf("0-60km/h: %.2f秒\n", performance_data.accel_0_60_time);
        }
        
        // 记录0-100km/h时间
        if (performance_data.accel_0_100_time == 0.0 && current_speed >= ACCEL_TEST_100_SPEED) {
            performance_data.accel_0_100_time = elapsed / 1000.0;
            Serial.printf("0-100km/h: %.2f秒\n", performance_data.accel_0_100_time);
        }
        
        // 记录100-200km/h时间
        if (performance_data.accel_100_200_time == 0.0 && current_speed >= ACCEL_TEST_200_SPEED) {
            performance_data.accel_100_200_time = elapsed / 1000.0;
            Serial.printf("100-200km/h: %.2f秒\n", performance_data.accel_100_200_time);
            stopAccelTest();
        }
        
        // 超时停止测试
        if (elapsed > 30000) { // 30秒超时
            stopAccelTest();
        }
    }
    
    // 制动测试逻辑
    if (!performance_data.brake_test_active && 
        current_speed >= BRAKE_TEST_START_SPEED) {
        performance_data.brake_test_active = true;
        performance_data.brake_start_speed = current_speed;
        performance_data.brake_start_time = current_time;
        Serial.printf("开始制动测试 - 起始速度: %.1fkm/h\n", current_speed);
    }
    
    if (performance_data.brake_test_active && current_speed <= BRAKE_TEST_END_SPEED) {
        unsigned long elapsed = current_time - performance_data.brake_start_time;
        performance_data.brake_100_0_time = elapsed / 1000.0;
        Serial.printf("制动测试完成: %.2f秒\n", performance_data.brake_100_0_time);
        stopBrakeTest();
    }
}

/**
 * 开始加速测试
 */
void startAccelTest() {
    performance_data.accel_test_active = true;
    performance_data.accel_start_time = millis();
    performance_data.accel_0_60_time = 0.0;
    performance_data.accel_0_100_time = 0.0;
    performance_data.accel_100_200_time = 0.0;
    Serial.println("开始加速测试");
}

/**
 * 停止加速测试
 */
void stopAccelTest() {
    performance_data.accel_test_active = false;
    Serial.println("加速测试结束");
    saveSessionData();
}

/**
 * 开始制动测试
 */
void startBrakeTest() {
    performance_data.brake_test_active = true;
    performance_data.brake_start_time = millis();
    Serial.println("开始制动测试");
}

/**
 * 停止制动测试
 */
void stopBrakeTest() {
    performance_data.brake_test_active = false;
    Serial.println("制动测试结束");
    saveSessionData();
}

/**
 * 重置性能数据
 */
void resetPerformanceData() {
    memset(&performance_data, 0, sizeof(PERFORMANCE_DATA)); // 清零结构体
    performance_data.session_start_time = millis();
    performance_data.session_active = true;
}

/**
 * 计算两点间距离 (haversine公式)
 */
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0; // 地球半径(米)
    double dLat = (lat2 - lat1) * PI / 180.0;
    double dLon = (lon2 - lon1) * PI / 180.0;
    double a = sin(dLat/2) * sin(dLat/2) + 
               cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) * 
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

/**
 * 格式化时间显示
 */
String formatTime(unsigned long milliseconds) {
    unsigned long seconds = milliseconds / 1000;
    unsigned long minutes = seconds / 60;
    seconds %= 60;
    unsigned long ms = milliseconds % 1000;
    
    char buffer[20];
    sprintf(buffer, "%02lu:%02lu.%03lu", minutes, seconds, ms);
    return String(buffer);
}

/**
 * 保存会话数据到SPIFFS
 */
void saveSessionData() {
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS初始化失败");
        return;
    }
    
    // 创建JSON文档
    JsonDocument doc;
    
    // 保存GPS数据
    doc["gps"]["max_speed"] = gps_data.max_speed_session;
    doc["gps"]["distance"] = gps_data.distance_traveled;
    
    // 保存性能数据
    doc["performance"]["accel_0_60"] = performance_data.accel_0_60_time;
    doc["performance"]["accel_0_100"] = performance_data.accel_0_100_time;
    doc["performance"]["accel_100_200"] = performance_data.accel_100_200_time;
    doc["performance"]["brake_100_0"] = performance_data.brake_100_0_time;
    doc["performance"]["max_accel_g"] = performance_data.max_accel_g;
    doc["performance"]["max_brake_g"] = performance_data.max_brake_g;
    doc["performance"]["max_lateral_g"] = performance_data.max_lateral_g;
    doc["performance"]["lap_count"] = performance_data.lap_count;
    doc["performance"]["best_lap"] = performance_data.best_lap_time;
    
    // 保存时间戳
    doc["timestamp"] = millis();
    
    // 写入文件
    File file = SPIFFS.open("/session.json", FILE_WRITE);
    if (file) {
        serializeJson(doc, file);
        file.close();
        Serial.println("会话数据已保存");
    } else {
        Serial.println("保存会话数据失败");
    }
}

/**
 * 从SPIFFS加载会话数据
 */
void loadSessionData() {
    if (!SPIFFS.begin(true)) return;
    
    File file = SPIFFS.open("/session.json", FILE_READ);
    if (!file) {
        Serial.println("没有找到会话数据文件");
        return;
    }
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        Serial.println("解析会话数据失败");
        return;
    }
    
    // 恢复数据
    gps_data.max_speed_session = doc["gps"]["max_speed"] | 0.0;
    gps_data.distance_traveled = doc["gps"]["distance"] | 0.0;
    
    performance_data.accel_0_60_time = doc["performance"]["accel_0_60"] | 0.0;
    performance_data.accel_0_100_time = doc["performance"]["accel_0_100"] | 0.0;
    performance_data.accel_100_200_time = doc["performance"]["accel_100_200"] | 0.0;
    performance_data.brake_100_0_time = doc["performance"]["brake_100_0"] | 0.0;
    performance_data.max_accel_g = doc["performance"]["max_accel_g"] | 0.0;
    performance_data.max_brake_g = doc["performance"]["max_brake_g"] | 0.0;
    performance_data.max_lateral_g = doc["performance"]["max_lateral_g"] | 0.0;
    performance_data.lap_count = doc["performance"]["lap_count"] | 0;
    performance_data.best_lap_time = doc["performance"]["best_lap"] | 0.0;
    
    Serial.println("会话数据已加载");
}

/**
 * 打印GPS数据到串口
 */
void printGPSData() {
    Serial.println("=== GPS状态 ===");
    Serial.printf("卫星数: %d, HDOP: %.2f\n", gps_data.satellites, gps_data.hdop);
    
    if (gps_data.valid_location) {
        Serial.printf("位置: %.6f, %.6f (高度: %.1fm)\n", 
                     gps_data.latitude, gps_data.longitude, gps_data.altitude);
    } else {
        Serial.println("位置: 无效");
    }
    
    if (gps_data.valid_speed) {
        Serial.printf("速度: %.1f km/h (%.1f m/s)\n", gps_data.speed_kmh, gps_data.speed_mps);
        Serial.printf("最高速度: %.1f km/h\n", gps_data.max_speed_session);
    } else {
        Serial.println("速度: 无效");
    }
    
    if (gps_data.valid_time) {
        Serial.printf("时间: %04d-%02d-%02d %02d:%02d:%02d\n",
                     gps_data.year, gps_data.month, gps_data.day,
                     gps_data.hour, gps_data.minute, gps_data.second);
    }
    
    Serial.println("=== 性能数据 ===");
    if (performance_data.accel_0_60_time > 0) {
        Serial.printf("0-60km/h: %.2fs\n", performance_data.accel_0_60_time);
    }
    if (performance_data.accel_0_100_time > 0) {
        Serial.printf("0-100km/h: %.2fs\n", performance_data.accel_0_100_time);
    }
    if (performance_data.brake_100_0_time > 0) {
        Serial.printf("100-0km/h制动: %.2fs\n", performance_data.brake_100_0_time);
    }
    
    Serial.println();
}

/**
 * 检查GPS数据有效性
 */
bool isGPSDataValid() {
    return gps_data.valid_location && gps_data.valid_speed && 
           gps_data.satellites >= 4 && gps_data.hdop < 5.0;
}

// =====================================================
// ===== Dragy模式 GPS-IMU 融合算法实现 =====
// =====================================================

/**
 * IMU偏置校准 - 静止时自动校准加速度偏置
 * 原理：静止时加速度应该为0，累积样本计算平均偏置
 */
void calibrateIMUBias(double accel_x, double accel_y) {
    // 只在速度非常低时进行校准
    if (gps_data.speed_ms < 0.5 && gps_data.calibration_count < 200) {
        // 累积校准 - 使用前向加速度（假设X轴为前进方向）
        double forward_accel = accel_x;  // 可以根据安装方向调整
        
        gps_data.accel_bias = (gps_data.accel_bias * gps_data.calibration_count + forward_accel) 
                               / (gps_data.calibration_count + 1);
        gps_data.calibration_count++;
        
        if (gps_data.calibration_count == 200) {
            gps_data.imu_calibrated = true;
            Serial.printf("IMU校准完成: 偏置=%.4f m/s²\n", gps_data.accel_bias);
        }
    }
}

/**
 * GPS-IMU速度融合 - 互补滤波算法
 * 
 * 原理：
 * - GPS提供绝对准确但低频(10Hz)的速度
 * - IMU提供高频(200Hz)但会漂移的加速度
 * - 用互补滤波将两者融合：短时靠IMU，长时靠GPS拉回
 * 
 * @param accel_x X轴加速度 (m/s²) - 假设为前进方向
 * @param accel_y Y轴加速度 (m/s²) - 横向
 * @param dt 时间间隔 (秒)
 */
void fuseIMUSpeed(double accel_x, double accel_y, double dt) {
    static double imu_integrated_speed = 0.0;  // IMU积分速度
    static unsigned long last_fusion_time = 0;
    
    unsigned long now = millis();
    
    // 防止异常的dt值
    if (dt <= 0 || dt > 0.1) {
        dt = 0.005; // 默认5ms
    }
    
    // 1. IMU偏置校准（静止时）
    calibrateIMUBias(accel_x, accel_y);
    
    // 2. 计算补偿后的前向加速度
    double forward_accel = accel_x;
    if (gps_data.imu_calibrated) {
        forward_accel -= gps_data.accel_bias;
    }
    
    // 3. 死区处理 - 过滤微小噪声
    const double ACCEL_DEADZONE = 0.3; // m/s² 死区
    if (abs(forward_accel) < ACCEL_DEADZONE) {
        forward_accel = 0.0;
    }
    
    // 4. IMU速度积分
    imu_integrated_speed += forward_accel * dt;
    
    // 5. 防止负速度（车辆不能倒退得太快）
    if (imu_integrated_speed < -2.0) {
        imu_integrated_speed = -2.0;
    }
    
    // 6. GPS-IMU融合（互补滤波）
    // 当GPS速度有效时，用GPS拉回IMU漂移
    if (gps_data.valid_speed && gps_data.satellites >= 4) {
        double gps_speed = gps_data.speed_kmh / 3.6; // 转换为m/s
        
        // 融合系数：GPS权重0.1，IMU权重0.9（每次GPS更新时）
        // 这意味着每100ms GPS会拉回10%的误差
        const double GPS_CORRECTION_RATE = 0.1;
        
        // 检测是否是新的GPS数据
        if (now - gps_data.last_gps_time > 80) { // 新的GPS数据（约100ms间隔）
            // 逐渐将IMU积分速度拉向GPS速度
            imu_integrated_speed = imu_integrated_speed * (1.0 - GPS_CORRECTION_RATE) 
                                  + gps_speed * GPS_CORRECTION_RATE;
            
            gps_data.last_gps_time = now;
            gps_data.last_gps_speed_ms = gps_speed;
        }
    }
    
    // 7. 静止时强制归零
    if (gps_data.speed_ms < 0.3 && abs(forward_accel) < 0.5) {
        imu_integrated_speed *= 0.95; // 逐渐衰减
        if (abs(imu_integrated_speed) < 0.1) {
            imu_integrated_speed = 0.0;
        }
    }
    
    // 8. 更新融合速度
    gps_data.fused_speed_ms = max(0.0, imu_integrated_speed);
    gps_data.smooth_speed_kmh = gps_data.fused_speed_ms * 3.6;
    gps_data.acceleration_ms2 = forward_accel;
    gps_data.speed_increasing = (forward_accel > 0.5);
    gps_data.imu_speed_delta = forward_accel * dt;
    
    last_fusion_time = now;
}

/**
 * 获取融合后的速度 (km/h)
 */
double getFusedSpeedKmh() {
    // 如果融合速度可用，返回融合速度；否则返回GPS速度
    if (gps_data.imu_calibrated && gps_data.fused_speed_ms > 0) {
        return gps_data.smooth_speed_kmh;
    }
    return gps_data.speed_kmh;
}

/**
 * 速度阈值穿越插值 - 用于精确测量0-100等事件
 * 
 * 原理：当检测到速度穿越目标阈值时，用线性插值计算精确穿越时刻
 * 这可以将10Hz GPS的~100ms不确定度降低到~10-20ms
 * 
 * @param target_speed_kmh 目标速度 (km/h)
 * @param crossing_time 输出：精确穿越时刻 (ms)
 * @return 穿越时的精确速度
 */
double interpolateSpeedAtTime(double target_speed_kmh, unsigned long* crossing_time) {
    static double prev_speed = 0.0;
    static unsigned long prev_time = 0;
    
    double current_speed = getFusedSpeedKmh();
    unsigned long current_time = millis();
    
    // 检测速度穿越
    if (prev_speed < target_speed_kmh && current_speed >= target_speed_kmh) {
        // 上升穿越 - 线性插值计算精确时刻
        double speed_diff = current_speed - prev_speed;
        double target_diff = target_speed_kmh - prev_speed;
        
        if (speed_diff > 0) {
            double ratio = target_diff / speed_diff;
            unsigned long time_diff = current_time - prev_time;
            *crossing_time = prev_time + (unsigned long)(time_diff * ratio);
            
            prev_speed = current_speed;
            prev_time = current_time;
            return target_speed_kmh;
        }
    }
    
    prev_speed = current_speed;
    prev_time = current_time;
    *crossing_time = 0; // 未穿越
    return current_speed;
}