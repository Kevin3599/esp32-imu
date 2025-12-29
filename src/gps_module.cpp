#include "gps_module.h"
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
 * 初始化GPS模块
 */
bool initGPS() {
    Serial.println("正在初始化GPS模块...");
    
    // 初始化GPS串口
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    // 等待GPS模块启动
    delay(1000);
    
    // 发送配置命令 (可选 - GT-U7通常默认配置就够用)
    // 设置更新频率为10Hz (100ms一次)
    gpsSerial.println("$PMTK220,100*2F");
    delay(100);
    
    // 设置输出的NMEA语句类型
    gpsSerial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
    delay(100);
    
    Serial.println("GPS模块初始化完成");
    Serial.printf("GPS引脚配置: RX=%d, TX=%d, 波特率=%d\n", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
    
    // 初始化性能数据
    resetPerformanceData();
    
    return true;
}

/**
 * 更新GPS数据
 */
void updateGPSData() {
    // 读取GPS数据
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            // 更新基础位置信息
            if (gps.location.isValid()) {
                gps_data.latitude = gps.location.lat();
                gps_data.longitude = gps.location.lng();
                gps_data.valid_location = true;
                gps_data.fix_age = gps.location.age();
            } else {
                gps_data.valid_location = false;
            }
            
            // 更新高度
            if (gps.altitude.isValid()) {
                gps_data.altitude = gps.altitude.meters();
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
            
            // 更新速度和方向
            if (gps.speed.isValid()) {
                gps_data.speed_kmh = gps.speed.kmph();
                gps_data.speed_mps = gps.speed.mps();
                gps_data.valid_speed = true;
                
                // 更新会话最高速度
                if (gps_data.speed_kmh > gps_data.max_speed_session) {
                    gps_data.max_speed_session = gps_data.speed_kmh;
                }
            } else {
                gps_data.valid_speed = false;
            }
            
            if (gps.course.isValid()) {
                gps_data.course = gps.course.deg();
            }
            
            // 更新卫星信息
            if (gps.satellites.isValid()) {
                gps_data.satellites = gps.satellites.value();
            }
            
            if (gps.hdop.isValid()) {
                gps_data.hdop = gps.hdop.hdop();
            }
        }
    }
    
    // 计算性能指标
    calculatePerformanceMetrics();
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