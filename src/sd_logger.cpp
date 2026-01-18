/**
 * SD卡数据记录模块 - 实现
 */

#include "sd_logger.h"
#include <FS.h>

// 使用HSPI而非默认VSPI，避免与其他SPI设备冲突
SPIClass sdSPI(HSPI);

// 全局状态
SDStatus sd_status = {false, false, 0, 0, 0, "", 0, 0, 0};

// 当前记录文件
File logFile;

// 自动记录状态
static bool auto_log_enabled = true;
static unsigned long last_motion_time = 0;
static const unsigned long STOP_DELAY = 30000;  // 停止30秒后自动结束记录
static const float START_SPEED = 5.0;           // 速度>5km/h开始记录
static const float STOP_SPEED = 2.0;            // 速度<2km/h判定为停止

/**
 * 初始化SD卡
 */
bool initSDCard() {
    Serial.println("正在初始化SD卡...");
    Serial.printf("SD卡引脚: SCK=%d, MISO=%d, MOSI=%d, CS=%d\n", SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    
    // 配置HSPI - 参数顺序: SCK, MISO, MOSI, SS(-1表示不用硬件SS)
    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, -1);
    
    // 设置CS引脚
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    
    // 初始化SD卡
    if (!SD.begin(SD_CS, sdSPI, 4000000)) {  // 4MHz SPI速度
        Serial.println("❌ SD卡初始化失败!");
        Serial.println("   请检查:");
        Serial.println("   1. SD卡是否插入");
        Serial.println("   2. SD卡格式是否为FAT32");
        Serial.printf("   3. 接线: MISO=%d, MOSI=%d, SCK=%d, CS=%d\n", SD_MISO, SD_MOSI, SD_SCK, SD_CS);
        sd_status.initialized = false;
        return false;
    }
    
    // 获取SD卡信息
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("❌ 未检测到SD卡!");
        sd_status.initialized = false;
        return false;
    }
    
    // 显示卡类型
    Serial.print("✅ SD卡类型: ");
    switch (cardType) {
        case CARD_MMC:  Serial.println("MMC"); break;
        case CARD_SD:   Serial.println("SDSC"); break;
        case CARD_SDHC: Serial.println("SDHC"); break;
        default:        Serial.println("未知"); break;
    }
    
    // 获取容量信息
    sd_status.total_bytes = SD.totalBytes();
    sd_status.used_bytes = SD.usedBytes();
    sd_status.free_bytes = sd_status.total_bytes - sd_status.used_bytes;
    sd_status.initialized = true;
    
    Serial.printf("   总容量: %.2f GB\n", sd_status.total_bytes / (1024.0 * 1024.0 * 1024.0));
    Serial.printf("   已用: %.2f MB\n", sd_status.used_bytes / (1024.0 * 1024.0));
    Serial.printf("   剩余: %.2f GB\n", sd_status.free_bytes / (1024.0 * 1024.0 * 1024.0));
    
    // 确保logs目录存在
    if (!SD.exists("/logs")) {
        SD.mkdir("/logs");
        Serial.println("   创建 /logs 目录");
    }
    
    Serial.println("✅ SD卡初始化完成");
    return true;
}

/**
 * 生成唯一文件名
 */
String generateFileName() {
    // 使用启动后的毫秒数作为唯一标识
    unsigned long ms = millis();
    char filename[32];
    snprintf(filename, sizeof(filename), "/logs/ride_%lu.csv", ms / 1000);
    return String(filename);
}

/**
 * 开始记录
 */
bool startLogging() {
    if (!sd_status.initialized) {
        Serial.println("❌ SD卡未初始化，无法开始记录");
        return false;
    }
    
    if (sd_status.logging) {
        Serial.println("⚠️ 已在记录中");
        return true;
    }
    
    // 生成新文件名
    sd_status.current_file = generateFileName();
    
    // 创建文件并写入CSV头
    logFile = SD.open(sd_status.current_file, FILE_WRITE);
    if (!logFile) {
        Serial.printf("❌ 无法创建文件: %s\n", sd_status.current_file.c_str());
        return false;
    }
    
    // 写入CSV头
    logFile.println("timestamp,lat,lon,speed_gps,altitude,satellites,hdop,"
                    "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll,pitch,"
                    "forward_g,lateral_g,fused_speed");
    logFile.flush();
    
    sd_status.logging = true;
    sd_status.data_points = 0;
    sd_status.file_size = logFile.size();
    sd_status.start_time = millis();
    
    Serial.printf("🔴 开始记录: %s\n", sd_status.current_file.c_str());
    return true;
}

/**
 * 停止记录
 */
void stopLogging() {
    if (!sd_status.logging) {
        return;
    }
    
    // 关闭文件
    if (logFile) {
        logFile.flush();
        sd_status.file_size = logFile.size();
        logFile.close();
    }
    
    unsigned long duration = (millis() - sd_status.start_time) / 1000;
    
    Serial.println("⬛ 停止记录");
    Serial.printf("   文件: %s\n", sd_status.current_file.c_str());
    Serial.printf("   数据点: %lu\n", sd_status.data_points);
    Serial.printf("   文件大小: %.2f KB\n", sd_status.file_size / 1024.0);
    Serial.printf("   时长: %lu 秒\n", duration);
    
    sd_status.logging = false;
    
    // 更新剩余空间
    sd_status.used_bytes = SD.usedBytes();
    sd_status.free_bytes = sd_status.total_bytes - sd_status.used_bytes;
}

/**
 * 记录一个数据点
 */
bool logDataPoint(const RideDataPoint& data) {
    if (!sd_status.logging || !logFile) {
        return false;
    }
    
    // 写入CSV行
    char buffer[256];
    snprintf(buffer, sizeof(buffer),
             "%lu,%.6f,%.6f,%.1f,%.1f,%d,%.2f,"
             "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,"
             "%.2f,%.2f,%.1f",
             data.timestamp,
             data.latitude, data.longitude,
             data.speed_kmh, data.altitude,
             data.satellites, data.hdop,
             data.accel_x, data.accel_y, data.accel_z,
             data.gyro_x, data.gyro_y, data.gyro_z,
             data.roll, data.pitch,
             data.forward_g, data.lateral_g,
             data.fused_speed);
    
    logFile.println(buffer);
    sd_status.data_points++;
    
    // 每100个点刷新一次
    if (sd_status.data_points % 100 == 0) {
        logFile.flush();
        sd_status.file_size = logFile.size();
    }
    
    return true;
}

/**
 * 获取SD卡状态
 */
SDStatus getSDStatus() {
    return sd_status;
}

/**
 * 列出所有记录文件
 */
void listLogFiles() {
    if (!sd_status.initialized) {
        Serial.println("SD卡未初始化");
        return;
    }
    
    Serial.println("\n=== SD卡记录文件 ===");
    
    File root = SD.open("/logs");
    if (!root || !root.isDirectory()) {
        Serial.println("无法打开 /logs 目录");
        return;
    }
    
    int fileCount = 0;
    unsigned long totalSize = 0;
    
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            Serial.printf("  %s  (%.2f KB)\n", file.name(), file.size() / 1024.0);
            fileCount++;
            totalSize += file.size();
        }
        file = root.openNextFile();
    }
    
    Serial.printf("共 %d 个文件, 总大小 %.2f MB\n", fileCount, totalSize / (1024.0 * 1024.0));
    Serial.printf("SD卡剩余: %.2f GB\n", sd_status.free_bytes / (1024.0 * 1024.0 * 1024.0));
    Serial.println("====================\n");
}

/**
 * 删除记录文件
 */
bool deleteLogFile(const char* filename) {
    if (!sd_status.initialized) {
        return false;
    }
    
    String path = String("/logs/") + filename;
    if (SD.exists(path)) {
        if (SD.remove(path)) {
            Serial.printf("已删除: %s\n", filename);
            sd_status.used_bytes = SD.usedBytes();
            sd_status.free_bytes = sd_status.total_bytes - sd_status.used_bytes;
            return true;
        }
    }
    
    Serial.printf("删除失败: %s\n", filename);
    return false;
}

/**
 * 获取SD卡剩余空间 (MB)
 */
float getSDFreeSpaceMB() {
    if (!sd_status.initialized) {
        return 0;
    }
    return sd_status.free_bytes / (1024.0 * 1024.0);
}

/**
 * 自动记录检测
 * 速度>5km/h开始记录，停止30秒后结束记录
 */
void checkAutoLogging(float speed_kmh) {
    if (!sd_status.initialized || !auto_log_enabled) {
        return;
    }
    
    unsigned long now = millis();
    
    if (speed_kmh > START_SPEED) {
        // 有运动
        last_motion_time = now;
        
        if (!sd_status.logging) {
            // 开始记录
            startLogging();
        }
    } else if (speed_kmh < STOP_SPEED) {
        // 速度很低
        if (sd_status.logging && (now - last_motion_time > STOP_DELAY)) {
            // 停止超过30秒，结束记录
            stopLogging();
        }
    }
}
