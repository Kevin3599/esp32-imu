/**
 * SD卡数据记录模块
 * 用于存储骑行/行驶数据到SD卡
 * 
 * SD卡接线 (HSPI):
 *   - MISO: GPIO12
 *   - MOSI: GPIO13
 *   - SCK:  GPIO14
 *   - CS:   GPIO15
 *   - VCC:  3.3V 或 5V
 *   - GND:  GND
 */

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// SD卡引脚定义 (HSPI)
// MISO使用GPIO25，避免GPIO12的启动冲突
#define SD_MISO  25    // GPIO25
#define SD_MOSI  13
#define SD_SCK   14
#define SD_CS    15

// 数据记录结构
struct RideDataPoint {
    unsigned long timestamp;      // 毫秒时间戳
    // GPS数据
    double latitude;
    double longitude;
    float speed_kmh;              // GPS速度 km/h
    float altitude;               // 海拔 m
    uint8_t satellites;           // 卫星数
    float hdop;                   // 水平精度因子
    // IMU数据
    float accel_x, accel_y, accel_z;  // 加速度 m/s²
    float gyro_x, gyro_y, gyro_z;     // 角速度 °/s
    float roll, pitch;                 // 姿态角 °
    // 性能数据
    float forward_g;              // 纵向G值
    float lateral_g;              // 横向G值
    float fused_speed;            // 融合速度 km/h
};

// SD卡状态
struct SDStatus {
    bool initialized;             // SD卡是否初始化成功
    bool logging;                 // 是否正在记录
    uint64_t total_bytes;         // SD卡总容量
    uint64_t used_bytes;          // 已用容量
    uint64_t free_bytes;          // 剩余容量
    String current_file;          // 当前记录文件名
    unsigned long data_points;    // 已记录数据点数
    unsigned long file_size;      // 当前文件大小
    unsigned long start_time;     // 开始记录时间
};

// 全局SD卡状态
extern SDStatus sd_status;

/**
 * 初始化SD卡
 * @return true=成功, false=失败
 */
bool initSDCard();

/**
 * 开始记录数据
 * 自动创建新的CSV文件
 * @return true=成功, false=失败
 */
bool startLogging();

/**
 * 停止记录数据
 */
void stopLogging();

/**
 * 记录一个数据点
 * @param data 数据点
 * @return true=成功, false=失败
 */
bool logDataPoint(const RideDataPoint& data);

/**
 * 获取SD卡状态
 */
SDStatus getSDStatus();

/**
 * 列出SD卡上的所有记录文件
 */
void listLogFiles();

/**
 * 删除指定文件
 * @param filename 文件名
 * @return true=成功, false=失败
 */
bool deleteLogFile(const char* filename);

/**
 * 获取SD卡剩余空间 (MB)
 */
float getSDFreeSpaceMB();

/**
 * 检查是否应该自动开始/停止记录
 * 基于速度判断骑行状态
 * @param speed_kmh 当前速度
 */
void checkAutoLogging(float speed_kmh);

#endif // SD_LOGGER_H
