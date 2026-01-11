#ifndef AUDIO_RECORDER_H
#define AUDIO_RECORDER_H

#include <Arduino.h>
#include <SPIFFS.h>
#include <driver/adc.h>

// ===== 麦克风硬件配置 =====
#define MIC_PIN 34              // 麦克风OUT接GPIO34 (ADC1_CH6)
#define MIC_ADC_CHANNEL ADC1_CHANNEL_6

// ===== 录音参数配置 =====
#define SAMPLE_RATE 8000        // 采样率 8kHz (节省存储空间)
#define BITS_PER_SAMPLE 8       // 8位采样 (节省存储)
#define AUDIO_BUFFER_SIZE 512   // 录音缓冲区大小
#define MAX_RECORDING_TIME 300  // 最大录音时长(秒) - 5分钟

// ===== 录音状态枚举 =====
enum RecordingState {
    RECORDING_IDLE,         // 空闲
    RECORDING_WAITING,      // 等待开始骑行
    RECORDING_ACTIVE,       // 正在录音
    RECORDING_PAUSED,       // 暂停
    RECORDING_STOPPED       // 已停止
};

// ===== 录音数据结构 =====
struct AUDIO_RECORDER_DATA {
    RecordingState state = RECORDING_IDLE;
    
    // 录音文件信息
    char filename[32] = "";
    unsigned long file_size = 0;
    unsigned long samples_recorded = 0;
    
    // 时间信息
    unsigned long start_time = 0;
    unsigned long duration_ms = 0;
    
    // 音量指示
    uint16_t current_volume = 0;
    uint16_t peak_volume = 0;
    
    // 自动录音配置
    bool auto_record_on_ride = true;     // 骑行时自动录音
    float ride_start_speed = 5.0;        // 开始骑行的速度阈值 (km/h)
    float ride_stop_speed = 2.0;         // 停止骑行的速度阈值 (km/h)
    unsigned long stop_delay_ms = 5000;  // 停车后延迟停止录音 (5秒)
    
    // 存储空间
    unsigned long spiffs_total = 0;
    unsigned long spiffs_used = 0;
    unsigned long spiffs_free = 0;
    
    // 错误信息
    bool error = false;
    char error_msg[64] = "";
};

// 全局录音数据
extern AUDIO_RECORDER_DATA audio_data;

// ===== 函数声明 =====

/**
 * 初始化录音模块
 */
void initAudioRecorder();

/**
 * 开始录音
 * @return true 成功开始录音
 */
bool startRecording();

/**
 * 停止录音
 */
void stopRecording();

/**
 * 暂停/恢复录音
 */
void pauseRecording();
void resumeRecording();

/**
 * 更新录音 - 在主循环中调用
 * 处理自动开始/停止和音频采样
 */
void updateAudioRecorder();

/**
 * 检查是否应该自动开始录音
 * 根据GPS速度判断是否开始骑行
 */
void checkAutoRecordTrigger();

/**
 * 获取SPIFFS存储空间信息
 */
void updateStorageInfo();

/**
 * 删除所有录音文件
 */
void deleteAllRecordings();

/**
 * 获取录音文件列表
 */
String getRecordingsList();

/**
 * 生成新的录音文件名
 */
void generateFilename();

#endif
