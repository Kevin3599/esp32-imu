#include "audio_recorder.h"
#include "gps_module.h"
#include <esp_timer.h>

// 全局录音数据
AUDIO_RECORDER_DATA audio_data;

// 录音文件句柄
static File audioFile;

// 音频缓冲区
static uint8_t audioBuffer[AUDIO_BUFFER_SIZE];
static uint16_t bufferIndex = 0;

// 采样定时器
static hw_timer_t* sampleTimer = NULL;
static volatile bool sampleReady = false;
static volatile uint16_t lastSample = 0;

// WAV文件头结构
struct WAVHeader {
    char riff[4] = {'R', 'I', 'F', 'F'};
    uint32_t fileSize = 0;
    char wave[4] = {'W', 'A', 'V', 'E'};
    char fmt[4] = {'f', 'm', 't', ' '};
    uint32_t fmtSize = 16;
    uint16_t audioFormat = 1;  // PCM
    uint16_t numChannels = 1;  // Mono
    uint32_t sampleRate = SAMPLE_RATE;
    uint32_t byteRate = SAMPLE_RATE;  // sampleRate * numChannels * bitsPerSample/8
    uint16_t blockAlign = 1;   // numChannels * bitsPerSample/8
    uint16_t bitsPerSample = BITS_PER_SAMPLE;
    char data[4] = {'d', 'a', 't', 'a'};
    uint32_t dataSize = 0;
};

// 采样计数器
static unsigned long sampleCount = 0;
static unsigned long lastSampleTime = 0;

// 自动停止计时器
static unsigned long lowSpeedStartTime = 0;
static bool waitingToStop = false;

/**
 * 定时器中断 - 采集音频样本
 */
void IRAM_ATTR onSampleTimer() {
    lastSample = adc1_get_raw(MIC_ADC_CHANNEL);
    sampleReady = true;
}

/**
 * 初始化录音模块
 */
void initAudioRecorder() {
    Serial.println("正在初始化录音模块...");
    
    // 配置ADC
    adc1_config_width(ADC_WIDTH_BIT_12);  // 12位ADC
    adc1_config_channel_atten(MIC_ADC_CHANNEL, ADC_ATTEN_DB_12);  // 0-3.3V范围
    
    // 读取测试采样
    int testSample = adc1_get_raw(MIC_ADC_CHANNEL);
    Serial.printf("麦克风测试采样: %d (GPIO34)\n", testSample);
    
    // 更新存储信息
    updateStorageInfo();
    
    // 初始状态
    audio_data.state = RECORDING_IDLE;
    audio_data.auto_record_on_ride = true;
    
    Serial.printf("录音模块初始化完成\n");
    Serial.printf("  采样率: %d Hz\n", SAMPLE_RATE);
    Serial.printf("  位深: %d bit\n", BITS_PER_SAMPLE);
    Serial.printf("  SPIFFS 可用: %lu KB\n", audio_data.spiffs_free / 1024);
    Serial.printf("  自动骑行录音: %s\n", audio_data.auto_record_on_ride ? "开启" : "关闭");
}

/**
 * 生成新的录音文件名
 */
void generateFilename() {
    // 使用时间戳生成文件名
    unsigned long timestamp = millis() / 1000;
    sprintf(audio_data.filename, "/ride_%lu.wav", timestamp);
}

/**
 * 写入WAV文件头
 */
bool writeWAVHeader(File& file) {
    WAVHeader header;
    header.sampleRate = SAMPLE_RATE;
    header.byteRate = SAMPLE_RATE * 1 * BITS_PER_SAMPLE / 8;
    header.blockAlign = 1 * BITS_PER_SAMPLE / 8;
    header.bitsPerSample = BITS_PER_SAMPLE;
    header.dataSize = 0;  // 初始为0，结束时更新
    header.fileSize = 36; // 初始为0，结束时更新
    
    file.seek(0);
    size_t written = file.write((uint8_t*)&header, sizeof(WAVHeader));
    return written == sizeof(WAVHeader);
}

/**
 * 更新WAV文件头中的大小信息
 */
void updateWAVHeader(File& file, uint32_t dataSize) {
    WAVHeader header;
    header.sampleRate = SAMPLE_RATE;
    header.byteRate = SAMPLE_RATE * 1 * BITS_PER_SAMPLE / 8;
    header.blockAlign = 1 * BITS_PER_SAMPLE / 8;
    header.bitsPerSample = BITS_PER_SAMPLE;
    header.dataSize = dataSize;
    header.fileSize = dataSize + 36;
    
    file.seek(0);
    file.write((uint8_t*)&header, sizeof(WAVHeader));
}

/**
 * 开始录音
 */
bool startRecording() {
    if (audio_data.state == RECORDING_ACTIVE) {
        Serial.println("已在录音中");
        return true;
    }
    
    // 检查存储空间
    updateStorageInfo();
    if (audio_data.spiffs_free < 50000) {  // 至少需要50KB
        audio_data.error = true;
        strcpy(audio_data.error_msg, "存储空间不足");
        Serial.println("❌ 录音失败: 存储空间不足");
        return false;
    }
    
    // 生成文件名
    generateFilename();
    
    // 创建文件
    audioFile = SPIFFS.open(audio_data.filename, FILE_WRITE);
    if (!audioFile) {
        audio_data.error = true;
        strcpy(audio_data.error_msg, "无法创建文件");
        Serial.printf("❌ 无法创建录音文件: %s\n", audio_data.filename);
        return false;
    }
    
    // 写入WAV头
    if (!writeWAVHeader(audioFile)) {
        audioFile.close();
        audio_data.error = true;
        strcpy(audio_data.error_msg, "写入文件头失败");
        return false;
    }
    
    // 重置状态
    audio_data.samples_recorded = 0;
    audio_data.file_size = sizeof(WAVHeader);
    audio_data.start_time = millis();
    audio_data.duration_ms = 0;
    audio_data.peak_volume = 0;
    audio_data.error = false;
    bufferIndex = 0;
    sampleCount = 0;
    waitingToStop = false;
    
    // 启动采样定时器 (8000Hz = 125us间隔)
    sampleTimer = timerBegin(0, 80, true);  // 80分频 = 1MHz
    timerAttachInterrupt(sampleTimer, &onSampleTimer, true);
    timerAlarmWrite(sampleTimer, 125, true);  // 125us = 8kHz
    timerAlarmEnable(sampleTimer);
    
    audio_data.state = RECORDING_ACTIVE;
    
    Serial.printf("🎤 开始录音: %s\n", audio_data.filename);
    return true;
}

/**
 * 停止录音
 */
void stopRecording() {
    if (audio_data.state != RECORDING_ACTIVE && audio_data.state != RECORDING_PAUSED) {
        return;
    }
    
    // 停止定时器
    if (sampleTimer != NULL) {
        timerAlarmDisable(sampleTimer);
        timerDetachInterrupt(sampleTimer);
        timerEnd(sampleTimer);
        sampleTimer = NULL;
    }
    
    // 写入剩余缓冲区
    if (bufferIndex > 0 && audioFile) {
        audioFile.write(audioBuffer, bufferIndex);
        audio_data.file_size += bufferIndex;
    }
    
    // 更新WAV头
    if (audioFile) {
        uint32_t dataSize = audio_data.samples_recorded;
        updateWAVHeader(audioFile, dataSize);
        audioFile.close();
    }
    
    audio_data.state = RECORDING_STOPPED;
    audio_data.duration_ms = millis() - audio_data.start_time;
    
    Serial.printf("⏹️ 录音结束: %s\n", audio_data.filename);
    Serial.printf("   时长: %.1f 秒\n", audio_data.duration_ms / 1000.0);
    Serial.printf("   大小: %lu 字节\n", audio_data.file_size);
    Serial.printf("   采样数: %lu\n", audio_data.samples_recorded);
    
    // 更新存储信息
    updateStorageInfo();
}

/**
 * 暂停录音
 */
void pauseRecording() {
    if (audio_data.state == RECORDING_ACTIVE) {
        if (sampleTimer != NULL) {
            timerAlarmDisable(sampleTimer);
        }
        audio_data.state = RECORDING_PAUSED;
        Serial.println("⏸️ 录音暂停");
    }
}

/**
 * 恢复录音
 */
void resumeRecording() {
    if (audio_data.state == RECORDING_PAUSED) {
        if (sampleTimer != NULL) {
            timerAlarmEnable(sampleTimer);
        }
        audio_data.state = RECORDING_ACTIVE;
        Serial.println("▶️ 录音恢复");
    }
}

/**
 * 处理音频采样
 */
void processSample() {
    if (!sampleReady || audio_data.state != RECORDING_ACTIVE) {
        return;
    }
    sampleReady = false;
    
    // 12位ADC转8位
    uint8_t sample8bit = lastSample >> 4;  // 4096 -> 256
    
    // 更新音量指示
    int16_t centered = (int16_t)sample8bit - 128;
    audio_data.current_volume = abs(centered);
    if (audio_data.current_volume > audio_data.peak_volume) {
        audio_data.peak_volume = audio_data.current_volume;
    }
    
    // 添加到缓冲区
    audioBuffer[bufferIndex++] = sample8bit;
    audio_data.samples_recorded++;
    
    // 缓冲区满时写入文件
    if (bufferIndex >= AUDIO_BUFFER_SIZE) {
        if (audioFile) {
            audioFile.write(audioBuffer, AUDIO_BUFFER_SIZE);
            audio_data.file_size += AUDIO_BUFFER_SIZE;
        }
        bufferIndex = 0;
    }
    
    // 更新时长
    audio_data.duration_ms = millis() - audio_data.start_time;
    
    // 检查最大录音时长
    if (audio_data.duration_ms > MAX_RECORDING_TIME * 1000) {
        Serial.println("⚠️ 达到最大录音时长，自动停止");
        stopRecording();
    }
}

/**
 * 检查是否应该自动开始/停止录音
 */
void checkAutoRecordTrigger() {
    if (!audio_data.auto_record_on_ride) {
        return;
    }
    
    // 获取当前速度
    float current_speed = 0.0;
    if (gps_data.valid_speed) {
        current_speed = gps_data.speed_kmh;
    }
    
    // 状态机: 自动录音控制
    switch (audio_data.state) {
        case RECORDING_IDLE:
        case RECORDING_STOPPED:
            // 等待开始骑行
            if (current_speed >= audio_data.ride_start_speed) {
                Serial.printf("🏍️ 检测到骑行开始 (%.1f km/h)，自动开始录音\n", current_speed);
                startRecording();
            }
            break;
            
        case RECORDING_ACTIVE:
            // 检查是否应该停止
            if (current_speed < audio_data.ride_stop_speed) {
                if (!waitingToStop) {
                    waitingToStop = true;
                    lowSpeedStartTime = millis();
                    Serial.printf("⏳ 速度降低 (%.1f km/h)，%lu秒后自动停止录音\n", 
                                 current_speed, audio_data.stop_delay_ms / 1000);
                } else {
                    // 检查延迟时间
                    if (millis() - lowSpeedStartTime > audio_data.stop_delay_ms) {
                        Serial.println("🛑 骑行结束，自动停止录音");
                        stopRecording();
                    }
                }
            } else {
                // 速度恢复，取消停止计时
                if (waitingToStop) {
                    waitingToStop = false;
                    Serial.println("▶️ 速度恢复，继续录音");
                }
            }
            break;
            
        default:
            break;
    }
}

/**
 * 更新录音模块 - 在主循环中调用
 */
void updateAudioRecorder() {
    // 处理音频采样
    processSample();
    
    // 检查自动录音触发 (每500ms检查一次)
    static unsigned long lastCheckTime = 0;
    if (millis() - lastCheckTime > 500) {
        checkAutoRecordTrigger();
        lastCheckTime = millis();
    }
}

/**
 * 更新存储空间信息
 */
void updateStorageInfo() {
    audio_data.spiffs_total = SPIFFS.totalBytes();
    audio_data.spiffs_used = SPIFFS.usedBytes();
    audio_data.spiffs_free = audio_data.spiffs_total - audio_data.spiffs_used;
}

/**
 * 删除所有录音文件
 */
void deleteAllRecordings() {
    Serial.println("正在删除所有录音文件...");
    
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    int count = 0;
    
    while (file) {
        String filename = file.name();
        file.close();
        
        if (filename.startsWith("/ride_") && filename.endsWith(".wav")) {
            SPIFFS.remove(filename);
            Serial.printf("  删除: %s\n", filename.c_str());
            count++;
        }
        file = root.openNextFile();
    }
    
    Serial.printf("已删除 %d 个录音文件\n", count);
    updateStorageInfo();
}

/**
 * 获取录音文件列表 (JSON格式)
 */
String getRecordingsList() {
    String json = "[";
    bool first = true;
    
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    
    while (file) {
        String filename = file.name();
        if (filename.startsWith("/ride_") && filename.endsWith(".wav")) {
            if (!first) json += ",";
            first = false;
            
            json += "{\"name\":\"";
            json += filename;
            json += "\",\"size\":";
            json += String(file.size());
            json += "}";
        }
        file.close();
        file = root.openNextFile();
    }
    
    json += "]";
    return json;
}
