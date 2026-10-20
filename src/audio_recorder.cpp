#include "audio_recorder.h"
#include "gps_module.h"
#include <driver/i2s.h>
#include "rtos_runtime.h"

// 全局录音数据
AUDIO_RECORDER_DATA audio_data;

// 录音文件句柄
static File audioFile;

// 音频缓冲区
static uint8_t audioBuffer[AUDIO_BUFFER_SIZE];
static uint16_t bufferIndex = 0;

// I2S0 ADC DMA owns ADC1 during recording; no ADC calls in an ISR.
static bool dmaInstalled = false;
static QueueHandle_t dmaEvents = nullptr;
static bool startDma() {
    i2s_config_t config{};
    config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
    config.sample_rate = SAMPLE_RATE;
    config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    config.communication_format = I2S_COMM_FORMAT_STAND_MSB;
    config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    config.dma_buf_count = 8;
    config.dma_buf_len = 256;
    if (i2s_driver_install(I2S_NUM_0, &config, 16, &dmaEvents) != ESP_OK) return false;
    dmaInstalled = true;
    if (i2s_set_adc_mode(ADC_UNIT_1, MIC_ADC_CHANNEL) != ESP_OK || i2s_adc_enable(I2S_NUM_0) != ESP_OK) {
        i2s_driver_uninstall(I2S_NUM_0); dmaInstalled = false; dmaEvents = nullptr; return false;
    }
    return true;
}
static void stopDma() {
    if (!dmaInstalled) return;
    i2s_adc_disable(I2S_NUM_0);
    i2s_driver_uninstall(I2S_NUM_0);
    dmaInstalled = false; dmaEvents = nullptr;
}

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
    snprintf(audio_data.filename, sizeof(audio_data.filename), "/ride_%08lx_%08lx.wav", (unsigned long)millis(), (unsigned long)esp_random());
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
    if (file.write((uint8_t*)&header, sizeof(WAVHeader)) != sizeof(WAVHeader)) {
        ++audio_data.write_errors; audio_data.error = true;
        strcpy(audio_data.error_msg, "WAV header write failed");
    }
}

/**
 * 开始录音
 */
bool startRecording() {
    if (audio_data.state == RECORDING_PAUSED) { resumeRecording(); return audio_data.state == RECORDING_ACTIVE; }
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
    
    audio_data.dma_overflows = 0;
    audio_data.write_errors = 0;
    if (!startDma()) {
        audioFile.close();
        audio_data.error = true;
        strcpy(audio_data.error_msg, "ADC DMA initialization failed");
        return false;
    }

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
    
    stopDma();

    // 写入剩余缓冲区
    if (bufferIndex > 0 && audioFile) {
        const size_t written = audioFile.write(audioBuffer, bufferIndex);
        if (written != bufferIndex) { ++audio_data.write_errors; audio_data.error = true; strcpy(audio_data.error_msg, "Audio write failed"); }
        audio_data.file_size += written;
        bufferIndex = 0;
    }
    
    // 更新WAV头
    if (audioFile) {
        uint32_t dataSize = audioFile.size() >= sizeof(WAVHeader) ? audioFile.size() - sizeof(WAVHeader) : 0;
        audio_data.samples_recorded = dataSize;
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
        stopDma();
        audio_data.state = RECORDING_PAUSED;
        Serial.println("⏸️ 录音暂停");
    }
}

/**
 * 恢复录音
 */
void resumeRecording() {
    if (audio_data.state == RECORDING_PAUSED) {
        if (!startDma()) {
            audio_data.error = true;
            strcpy(audio_data.error_msg, "ADC DMA resume failed");
            return;
        }
        audio_data.state = RECORDING_ACTIVE;
        Serial.println("▶️ 录音恢复");
    }
}

/**
 * 处理音频采样
 */
void processSample() {
    if (audio_data.state != RECORDING_ACTIVE || !dmaInstalled) return;
    i2s_event_t event;
    while (xQueueReceive(dmaEvents, &event, 0) == pdTRUE) {
        if (event.type == I2S_EVENT_RX_Q_OVF || event.type == I2S_EVENT_DMA_ERROR) ++audio_data.dma_overflows;
    }
    uint16_t samples[256];
    // Bounded drain: other storage commands and log records must also run.
    for (int block = 0; block < 8; ++block) {
        size_t bytes = 0;
        if (i2s_read(I2S_NUM_0, samples, sizeof(samples), &bytes, 0) != ESP_OK || bytes == 0) break;
        for (size_t i = 0; i < bytes / sizeof(uint16_t); ++i) {
            const uint8_t value = (samples[i] & 0x0fff) >> 4;
            audio_data.current_volume = abs(int(value) - 128);
            audio_data.peak_volume = max(audio_data.peak_volume, audio_data.current_volume);
            audioBuffer[bufferIndex++] = value;
            ++audio_data.samples_recorded;
            if (bufferIndex == AUDIO_BUFFER_SIZE) {
                size_t written = audioFile.write(audioBuffer, bufferIndex);
                audio_data.file_size += written;
                bufferIndex = 0;
                if (written != AUDIO_BUFFER_SIZE) {
                    ++audio_data.write_errors; audio_data.error = true;
                    strcpy(audio_data.error_msg, "Audio write failed");
                    stopRecording(); return;
                }
            }
        }
    }
    audio_data.duration_ms = millis() - audio_data.start_time;
    if (audio_data.duration_ms > MAX_RECORDING_TIME * 1000) stopRecording();
}

/**
 * 检查是否应该自动开始/停止录音
 */
void checkAutoRecordTrigger() {
    if (!audio_data.auto_record_on_ride) {
        return;
    }
    
    // 获取当前速度
    const auto snapshot = readTelemetry();
    float current_speed = snapshot.gps.valid_speed && millis() - snapshot.publishedMs < 1500 ? snapshot.gps.speed_kmh : 0;
    
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
