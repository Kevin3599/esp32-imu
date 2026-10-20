#include "rtos_runtime.h"
#include "sd_logger.h"
#include "measurement.h"
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <atomic>
#include <type_traits>

namespace {
constexpr EventBits_t Ready = BIT0;
constexpr size_t ImuDepth = 32, LogDepth = 32;
struct ImuSample { MPU6050_DATA data; uint32_t timestampUs; };
struct LogSample { RideDataPoint data; bool validSpeed; };
struct Session { GPS_DATA gps; PERFORMANCE_DATA performance; };
static_assert(std::is_trivially_copyable<ImuSample>::value, "Queue copies bytes");
static_assert(std::is_trivially_copyable<LogSample>::value, "Queue copies bytes");
static_assert(std::is_trivially_copyable<Session>::value, "Queue copies bytes");
QueueHandle_t imuQueue, logQueue, commandQueue, sessionQueue, modelCommandQueue;
SemaphoreHandle_t snapshotMutex, filesystemMutex;
EventGroupHandle_t startup;
TaskHandle_t tasks[5] = {};
TelemetrySnapshot telemetry;
StorageSnapshot storage;
std::atomic<uint32_t> imuDropped{0}, logDropped{0}, commandsRejected{0};
bool manualLogging = false;

void waitReady() { xEventGroupWaitBits(startup, Ready, pdFALSE, pdTRUE, portMAX_DELAY); }
void periodicWait(TickType_t& wake, TickType_t period) {
    // Do not perform a burst of catch-up reads after an overrun.
    if (TickType_t(xTaskGetTickCount() - wake) >= period) wake = xTaskGetTickCount();
    vTaskDelayUntil(&wake, period);
}
void imuTask(void*) {
    waitReady();
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        ImuSample sample{};
        if (ReadMPU6050(sample.data)) {
            sample.timestampUs = micros();
            if (xQueueSend(imuQueue, &sample, 0) != pdTRUE) ++imuDropped;
        }
        periodicWait(wake, pdMS_TO_TICKS(5));
    }
}
void modelTask(void*) {
    waitReady();
    RuntimeState state;
    uint32_t previousUs = 0, lastLog = 0, lastPublish = 0;
    uint32_t previousChars = 0;
    uint32_t samples = 0, intervalUs = 0, overruns = 0;
    for (;;) {
        updateGPSData(); // UART and all GPS/parser state have one owner.
        const uint32_t chars = gps.charsProcessed();
        const bool gpsBytes = chars != previousChars;
        previousChars = chars;
        bool imuReceived = false;
        uint8_t command;
        if (xQueueReceive(modelCommandQueue, &command, 0) == pdTRUE) resetMotorcycleSession();
        ImuSample sample;
        // Bounded processing prevents IMU backlog from starving UART handling.
        for (int i = 0; i < 8 && xQueueReceive(imuQueue, &sample, i == 0 ? pdMS_TO_TICKS(2) : 0) == pdTRUE; ++i) {
            intervalUs = previousUs ? uint32_t(sample.timestampUs - previousUs) : 0;
            if (intervalUs > 10000) ++overruns;
            previousUs = sample.timestampUs;
            imuReceived = true;
            ++samples;
            mpu6050_data = sample.data;
            const double dt = intervalUs / 1000000.0;
            updateMotorcycleData(dt);
            fuseIMUSpeed(measurement::forward(sample.data.Acc_Z), measurement::lateral(sample.data.Acc_Y), dt);
        }
        updateFusedData();
        const uint32_t now = millis();
        state.observe(now, gpsBytes, gps_data.valid_location, gps_data.valid_speed,
                      gps_data.raw_speed_kmh, imuReceived);
        if (now - lastLog >= 100) {
            lastLog = now;
            LogSample log{};
            auto& p = log.data;
            p.timestamp = now;
            p.latitude = gps_data.latitude; p.longitude = gps_data.longitude;
            p.speed_kmh = gps_data.raw_speed_kmh; p.altitude = gps_data.altitude;
            p.satellites = gps_data.satellites; p.hdop = gps_data.hdop;
            p.accel_x = mpu6050_data.Acc_X; p.accel_y = mpu6050_data.Acc_Y; p.accel_z = mpu6050_data.Acc_Z;
            p.gyro_x = mpu6050_data.Angle_Velocity_R * measurement::radToDeg;
            p.gyro_y = mpu6050_data.Angle_Velocity_P * measurement::radToDeg;
            p.gyro_z = mpu6050_data.Angle_Velocity_Y * measurement::radToDeg;
            p.roll = moto_data.lean_angle_filtered; p.pitch = moto_data.wheelie_angle;
            p.forward_g = fused_data.current_accel_g; p.lateral_g = fused_data.current_lateral_g;
            p.fused_speed = getFusedSpeedKmh();
            log.validSpeed = gps_data.valid_speed;
            if (xQueueSend(logQueue, &log, 0) != pdTRUE) ++logDropped;
        }
        if (now - lastPublish >= 10) {
            lastPublish = now;
            xSemaphoreTake(snapshotMutex, portMAX_DELAY);
            telemetry.imu = mpu6050_data; telemetry.gps = gps_data;
            telemetry.performance = performance_data; telemetry.fused = fused_data; telemetry.moto = moto_data;
            telemetry.state = state;
            telemetry.publishedMs = now;
            telemetry.imuSamples = samples; telemetry.imuIntervalUs = intervalUs; telemetry.imuOverruns = overruns;
            xSemaphoreGive(snapshotMutex);
        }
    }
}
void executeCommand(StorageCommand command) {
    switch (command) {
    case StorageCommand::AudioStart: startRecording(); break;
    case StorageCommand::AudioStop: stopRecording(); break;
    case StorageCommand::AudioPause: pauseRecording(); break;
    case StorageCommand::AudioResume: resumeRecording(); break;
    case StorageCommand::AudioDelete:
        stopRecording(); deleteAllRecordings(); break;
    case StorageCommand::AudioToggleAuto: audio_data.auto_record_on_ride = !audio_data.auto_record_on_ride; break;
    case StorageCommand::LogToggle:
        manualLogging = true;
        if (sd_status.logging) stopLogging(); else startLogging();
        break;
    case StorageCommand::LogList: listLogFiles(); break;
    }
}
void storageTask(void*) {
    waitReady();
    initSDCard(); // Failure leaves IMU/GPS/web running.
    uint32_t errors = 0;
    for (;;) {
        LogSample log;
        for (int i = 0; i < 8 && xQueueReceive(logQueue, &log, 0) == pdTRUE; ++i) {
            if (millis() - log.data.timestamp > 2000) { ++logDropped; continue; }
            if (!manualLogging) checkAutoLogging(log.validSpeed ? log.data.speed_kmh : 0);
            if (sd_status.logging && !logDataPoint(log.data)) ++errors;
        }
        {
            FilesystemGuard lock(0);
            if (lock) {
                StorageCommand command;
                for (int i = 0; i < 4 && xQueueReceive(commandQueue, &command, 0) == pdTRUE; ++i) executeCommand(command);
                updateAudioRecorder();
                Session session;
                if (xQueueReceive(sessionQueue, &session, 0) == pdTRUE) saveSessionData(session.gps, session.performance);
                xSemaphoreTake(snapshotMutex, portMAX_DELAY);
                storage.audio = audio_data;
                xSemaphoreGive(snapshotMutex);
            }
        }
        xSemaphoreTake(snapshotMutex, portMAX_DELAY);
        storage.sdReady = sd_status.initialized; storage.logging = sd_status.logging;
        storage.points = sd_status.data_points; storage.writeErrors = errors;
        xSemaphoreGive(snapshotMutex);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
void networkTask(void*) { waitReady(); for (;;) { networkStep(); vTaskDelay(pdMS_TO_TICKS(2)); } }
void displayTask(void*) {
    waitReady(); TickType_t wake = xTaskGetTickCount();
    for (;;) { displayStep(); periodicWait(wake, pdMS_TO_TICKS(50)); }
}
}

FilesystemGuard::FilesystemGuard(TickType_t wait) : acquired(filesystemMutex && xSemaphoreTake(filesystemMutex, wait) == pdTRUE) {}
FilesystemGuard::~FilesystemGuard() { if (acquired) xSemaphoreGive(filesystemMutex); }
TelemetrySnapshot readTelemetry() {
    xSemaphoreTake(snapshotMutex, portMAX_DELAY);
    TelemetrySnapshot result = telemetry;
    xSemaphoreGive(snapshotMutex); return result;
}
StorageSnapshot readStorageStatus() {
    xSemaphoreTake(snapshotMutex, portMAX_DELAY);
    StorageSnapshot result = storage;
    xSemaphoreGive(snapshotMutex); return result;
}
bool sendStorageCommand(StorageCommand command) {
    if (xQueueSend(commandQueue, &command, 0) == pdTRUE) return true;
    ++commandsRejected; return false;
}
bool requestMotoReset() {
    uint8_t reset = 1;
    if (xQueueSend(modelCommandQueue, &reset, 0) == pdTRUE) return true;
    ++commandsRejected; return false;
}
void queueSessionSave() {
    Session session{gps_data, performance_data};
    if (sessionQueue) xQueueOverwrite(sessionQueue, &session);
}
RtosDiagnostics readRtosDiagnostics() {
    RtosDiagnostics result;
    result.imuDropped = imuDropped.load(); result.logDropped = logDropped.load();
    result.commandsRejected = commandsRejected.load();
    result.imuQueued = uxQueueMessagesWaiting(imuQueue); result.logQueued = uxQueueMessagesWaiting(logQueue);
    for (int i = 0; i < 5; ++i) result.stackFreeBytes[i] = uxTaskGetStackHighWaterMark(tasks[i]);
    return result;
}
bool startRtosRuntime() {
    static_assert(configTICK_RATE_HZ >= 200, "200Hz IMU requires ticks <=5ms");
    snapshotMutex = xSemaphoreCreateMutex(); filesystemMutex = xSemaphoreCreateMutex();
    startup = xEventGroupCreate();
    imuQueue = xQueueCreate(ImuDepth, sizeof(ImuSample)); logQueue = xQueueCreate(LogDepth, sizeof(LogSample));
    commandQueue = xQueueCreate(8, sizeof(StorageCommand)); modelCommandQueue = xQueueCreate(4, sizeof(uint8_t));
    sessionQueue = xQueueCreate(1, sizeof(Session));
    bool ok = snapshotMutex && filesystemMutex && startup && imuQueue && logQueue && commandQueue && modelCommandQueue && sessionQueue;
    if (ok) {
        ok = xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr, 4, &tasks[0], 1) == pdPASS &&
             xTaskCreatePinnedToCore(modelTask, "model", 8192, nullptr, 3, &tasks[1], 1) == pdPASS &&
             xTaskCreatePinnedToCore(storageTask, "storage", 8192, nullptr, 2, &tasks[2], 0) == pdPASS &&
             xTaskCreatePinnedToCore(networkTask, "web", 12288, nullptr, 1, &tasks[3], 0) == pdPASS &&
             xTaskCreatePinnedToCore(displayTask, "display", 6144, nullptr, 1, &tasks[4], 1) == pdPASS;
    }
    if (!ok) {
        // Workers wait on the startup event; none can touch partially created resources.
        for (auto task : tasks) if (task) vTaskDelete(task);
        for (auto queue : {imuQueue, logQueue, commandQueue, modelCommandQueue, sessionQueue}) if (queue) vQueueDelete(queue);
        if (snapshotMutex) vSemaphoreDelete(snapshotMutex);
        if (filesystemMutex) vSemaphoreDelete(filesystemMutex);
        if (startup) vEventGroupDelete(startup);
        return false;
    }
    storage.audio = audio_data;
    xEventGroupSetBits(startup, Ready);
    return true;
}
