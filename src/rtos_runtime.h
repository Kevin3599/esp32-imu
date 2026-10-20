#pragma once
#include "mpu6050.h"
#include "gps_module.h"
#include "performance_analyzer.h"
#include "audio_recorder.h"
#include "runtime_state.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Algorithm globals are owned by ModelTask. Consumers only read this copy.
struct TelemetrySnapshot {
    MPU6050_DATA imu;
    GPS_DATA gps;
    PERFORMANCE_DATA performance;
    FUSED_PERFORMANCE_DATA fused;
    MOTORCYCLE_DATA moto;
    RuntimeState state;
    uint32_t publishedMs = 0;
    uint32_t imuSamples = 0, imuIntervalUs = 0, imuOverruns = 0;
};
struct StorageSnapshot {
    AUDIO_RECORDER_DATA audio;
    bool sdReady = false, logging = false;
    uint32_t points = 0, writeErrors = 0;
};
struct RtosDiagnostics {
    uint32_t imuDropped = 0, logDropped = 0, commandsRejected = 0;
    uint32_t imuQueued = 0, logQueued = 0;
    uint32_t stackFreeBytes[5] = {};
};
enum class StorageCommand : uint8_t {
    AudioStart, AudioStop, AudioPause, AudioResume, AudioDelete, AudioToggleAuto,
    LogToggle, LogList
};
bool startRtosRuntime();
TelemetrySnapshot readTelemetry();
StorageSnapshot readStorageStatus();
RtosDiagnostics readRtosDiagnostics();
bool sendStorageCommand(StorageCommand command);
bool requestMotoReset();
void queueSessionSave(); // ModelTask only, nonblocking latest-value mailbox.

// Filesystem lock never protects sensor state. Never acquire it in IMU/Model tasks.
class FilesystemGuard {
public:
    explicit FilesystemGuard(TickType_t wait = pdMS_TO_TICKS(20));
    ~FilesystemGuard();
    explicit operator bool() const { return acquired; }
    FilesystemGuard(const FilesystemGuard&) = delete;
    FilesystemGuard& operator=(const FilesystemGuard&) = delete;
private:
    bool acquired;
};

// UI entry points implemented by main.cpp; called by a single owning task each.
void networkStep();
void displayStep();
