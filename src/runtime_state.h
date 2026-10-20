#pragma once
#include <stdint.h>

enum class GpsMode : uint8_t { NoSignal, Acquiring, Fixed, Stale };
enum class RideMode : uint8_t { Idle, Riding, Cooling };
enum class HealthMode : uint8_t { Booting, Ready, Degraded };

struct RuntimeState {
    GpsMode gps = GpsMode::NoSignal;
    RideMode ride = RideMode::Idle;
    HealthMode health = HealthMode::Booting;
    uint32_t coolingSince = 0;
    uint32_t lastGpsByteMs = 0;
    uint32_t lastImuMs = 0;

    void observe(uint32_t now, bool gpsBytes, bool fix, bool validSpeed,
                 double speedKmh, bool imuSample) {
        if (gpsBytes) lastGpsByteMs = now;
        if (imuSample) lastImuMs = now;
        if (!gpsBytes && lastGpsByteMs == 0) gps = GpsMode::NoSignal;
        else if (uint32_t(now - lastGpsByteMs) > 1500) gps = GpsMode::Stale;
        else gps = fix && validSpeed ? GpsMode::Fixed : GpsMode::Acquiring;
        const bool moving = validSpeed && speedKmh >= 5;
        const bool stopped = !validSpeed || speedKmh < 2;
        if (ride == RideMode::Idle && moving) ride = RideMode::Riding;
        else if (ride == RideMode::Riding && stopped) {
            ride = RideMode::Cooling; coolingSince = now;
        } else if (ride == RideMode::Cooling) {
            if (moving) ride = RideMode::Riding;
            else if (uint32_t(now - coolingSince) >= 30000) ride = RideMode::Idle;
        }
        if (gps == GpsMode::Fixed && lastImuMs != 0 && uint32_t(now - lastImuMs) <= 100)
            health = HealthMode::Ready;
        else health = now < 10000 ? HealthMode::Booting : HealthMode::Degraded;
    }
};
inline const char* name(GpsMode state) {
    switch (state) {
    case GpsMode::NoSignal: return "no_signal";
    case GpsMode::Acquiring: return "acquiring";
    case GpsMode::Fixed: return "fixed";
    case GpsMode::Stale: return "stale";
    }
    return "unknown";
}
inline const char* name(RideMode state) {
    switch (state) {
    case RideMode::Idle: return "idle";
    case RideMode::Riding: return "riding";
    case RideMode::Cooling: return "cooling";
    }
    return "unknown";
}
inline const char* name(HealthMode state) {
    switch (state) {
    case HealthMode::Booting: return "booting";
    case HealthMode::Ready: return "ready";
    case HealthMode::Degraded: return "degraded";
    }
    return "unknown";
}
