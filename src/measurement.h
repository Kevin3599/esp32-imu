#pragma once
#include <stdint.h>
#include <cmath>

namespace measurement {
constexpr double radToDeg = 57.29577951308232;
// Existing vertical mount: sensor X down, Y lateral, -Z forward.
inline double forward(double z) { return -z; }
inline double lateral(double y) { return y; }
inline double vertical(double x) { return x; }
// Signs match lean=-atan2(ay,ax), pitch=atan2(-az,ax) near upright.
inline double leanRate(double gz) { return gz * radToDeg; }
inline double pitchRate(double gy) { return -gy * radToDeg; }

// Arrival-time estimates, not certified GNSS measurement-time results.
// Explicit intervals: 1-60, 1-100, 100-200 and 100-1 km/h.
struct Timer {
    bool previous = false, armed = false, accelerating = false, braking = false;
    bool reached100 = false, interval100 = false;
    double speed = 0, to60 = 0, to100 = 0, from100to200 = 0, brake = 0;
    uint32_t time = 0, start = 0, at100 = 0, brakeStart = 0;
    void invalidate() {
        previous = armed = accelerating = braking = reached100 = interval100 = false;
    }
    uint32_t crossing(double target, double next, uint32_t now) const {
        return time + static_cast<uint32_t>((target-speed)/(next-speed) * uint32_t(now-time));
    }
    void observe(double next, uint32_t now, bool valid) {
        if (!valid || !std::isfinite(next) || next < 0 || next >= 400) { invalidate(); return; }
        if (previous && uint32_t(now-time) == 0) return;
        if (previous && uint32_t(now-time) > 1500) invalidate();
        if (previous) {
            if (armed && speed < 1 && next >= 1) {
                start = crossing(1, next, now);
                accelerating = true; armed = false; reached100 = false;
                to60 = to100 = from100to200 = 0;
            }
            if (accelerating) {
                if (speed < 60 && next >= 60 && to60 == 0) to60 = uint32_t(crossing(60,next,now)-start)/1000.0;
                if (speed < 100 && next >= 100 && !reached100) {
                    at100 = crossing(100,next,now); reached100 = true;
                    to100 = uint32_t(at100-start)/1000.0;
                }
                if (next < 1 || uint32_t(now-start) > 60000) accelerating = false;
            }
            // A rolling 100-200 run does not require a launch from rest.
            if (!interval100 && speed < 100 && next >= 100) {
                at100 = crossing(100,next,now); interval100 = true; from100to200 = 0;
            }
            if (interval100) {
                if (speed < 200 && next >= 200) {
                    from100to200 = uint32_t(crossing(200,next,now)-at100)/1000.0;
                    interval100 = accelerating = false;
                }
                if (next < 100 || uint32_t(now-at100) > 60000) interval100 = false;
            }
            if (!braking && speed >= 100 && next < 100) {
                brakeStart = crossing(100,next,now); braking = true; brake = 0;
            }
            if (braking) {
                if (speed > 1 && next <= 1) {
                    brake = uint32_t(crossing(1,next,now)-brakeStart)/1000.0; braking = false;
                }
                if (next > 100 || uint32_t(now-brakeStart) > 60000) braking = false;
            }
        }
        if (next < 1) armed = true;
        speed = next; time = now; previous = true;
    }
};
}
