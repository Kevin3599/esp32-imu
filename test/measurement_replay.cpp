#include "../src/measurement.h"
#include <cassert>
#include <iostream>

int main() {
    measurement::Timer t;
    // Linear ramp 20 km/h/s, with arrival intervals of 100ms.
    for (uint32_t ms=0; ms<=11000; ms+=100) t.observe(ms*0.02, ms, true);
    assert(std::abs(t.to60-2.95)<0.002);
    assert(std::abs(t.to100-4.95)<0.002);
    assert(std::abs(t.from100to200-5.0)<0.002);
    for (uint32_t ms=11100; ms<=22000; ms+=100) t.observe(220-(ms-11000)*0.02, ms, true);
    assert(std::abs(t.brake-4.95)<0.002);
    // An invalid observation must break continuity across a threshold.
    t = {};
    t.observe(0,0,true); t.observe(40,100,true); t.observe(70,200,false);
    t.observe(100,300,true); assert(t.to100==0);
    // Outage, duplicate timestamp and boot while already moving.
    t = {};
    t.observe(0,0,true); t.observe(50,100,true); t.observe(110,2000,true);
    assert(t.to100==0);
    t = {}; t.observe(30,10,true); t.observe(120,110,true); assert(t.to100==0);
    t = {}; t.observe(0,10,true); t.observe(100,10,true); assert(!t.accelerating);
    // Rolling 100-200, independent of launch; exactly 100 starts braking on next decrease.
    t = {}; t.observe(90,0,true);
    for (uint32_t ms=100; ms<=6000; ms+=100) t.observe(90+ms*0.02,ms,true);
    assert(std::abs(t.from100to200-5)<0.002 && t.to100==0);
    t = {}; t.observe(100,0,true); t.observe(50,500,true); t.observe(0,1000,true);
    assert(std::abs(t.brake-0.99)<0.002);
    // Jittered sampling of a linear trace preserves threshold times.
    t = {}; t.observe(0,0,true);
    for (uint32_t ms=71; ms<6000; ms+=137) t.observe(ms*0.02,ms,true);
    assert(std::abs(t.to100-4.95)<0.003);
    // Millis wrap is supported by unsigned subtraction.
    t = {}; t.observe(0,UINT32_MAX-99,true); t.observe(2,100,true);
    assert(t.accelerating && t.start==0);
    assert(std::abs(measurement::radToDeg*3.141592653589793-180)<1e-9);
    assert(measurement::forward(2)==-2);
    // Rotate the upright gravity vector by 0.01 rad around each sensor axis.
    const double angle = 0.01;
    const double lean = -std::atan2(-std::sin(angle), std::cos(angle))*measurement::radToDeg;
    const double pitch = std::atan2(-std::sin(angle), std::cos(angle))*measurement::radToDeg;
    assert(std::abs(lean-measurement::leanRate(1)*angle)<1e-9);
    assert(std::abs(pitch-measurement::pitchRate(1)*angle)<1e-9);
    std::cout << "measurement replay: PASS\n";
}
