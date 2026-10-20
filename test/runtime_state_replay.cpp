#include "../src/runtime_state.h"
#include <cassert>
#include <iostream>

int main() {
    RuntimeState state;
    state.observe(100, false, false, false, 0, false);
    assert(state.gps == GpsMode::NoSignal && state.health == HealthMode::Booting);
    state.observe(200, true, false, false, 0, true);
    assert(state.gps == GpsMode::Acquiring);
    state.observe(300, true, true, true, 0, true);
    assert(state.gps == GpsMode::Fixed && state.health == HealthMode::Ready);
    state.observe(310, false, true, true, 6, false);
    assert(state.ride == RideMode::Riding && state.health == HealthMode::Ready);
    state.observe(420, false, true, false, 0, false);
    assert(state.ride == RideMode::Cooling);
    state.observe(500, true, true, true, 6, true);
    assert(state.ride == RideMode::Riding);
    state.observe(600, true, true, true, 0, true);
    assert(state.ride == RideMode::Cooling);
    state.observe(30600, false, false, false, 0, false);
    assert(state.ride == RideMode::Idle && state.gps == GpsMode::Stale && state.health == HealthMode::Degraded);
    std::cout << "runtime state replay: PASS\n";
}
