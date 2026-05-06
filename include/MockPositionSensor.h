#pragma once

#include "IPositionSensor.h"
#include "DroneState.h"

// MockPositionSensor is a fake position sensor for the simulator.
// It reads the current pose from DroneState and returns it to the drone.
class MockPositionSensor : public IPositionSensor {
public:
    explicit MockPositionSensor(const DroneState& droneState);

    Pose getPose() const override;

private:
    //The sensor does not own the drone state.
    //It only keeps a reference to the existing state.
    const DroneState& droneState;
};