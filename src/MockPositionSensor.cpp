#include "MockPositionSensor.h"

// Constructor.
// Stores a reference to the simulator's DroneState.
MockPositionSensor::MockPositionSensor(const DroneState& droneState)
    : droneState(droneState) {
}

// Returns the current drone pose.
// This simulates a real position sensor.
Pose MockPositionSensor::getPose() const {
    return droneState.pose;
}