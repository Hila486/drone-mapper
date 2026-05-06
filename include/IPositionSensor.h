#pragma once

#include "Types.h"

// IPositionSensor is an interface used by the drone to ask for its current pose.
// The drone sees this interface, not the simulator's internal DroneState directly.
class IPositionSensor {
public:
    virtual ~IPositionSensor() = default;

    virtual Pose getPose() const = 0;
};