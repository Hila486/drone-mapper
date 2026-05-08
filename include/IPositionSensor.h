#ifndef IPOSITION_SENSOR_H
#define IPOSITION_SENSOR_H

#include "Types.h"

// Interface for the drone's position sensor.
//
// The Drone class will use this interface.
// In the simulator, we will give the drone a MockPositionSensor,
// but the drone should not know that it is a mock.
//
// The sensor returns the drone's current pose:
// - position: x, y, height
// - xyAngle: direction in the XY plane
class IPositionSensor {
public:
    virtual ~IPositionSensor() = default;
    virtual Pose getPose() const = 0;
};

#endif