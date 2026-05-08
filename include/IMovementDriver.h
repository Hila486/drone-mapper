#ifndef IMOVEMENT_DRIVER_H
#define IMOVEMENT_DRIVER_H

#include "Types.h"

// Interface for the drone movement driver.
//
// The movement driver is responsible for actually moving the drone
// inside the simulator.
//
// In our project, the real implementation will be MockMovementDriver.
//
// Movement commands:
// - rotate left/right by some angle
// - advance forward by some distance
// - elevate up/down by some distance
//
// According to the assignment, angle and distance may be positive or negative.
class IMovementDriver {
public:
    virtual ~IMovementDriver() = default;

    virtual bool rotate(RotationDirection direction, Degree angle) = 0;

    virtual bool advance(Cm distance) = 0;

    virtual bool elevate(Cm distance) = 0;
};

#endif

//virtual - This function can be replaced/implemented by a child class, and when I call it through a parent pointer, C++ should use the child version.”