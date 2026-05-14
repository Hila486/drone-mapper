#pragma once

#include "IMovementDriver.h"
#include "DroneState.h"
#include "DroneConfig.h"
#include "GroundTruthMap.h"

/*
    MockMovementDriver

    This class simulates the drone movement engine.

    The drone will ask the movement driver to:
    - rotate left/right
    - advance forward/backward
    - elevate up/down

    In a real drone, this would control motors.
    In our simulator, it only updates DroneState.

    Important:
    The movement driver is allowed to see the hidden GroundTruthMap,
    because it is part of the simulator/mocks.

    The drone algorithm itself should NOT directly use GroundTruthMap.
*/
class MockMovementDriver : public IMovementDriver {
public:
    /*
        Constructor.

        droneState:
        The real current state of the drone in the simulator.
        This object will be updated when movement succeeds.

        droneConfig:
        Contains movement limits, such as max advance/elevate/rotate.

        worldMap:
        The hidden real building map.
        Used only to prevent illegal movement into walls/out of bounds.
    */
    MockMovementDriver(
        DroneState& droneState,
        const DroneConfig& droneConfig,
        const GroundTruthMap& worldMap
    );

    /*
        Rotates the drone left or right by angle degrees.

        Returns true if the rotation succeeded.
        Returns false if the requested angle is too large.
    */
    bool rotate(RotationDirection direction, Degree angle) override;

    /*
        Moves the drone forward according to its current XY angle.

        Positive distance = forward.
        Negative distance = backward.

        Returns true if movement succeeded.
        Returns false if movement is too large or would collide.
    */
    bool advance(Cm distance) override;

    /*
        Moves the drone up or down.

        Positive distance = up.
        Negative distance = down.

        Returns true if movement succeeded.
        Returns false if movement is too large or would collide.
    */
    bool elevate(Cm distance) override;

private:
    DroneState& droneState;
    const DroneConfig& droneConfig;
    const GroundTruthMap& worldMap;

    /*
        Converts any angle into the range [0, 359].

        Examples:
        360  -> 0
        -90  -> 270
        450  -> 90
    */
    Degree normalizeAngle(Degree angle) const;

    /*
        Checks whether the drone may move into this position.

        Movement is illegal if:
        - position is outside the map
        - position is occupied by a wall/obstacle
    */
    bool canMoveTo(const Position& position) const;
};