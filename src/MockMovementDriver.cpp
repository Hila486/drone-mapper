#include "MockMovementDriver.h"

#include <cstdlib>
#include <iostream>

/*
    Constructor.

    We keep references to the simulator objects:
    - droneState: so this driver can update the drone's real position
    - droneConfig: so this driver can check movement limits
    - worldMap: so this driver can prevent collisions
*/
MockMovementDriver::MockMovementDriver(
    DroneState& droneState,
    const DroneConfig& droneConfig,
    const GroundTruthMap& worldMap
)
    : droneState(droneState),
      droneConfig(droneConfig),
      worldMap(worldMap) {
}

/*
    Converts any angle to the range [0, 359].

    Examples:
    360  -> 0
    -90  -> 270
    450  -> 90
*/
Degree MockMovementDriver::normalizeAngle(Degree angle) const {
    Degree result = angle % 360;

    if (result < 0) {
        result += 360;
    }

    return result;
}

/*
    Checks if the drone can move to a target position.

    The movement is blocked if:
    - the position is outside the map
    - the position is occupied
*/
bool MockMovementDriver::canMoveTo(const Position& position) const {
    CellState cell = worldMap.getCell(position);

    if (cell == CellState::OutOfBounds) {
        std::cout << "Movement blocked: target position is out of bounds"
                  << std::endl;
        return false;
    }

    if (cell == CellState::Occupied) {
        std::cout << "Movement blocked: target position is occupied"
                  << std::endl;
        return false;
    }

    return true;
}

/*
    Rotates the drone left or right.

    Assignment angle convention:
    0   = east
    90  = south
    180 = west
    270 = north

    In this implementation:
    - Right turn increases the angle.
    - Left turn decreases the angle.
*/
bool MockMovementDriver::rotate(RotationDirection direction, Degree angle) {
    if (std::abs(angle) > droneConfig.maxRotateDeg) {
        std::cout << "Rotate failed: requested angle "
                  << angle
                  << " is larger than max rotate "
                  << droneConfig.maxRotateDeg
                  << std::endl;
        return false;
    }

    Degree signedAngle = angle;

    if (direction == RotationDirection::Left) {
        signedAngle = -signedAngle;
    }

    droneState.pose.xyAngle =
        normalizeAngle(droneState.pose.xyAngle + signedAngle);

    return true;
}

/*
    Moves the drone forward or backward.

    Positive distance = forward.
    Negative distance = backward.

    For now, we support only the four grid directions:
    0, 90, 180, 270.
*/
bool MockMovementDriver::advance(Cm distance) {
    if (std::abs(distance) > droneConfig.maxAdvanceCm) {
        std::cout << "Advance failed: requested distance "
                  << distance
                  << " is larger than max advance "
                  << droneConfig.maxAdvanceCm
                  << std::endl;
        return false;
    }

    Position nextPosition = droneState.pose.position;
    Degree direction = normalizeAngle(droneState.pose.xyAngle);

    if (direction == 0) {
        nextPosition.x += distance;
    } else if (direction == 90) {
        nextPosition.y += distance;
    } else if (direction == 180) {
        nextPosition.x -= distance;
    } else if (direction == 270) {
        nextPosition.y -= distance;
    } else {
        std::cout << "Advance failed: unsupported angle "
                  << direction
                  << ". Current simple movement supports only 0, 90, 180, 270."
                  << std::endl;
        return false;
    }

    if (!canMoveTo(nextPosition)) {
        return false;
    }

    droneState.pose.position = nextPosition;
    return true;
}

/*
    Moves the drone up or down.

    Positive distance = up.
    Negative distance = down.
*/
bool MockMovementDriver::elevate(Cm distance) {
    if (std::abs(distance) > droneConfig.maxElevateCm) {
        std::cout << "Elevate failed: requested distance "
                  << distance
                  << " is larger than max elevate "
                  << droneConfig.maxElevateCm
                  << std::endl;
        return false;
    }

    Position nextPosition = droneState.pose.position;
    nextPosition.height += distance;

    if (!canMoveTo(nextPosition)) {
        return false;
    }

    droneState.pose.position = nextPosition;
    return true;
}