

//-----------------------------------------
// Types.h
#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include "Units.h"

// --------------------
// Position / Pose
// --------------------

struct Position {
    Cm x;
    Cm y;
    Cm height;

    Position() : x(0), y(0), height(0) {}

    Position(Cm x_, Cm y_, Cm height_)
        : x(x_), y(y_), height(height_) {}
};

// Full drone state from position sensor:
// position + XY angle.
// Assignment convention:
// 0   = east
// 90  = south
// 180 = west
// 270 = north
struct Pose {
    Position position;
    Degree xyAngle;

    Pose() : position(), xyAngle(0) {}

    Pose(const Position& position_, Degree xyAngle_)
        : position(position_), xyAngle(xyAngle_) {}
};

// --------------------
// Map cells
// --------------------

enum class CellState : int {
    Unknown = UNKNOWN_CELL,
    Free = FREE_CELL,
    Occupied = OCCUPIED_CELL,
    OutOfBounds = OUT_OF_BOUNDS
};

// --------------------
// Commands
// --------------------

enum class RotationDirection {
    Left,
    Right
};

enum class CommandType {
    Rotate,
    Advance,
    Elevate,
    Scan,
    GetLocation,
    Finished
};

struct ScanAngle {
    Degree xyAngle;
    Degree heightAngle;

    ScanAngle() : xyAngle(0), heightAngle(0) {}

    ScanAngle(Degree xyAngle_, Degree heightAngle_)
        : xyAngle(xyAngle_), heightAngle(heightAngle_) {}
};

struct Command {
    CommandType type;

    // Used only for Rotate
    RotationDirection rotationDirection;
    Degree angle;

    // Used for Advance / Elevate
    Cm distance;

    // Used for Scan
    ScanAngle scanAngle;

    static Command rotate(RotationDirection dir, Degree angle_) {
        Command command;
        command.type = CommandType::Rotate;
        command.rotationDirection = dir;
        command.angle = angle_;
        return command;
    }

    static Command advance(Cm distance_) {
        Command command;
        command.type = CommandType::Advance;
        command.distance = distance_;
        return command;
    }

    static Command elevate(Cm distance_) {
        Command command;
        command.type = CommandType::Elevate;
        command.distance = distance_;
        return command;
    }

    static Command scan(ScanAngle angle_) {
        Command command;
        command.type = CommandType::Scan;
        command.scanAngle = angle_;
        return command;
    }

    static Command getLocation() {
        Command command;
        command.type = CommandType::GetLocation;
        return command;
    }

    static Command finished() {
        Command command;
        command.type = CommandType::Finished;
        return command;
    }
};

// --------------------
// Lidar result
// --------------------

struct ScanHit {
    ScanAngle angle;
    Cm distance;

    ScanHit() : angle(), distance(0) {}

    ScanHit(const ScanAngle& angle_, Cm distance_)
        : angle(angle_), distance(distance_) {}
};

using ScanResult = std::vector<ScanHit>;

#endif