#pragma once
//Contains shared basic types used across the project, such as Position, Pose, CellState, and direction enums.

enum class CellState {
    Empty = 0,
    Occupied = 1,
    Unmapped = -1,
    OutOfBounds = -2
};

struct Position {
    int x;
    int y;
    int z;
};

struct Pose {
    Position position;
    int angleDegrees;
};

enum class RotationDirection {
    Left,
    Right
    
};