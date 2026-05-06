#pragma once

#include <vector>
#include "Types.h"

// GroundTruthMap represents the real building/world.
// It is used by the simulator and mock sensors, not directly by the drone.
class GroundTruthMap {
public:
    GroundTruthMap(int sizeX, int sizeY, int sizeZ);

    CellState getCell(const Position& pos) const;
    void setCell(const Position& pos, CellState state);

    bool isInside(const Position& pos) const;

private:
    int sizeX;
    int sizeY;
    int sizeZ;

    std::vector<CellState> cells;

    int index(const Position& pos) const;
};