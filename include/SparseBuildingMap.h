#pragma once

#include <vector>
#include "Types.h"

// SparseBuildingMap represents the drone's discovered map.
// It starts as Unmapped because the drone does not know the world yet.
class SparseBuildingMap {
public:
    SparseBuildingMap(int sizeX, int sizeY, int sizeZ);

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