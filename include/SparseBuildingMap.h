#pragma once

#include <vector>

#include "Types.h"
#include "IBuildingMap.h"

/*
    SparseBuildingMap represents the drone's discovered map.

    It starts as Unknown because the drone does not know the world yet.

    This class implements IBuildingMap so the Drone can use it through
    the interface, without knowing the concrete map class.
*/
class SparseBuildingMap : public IBuildingMap {
public:
    SparseBuildingMap(int sizeX, int sizeY, int sizeZ);

    CellState getCell(const Position& pos) const override;
    void setCell(const Position& pos, CellState state) override;

    bool isInside(const Position& pos) const;

    int getSizeX() const;
    int getSizeY() const;
    int getSizeZ() const;

private:
    int sizeX;
    int sizeY;
    int sizeZ;

    std::vector<CellState> cells;

    int index(const Position& pos) const;
};