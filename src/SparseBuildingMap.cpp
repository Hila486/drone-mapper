#include "SparseBuildingMap.h"

/*
    Constructor.

    Creates a 3D map with sizeX * sizeY * sizeZ cells.
    The drone's map starts as Unknown, because the drone does not know
    the world at the beginning.
*/
SparseBuildingMap::SparseBuildingMap(int sizeX, int sizeY, int sizeZ)
    : sizeX(sizeX),
      sizeY(sizeY),
      sizeZ(sizeZ),
      cells(sizeX * sizeY * sizeZ, CellState::Unknown) {
}

/*
    Checks whether a given position is inside the map boundaries.
*/
bool SparseBuildingMap::isInside(const Position& pos) const {
    return pos.x >= 0 && pos.x < sizeX &&
           pos.y >= 0 && pos.y < sizeY &&
           pos.height >= 0 && pos.height < sizeZ;
}

/*
    Converts a 3D position into a 1D vector index.

    Order:
    height layer -> y row -> x column
*/
int SparseBuildingMap::index(const Position& pos) const {
    return pos.height * sizeX * sizeY + pos.y * sizeX + pos.x;
}

/*
    Returns the cell state at the given position.

    If the position is outside the map, returns OutOfBounds.
*/
CellState SparseBuildingMap::getCell(const Position& pos) const {
    if (!isInside(pos)) {
        return CellState::OutOfBounds;
    }

    return cells[index(pos)];
}

/*
    Updates the cell state at the given position.

    If the position is outside the map, the function does nothing.
*/
void SparseBuildingMap::setCell(const Position& pos, CellState state) {
    if (!isInside(pos)) {
        return;
    }

    cells[index(pos)] = state;
}

/*
    Getters for map dimensions.
*/
int SparseBuildingMap::getSizeX() const {
    return sizeX;
}

int SparseBuildingMap::getSizeY() const {
    return sizeY;
}

int SparseBuildingMap::getSizeZ() const {
    return sizeZ;
}