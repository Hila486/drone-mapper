#include "GroundTruthMap.h"

GroundTruthMap::GroundTruthMap(int sizeX, int sizeY, int sizeZ)
    : sizeX(sizeX),
      sizeY(sizeY),
      sizeZ(sizeZ),
      cells(sizeX * sizeY * sizeZ, CellState::Empty) {
}

bool GroundTruthMap::isInside(const Position& pos) const {
    return pos.x >= 0 && pos.x < sizeX &&
           pos.y >= 0 && pos.y < sizeY &&
           pos.z >= 0 && pos.z < sizeZ;
}

int GroundTruthMap::index(const Position& pos) const {
    return pos.z * sizeX * sizeY + pos.y * sizeX + pos.x;
}

CellState GroundTruthMap::getCell(const Position& pos) const {
    if (!isInside(pos)) {
        return CellState::OutOfBounds;
    }

    return cells[index(pos)];
}

void GroundTruthMap::setCell(const Position& pos, CellState state) {
    if (!isInside(pos)) {
        return;
    }

    cells[index(pos)] = state;
}