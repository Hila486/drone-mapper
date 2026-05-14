#ifndef IBUILDINGMAP_H
#define IBUILDINGMAP_H

#include "Types.h"

class IBuildingMap {
public:
    virtual ~IBuildingMap() = default;

    virtual CellState getCell(const Position& pos) const = 0;
    virtual void setCell(const Position& pos, CellState state) = 0;
};

#endif
