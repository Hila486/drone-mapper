#ifndef IBUILDINGMAP_H
#define IBUILDINGMAP_H

#include "Types.h"

class IBuildingMap {
public:
    virtual int getCell(const Position& pos) const = 0;
    virtual void setCell(const Position& pos, int value) = 0;
    virtual ~IBuildingMap() = default;
    
};

#endif
