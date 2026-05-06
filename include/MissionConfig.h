#pragma once

#include "Types.h"
//Defines the MissionConfig struct, which stores mission settings such as start position, start angle, map boundaries, and resolution.
struct MissionConfig {
    // Starting drone pose
    Position startPosition{0, 0, 0};
    int startAngleDeg = 0;

    // Mapping boundaries
    int minX = 0;
    int maxX = 0;

    int minY = 0;
    int maxY = 0;

    int minZ = 0;
    int maxZ = 0;

    // Required output/map resolution
    int resolutionCm = 0;
};