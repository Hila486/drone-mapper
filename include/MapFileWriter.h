#pragma once

#include <string>

#include "SparseBuildingMap.h"

/*
    MapFileWriter

    Responsible for writing the drone's discovered map into map_output.txt.

    Important:
    This writes SparseBuildingMap, not GroundTruthMap.

    GroundTruthMap = hidden real world.
    SparseBuildingMap = what the drone discovered.
*/
class MapFileWriter {
public:
    static bool writeSparseMap(
        const std::string& filePath,
        const SparseBuildingMap& map
    );
};