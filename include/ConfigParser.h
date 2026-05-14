#pragma once

#include <string>

#include "DroneConfig.h"
#include "MissionConfig.h"
#include "GroundTruthMap.h"

/*
    ConfigParser

    This class reads the simulator input files and converts them into C++ objects.

    We use static functions because ConfigParser does not need to remember state.
    It just receives a file path, reads the file, and returns the parsed object.

    Files:
    - drone_config.txt   -> DroneConfig
    - mission_config.txt -> MissionConfig
    - map_input.txt      -> GroundTruthMap
*/
class ConfigParser {
public:
    static DroneConfig parseDroneConfig(const std::string& filePath);

    static MissionConfig parseMissionConfig(const std::string& filePath);

    static GroundTruthMap parseMapInput(const std::string& filePath);
};