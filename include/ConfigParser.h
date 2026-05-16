#pragma once

#include <string>

#include "DroneConfig.h"
#include "MissionConfig.h"
#include "GroundTruthMap.h"

/*
    ConfigParser

    Reads the simulator input files and converts them into C++ objects.

    Files:
    - drone_config.txt   -> DroneConfig
    - mission_config.txt -> MissionConfig
    - map_input.txt      -> GroundTruthMap

    It also collects recoverable input errors.
    At the end, Simulator writes them to input_errors.txt.
*/
class ConfigParser {
public:
    static void clearInputErrors();

    static void writeInputErrors(const std::string& filePath);

    static DroneConfig parseDroneConfig(const std::string& filePath);

    static MissionConfig parseMissionConfig(const std::string& filePath);

    static GroundTruthMap parseMapInput(const std::string& filePath);
};