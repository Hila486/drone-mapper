#include "ConfigParser.h"

#include <fstream>
#include <iostream>

/*
    ConfigParser.cpp

    This file is responsible for reading the input files of the simulator.

    this parser reads:
    1. drone_config.txt   -> creates DroneConfig
    2. mission_config.txt -> creates MissionConfig
    3. map_input.txt      -> creates GroundTruthMap
*/


DroneConfig ConfigParser::parseDroneConfig(const std::string& filePath) {
    /*
        Reads drone_config.txt.

        Expected simple format:

        maxAdvanceCm
        maxElevateCm
        maxRotateDeg
        lidarRangeCm

        Example:

        100
        50
        90
        500

        Meaning:
        - The drone can advance at most 100 cm per command.
        - The drone can elevate at most 50 cm per command.
        - The drone can rotate at most 90 degrees per command.
        - The lidar can scan up to 500 cm.
    */

    DroneConfig config;

    std::ifstream file(filePath);

    // If the file does not exist or cannot be opened,
    // we use reasonable default values so the program can still run.
    if (!file.is_open()) {
        std::cout << "Could not open drone config file: " << filePath
                  << ". Using default drone config."
                  << std::endl;

        config.maxAdvanceCm = 100;
        config.maxElevateCm = 50;
        config.maxRotateDeg = 90;
        config.lidarRangeCm = 500;

        return config;
    }

    // Read values in the exact order described above.
    file >> config.maxAdvanceCm;
    file >> config.maxElevateCm;
    file >> config.maxRotateDeg;
    file >> config.lidarRangeCm;

    return config;
}


MissionConfig ConfigParser::parseMissionConfig(const std::string& filePath) {
    /*
        Reads mission_config.txt.

        Expected simple format:

        startX startY startHeight
        startAngleDeg
        minX maxX
        minY maxY
        minZ maxZ
        resolutionCm

        Example:

        0 0 0
        0
        0 10
        0 10
        0 3
        100

        Meaning:
        - Start position is (0, 0, 0)
        - Start angle is 0 degrees
        - Mapping boundaries:
            X from 0 to 10
            Y from 0 to 10
            Height from 0 to 3
        - Resolution is 100 cm
    */

    MissionConfig config;

    std::ifstream file(filePath);

    // If the mission config file is missing,
    // use a small default mission so the simulator can still run.
    if (!file.is_open()) {
        std::cout << "Could not open mission config file: " << filePath
                  << ". Using default mission config."
                  << std::endl;

        config.startPosition = Position{0, 0, 0};
        config.startAngleDeg = 0;

        config.minX = 0;
        config.maxX = 10;

        config.minY = 0;
        config.maxY = 10;

        config.minZ = 0;
        config.maxZ = 3;

        config.resolutionCm = 100;

        return config;
    }

    // Read start position.
    // Notice: we use .height, not .z, because our Position struct uses height.
    file >> config.startPosition.x;
    file >> config.startPosition.y;
    file >> config.startPosition.height;

    // Read start direction angle.
    file >> config.startAngleDeg;

    // Read mapping boundaries.
    file >> config.minX;
    file >> config.maxX;

    file >> config.minY;
    file >> config.maxY;

    file >> config.minZ;
    file >> config.maxZ;

    // Read required map resolution.
    file >> config.resolutionCm;

    return config;
}


GroundTruthMap ConfigParser::parseMapInput(const std::string& filePath) {
    /*
        Reads map_input.txt.

        This file represents the hidden real world.
        The drone is NOT allowed to read this map directly.
        Only the simulator and mock sensors use it.

        Expected simple format:

        sizeX sizeY sizeHeight
        then sizeX * sizeY * sizeHeight cell values

        Cell values:
        0 = empty/free
        1 = occupied/wall/obstacle

        Example for sizeX=3, sizeY=2, sizeHeight=1:

        3 2 1
        0 0 1
        0 1 0

        This means:
        height layer 0 has 2 rows.
        Each row has 3 cells.

        Position meaning:
        x = column
        y = row
        height = height layer
    */

    std::ifstream file(filePath);

    // If the map input file is missing,
    // create a small empty default world.
    if (!file.is_open()) {
        std::cout << "Could not open map input file: " << filePath
                  << ". Using default empty map."
                  << std::endl;

        return GroundTruthMap(10, 10, 3);
    }

    int sizeX = 0;
    int sizeY = 0;
    int sizeHeight = 0;

    // First line: map dimensions.
    file >> sizeX >> sizeY >> sizeHeight;

    GroundTruthMap map(sizeX, sizeY, sizeHeight);

    /*
        Read all cells.

        Loop order:
        1. height layer
        2. y row
        3. x column

        So the file is interpreted layer by layer.
    */
    for (int height = 0; height < sizeHeight; ++height) {
        for (int y = 0; y < sizeY; ++y) {
            for (int x = 0; x < sizeX; ++x) {
                int value = 0;

                file >> value;

                Position pos{x, y, height};

                if (value == 1) {
                    map.setCell(pos, CellState::Occupied);
                } else {
                    map.setCell(pos, CellState::Free);
                }
            }
        }
    }

    return map;
}