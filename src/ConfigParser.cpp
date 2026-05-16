#include "ConfigParser.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


namespace {
    std::vector<std::string> inputErrors;

    void addInputError(const std::string& message) {
        inputErrors.push_back(message);
    }

    bool parseIntToken(
        const std::string& token,
        int& value,
        const std::string& fieldName,
        const std::string& filePath
    ) {
        try {
            size_t usedChars = 0;
            int parsedValue = std::stoi(token, &usedChars);

            if (usedChars != token.size()) {
                addInputError(
                    filePath + ": bad value for " + fieldName +
                    " = '" + token + "'. Ignoring this token."
                );
                return false;
            }

            value = parsedValue;
            return true;
        } catch (...) {
            addInputError(
                filePath + ": bad value for " + fieldName +
                " = '" + token + "'. Ignoring this token."
            );
            return false;
        }
    }

    int getValueOrDefault(
        const std::vector<int>& values,
        size_t index,
        int defaultValue,
        const std::string& fieldName,
        const std::string& filePath
    ) {
        if (index < values.size()) {
            return values[index];
        }

        addInputError(
            filePath + ": missing " + fieldName +
            ". Using default value " + std::to_string(defaultValue) + "."
        );

        return defaultValue;
    }

    void validatePositive(
        int& value,
        int defaultValue,
        const std::string& fieldName,
        const std::string& filePath
    ) {
        if (value <= 0) {
            addInputError(
                filePath + ": " + fieldName +
                " must be positive. Got " + std::to_string(value) +
                ". Using default value " + std::to_string(defaultValue) + "."
            );

            value = defaultValue;
        }
    }

    void validateNonNegative(
        int& value,
        int defaultValue,
        const std::string& fieldName,
        const std::string& filePath
    ) {
        if (value < 0) {
            addInputError(
                filePath + ": " + fieldName +
                " cannot be negative. Got " + std::to_string(value) +
                ". Using default value " + std::to_string(defaultValue) + "."
            );

            value = defaultValue;
        }
    }
}


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
        Expected drone_config.txt format:

        maxAdvanceCm
        maxElevateCm
        maxRotateDeg
        lidarMinRangeCm
        lidarMaxRangeCm
        lidarBeamSpacingCm
        lidarFovCircleCount
        minPassWidthCm
        minPassHeightCm
    */

    DroneConfig config;

    std::ifstream file(filePath);

    if (!file.is_open()) {
        addInputError(
            filePath +
            ": could not open file. Using default drone configuration."
        );

        config.maxAdvanceCm = 100;
        config.maxElevateCm = 50;
        config.maxRotateDeg = 90;

        config.lidarMinRangeCm = 0;
        config.lidarMaxRangeCm = 500;
        config.lidarBeamSpacingCm = 1;
        config.lidarFovCircleCount = 1;

        config.lidarRangeCm = config.lidarMaxRangeCm;
        config.lidarResolutionCm = config.lidarBeamSpacingCm;

        config.minPassWidthCm = 1;
        config.minPassHeightCm = 1;

        return config;
    }

    auto readField = [&](const std::string& fieldName, int defaultValue) {
        std::string token;

        if (!(file >> token)) {
            addInputError(
                filePath + ": missing " + fieldName +
                ". Using default value " + std::to_string(defaultValue) + "."
            );
            return defaultValue;
        }

        int value = defaultValue;

        if (!parseIntToken(token, value, fieldName, filePath)) {
            addInputError(
                filePath + ": using default value " +
                std::to_string(defaultValue) +
                " for " + fieldName + "."
            );
            return defaultValue;
        }

        return value;
    };

    config.maxAdvanceCm =
        readField("maxAdvanceCm", 100);

    config.maxElevateCm =
        readField("maxElevateCm", 50);

    config.maxRotateDeg =
        readField("maxRotateDeg", 90);

    config.lidarMinRangeCm =
        readField("lidarMinRangeCm", 0);

    config.lidarMaxRangeCm =
        readField("lidarMaxRangeCm", 500);

    config.lidarBeamSpacingCm =
        readField("lidarBeamSpacingCm", 1);

    config.lidarFovCircleCount =
        readField("lidarFovCircleCount", 1);

    config.minPassWidthCm =
        readField("minPassWidthCm", 1);

    config.minPassHeightCm =
        readField("minPassHeightCm", 1);

    validatePositive(config.maxAdvanceCm, 100, "maxAdvanceCm", filePath);
    validatePositive(config.maxElevateCm, 50, "maxElevateCm", filePath);
    validatePositive(config.maxRotateDeg, 90, "maxRotateDeg", filePath);

    validateNonNegative(config.lidarMinRangeCm, 0, "lidarMinRangeCm", filePath);
    validatePositive(config.lidarMaxRangeCm, 500, "lidarMaxRangeCm", filePath);
    validatePositive(config.lidarBeamSpacingCm, 1, "lidarBeamSpacingCm", filePath);
    validatePositive(config.lidarFovCircleCount, 1, "lidarFovCircleCount", filePath);

    validatePositive(config.minPassWidthCm, 1, "minPassWidthCm", filePath);
    validatePositive(config.minPassHeightCm, 1, "minPassHeightCm", filePath);

    if (config.lidarMinRangeCm > config.lidarMaxRangeCm) {
        addInputError(
            filePath +
            ": lidarMinRangeCm is larger than lidarMaxRangeCm. "
            "Using lidarMinRangeCm = 0."
        );

        config.lidarMinRangeCm = 0;
    }

    /*
        Keep old fields synchronized for older code.
    */
    config.lidarRangeCm = config.lidarMaxRangeCm;
    config.lidarResolutionCm = config.lidarBeamSpacingCm;

    return config;
}

MissionConfig ConfigParser::parseMissionConfig(const std::string& filePath) {
    /*
        Expected mission_config.txt format:

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
    */

    MissionConfig config;

    std::ifstream file(filePath);

    if (!file.is_open()) {
        addInputError(
            filePath +
            ": could not open file. Using default mission configuration."
        );

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

    auto readField = [&](const std::string& fieldName, int defaultValue) {
        std::string token;

        if (!(file >> token)) {
            addInputError(
                filePath + ": missing " + fieldName +
                ". Using default value " + std::to_string(defaultValue) + "."
            );
            return defaultValue;
        }

        int value = defaultValue;

        if (!parseIntToken(token, value, fieldName, filePath)) {
            addInputError(
                filePath + ": using default value " +
                std::to_string(defaultValue) +
                " for " + fieldName + "."
            );
            return defaultValue;
        }

        return value;
    };

    config.startPosition.x =
        readField("startX", 0);

    config.startPosition.y =
        readField("startY", 0);

    config.startPosition.height =
        readField("startHeight", 0);

    config.startAngleDeg =
        readField("startAngleDeg", 0);

    config.minX =
        readField("minX", 0);

    config.maxX =
        readField("maxX", 10);

    config.minY =
        readField("minY", 0);

    config.maxY =
        readField("maxY", 10);

    config.minZ =
        readField("minZ", 0);

    config.maxZ =
        readField("maxZ", 3);

    config.resolutionCm =
        readField("resolutionCm", 100);

    /*
        Normalize the start angle.

        We keep it in the range [0, 359].
    */
    if (config.startAngleDeg < 0 || config.startAngleDeg >= 360) {
        addInputError(
            filePath + ": startAngleDeg should be in range [0, 359]. Got " +
            std::to_string(config.startAngleDeg) + ". Normalizing it."
        );

        config.startAngleDeg = config.startAngleDeg % 360;

        if (config.startAngleDeg < 0) {
            config.startAngleDeg += 360;
        }
    }

    /*
        Validate boundaries.

        If min is larger than max, we swap them.
        This is recoverable and keeps the mission usable.
    */
    if (config.minX > config.maxX) {
        addInputError(
            filePath + ": minX is larger than maxX. Swapping them."
        );

        int temp = config.minX;
        config.minX = config.maxX;
        config.maxX = temp;
    }

    if (config.minY > config.maxY) {
        addInputError(
            filePath + ": minY is larger than maxY. Swapping them."
        );

        int temp = config.minY;
        config.minY = config.maxY;
        config.maxY = temp;
    }

    if (config.minZ > config.maxZ) {
        addInputError(
            filePath + ": minZ is larger than maxZ. Swapping them."
        );

        int temp = config.minZ;
        config.minZ = config.maxZ;
        config.maxZ = temp;
    }

    /*
        Validate resolution.
    */
    validatePositive(config.resolutionCm, 100, "resolutionCm", filePath);

    /*
        Validate start position.

        If the start position is outside mission boundaries,
        move it to the minimum boundary corner.
    */
    bool startOutside =
        config.startPosition.x < config.minX ||
        config.startPosition.x > config.maxX ||
        config.startPosition.y < config.minY ||
        config.startPosition.y > config.maxY ||
        config.startPosition.height < config.minZ ||
        config.startPosition.height > config.maxZ;

    if (startOutside) {
        addInputError(
            filePath +
            ": start position is outside mission boundaries. "
            "Using minimum boundary position as start position."
        );

        config.startPosition = Position{
            config.minX,
            config.minY,
            config.minZ
        };
    }

    return config;
}


GroundTruthMap ConfigParser::parseMapInput(const std::string& filePath) {
    /*
        Expected map_input.txt format:

        sizeX sizeY sizeHeight

        Then sizeX * sizeY * sizeHeight cell values.

        Cell values:
        0 = free
        1 = occupied

        Example:
        10 10 3
        then 300 values.
    */

    std::ifstream file(filePath);

    if (!file.is_open()) {
        addInputError(
            filePath +
            ": could not open file. Using default empty map 10x10x3."
        );

        return GroundTruthMap(10, 10, 3);
    }

    auto readField = [&](const std::string& fieldName, int defaultValue) {
        std::string token;

        if (!(file >> token)) {
            addInputError(
                filePath + ": missing " + fieldName +
                ". Using default value " + std::to_string(defaultValue) + "."
            );
            return defaultValue;
        }

        int value = defaultValue;

        if (!parseIntToken(token, value, fieldName, filePath)) {
            addInputError(
                filePath + ": using default value " +
                std::to_string(defaultValue) +
                " for " + fieldName + "."
            );
            return defaultValue;
        }

        return value;
    };

    int sizeX = readField("sizeX", 10);
    int sizeY = readField("sizeY", 10);
    int sizeHeight = readField("sizeHeight", 3);

    validatePositive(sizeX, 10, "sizeX", filePath);
    validatePositive(sizeY, 10, "sizeY", filePath);
    validatePositive(sizeHeight, 3, "sizeHeight", filePath);

    GroundTruthMap map(sizeX, sizeY, sizeHeight);

    int expectedCells = sizeX * sizeY * sizeHeight;

    for (int height = 0; height < sizeHeight; ++height) {
        for (int y = 0; y < sizeY; ++y) {
            for (int x = 0; x < sizeX; ++x) {
                std::string token;

                if (!(file >> token)) {
                    addInputError(
                        filePath + ": missing cell value at position (" +
                        std::to_string(x) + ", " +
                        std::to_string(y) + ", " +
                        std::to_string(height) +
                        "). Using 0 = free."
                    );

                    map.setCell(Position{x, y, height}, CellState::Free);
                    continue;
                }

                int value = 0;

                if (!parseIntToken(token, value, "map cell", filePath)) {
                    addInputError(
                        filePath + ": using 0 = free for bad cell at position (" +
                        std::to_string(x) + ", " +
                        std::to_string(y) + ", " +
                        std::to_string(height) + ")."
                    );

                    map.setCell(Position{x, y, height}, CellState::Free);
                    continue;
                }

                if (value == 0) {
                    map.setCell(Position{x, y, height}, CellState::Free);
                } else if (value == 1) {
                    map.setCell(Position{x, y, height}, CellState::Occupied);
                } else {
                    addInputError(
                        filePath + ": invalid cell value " +
                        std::to_string(value) +
                        " at position (" +
                        std::to_string(x) + ", " +
                        std::to_string(y) + ", " +
                        std::to_string(height) +
                        "). Expected 0 or 1. Using 0 = free."
                    );

                    map.setCell(Position{x, y, height}, CellState::Free);
                }
            }
        }
    }

    /*
        Check for extra values after the expected number of cells.

        Extra values are recoverable: we ignore them but report them.
    */
    std::string extraToken;
    int extraCount = 0;

    while (file >> extraToken) {
        ++extraCount;
    }

    if (extraCount > 0) {
        addInputError(
            filePath + ": found " +
            std::to_string(extraCount) +
            " extra cell value(s) after expected " +
            std::to_string(expectedCells) +
            " cells. Ignoring extras."
        );
    }

    return map;
}

void ConfigParser::clearInputErrors() {
    inputErrors.clear();
}

void ConfigParser::writeInputErrors(const std::string& filePath) {
    /*
        Assignment requirement:
        Create input_errors.txt only if there are recoverable input errors.

        So if there are no errors, we remove an old file if it exists.
    */
    if (inputErrors.empty()) {
        std::remove(filePath.c_str());
        return;
    }

    std::ofstream file(filePath);

    if (!file.is_open()) {
        std::cout << "Could not create input errors file: "
                  << filePath
                  << std::endl;
        return;
    }

    for (const std::string& error : inputErrors) {
        file << error << "\n";
    }
}