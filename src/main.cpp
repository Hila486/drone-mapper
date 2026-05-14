#include <iostream>
#include <string>

#include "Types.h"
#include "DroneConfig.h"
#include "MissionConfig.h"
#include "GroundTruthMap.h"
#include "SparseBuildingMap.h"
#include "DroneState.h"
#include "MockPositionSensor.h"
#include "ConfigParser.h"
#include "MapFileWriter.h"
#include "ScoreCalculator.h"

// Helper function for creating paths like:
// basePath + "/" + fileName
static std::string makePath(const std::string& basePath, const std::string& fileName) {
    if (basePath.empty() || basePath == ".") {
        return "./" + fileName;
    }

    if (basePath.back() == '/') {
        return basePath + fileName;
    }

    return basePath + "/" + fileName;
}

int main(int argc, char* argv[]) {
    /*
        The assignment says the program can receive an optional input/output path.

        Example:
        ./my_exe ..

        If no path is given, we use the current working directory.
    */
    std::string inputOutputPath = ".";

    if (argc > 1) {
        inputOutputPath = argv[1];
    }

    std::string droneConfigPath = makePath(inputOutputPath, "drone_config.txt");
    std::string missionConfigPath = makePath(inputOutputPath, "mission_config.txt");
    std::string mapInputPath = makePath(inputOutputPath, "map_input.txt");

    // Read the input files.
    DroneConfig droneConfig =
        ConfigParser::parseDroneConfig(droneConfigPath);

    MissionConfig missionConfig =
        ConfigParser::parseMissionConfig(missionConfigPath);

    GroundTruthMap worldMap =
        ConfigParser::parseMapInput(mapInputPath);

    std::cout << "Drone max advance: "
              << droneConfig.maxAdvanceCm
              << " cm"
              << std::endl;

    std::cout << "Mission start position: "
              << missionConfig.startPosition.x << ", "
              << missionConfig.startPosition.y << ", "
              << missionConfig.startPosition.height
              << std::endl;

    // Small test: check the real world map.
    // This map is hidden from the drone. Only simulator-side code and mocks use it.
    Position wallPosition{2, 3, 0};
    CellState realState = worldMap.getCell(wallPosition);

    if (realState == CellState::Occupied) {
        std::cout << "GroundTruthMap: cell at "
                  << wallPosition.x << ", "
                  << wallPosition.y << ", "
                  << wallPosition.height
                  << " is occupied"
                  << std::endl;
    } else {
        std::cout << "GroundTruthMap: cell at "
                  << wallPosition.x << ", "
                  << wallPosition.y << ", "
                  << wallPosition.height
                  << " is not occupied"
                  << std::endl;
    }

    Position outsidePosition{100, 100, 100};
    CellState outsideState = worldMap.getCell(outsidePosition);

    if (outsideState == CellState::OutOfBounds) {
        std::cout << "GroundTruthMap: outside position is out of bounds"
                  << std::endl;
    }

    /*
        Create the drone's discovered map.

        This is NOT the real map.
        It starts unknown, because at the beginning the drone has not mapped anything.

        Important:
        We create it with the same dimensions as worldMap.
    */
    SparseBuildingMap droneMap(
        worldMap.getSizeX(),
        worldMap.getSizeY(),
        worldMap.getSizeZ()
    );

    Position testPosition{2, 3, 0};
    CellState droneInitialState = droneMap.getCell(testPosition);

    if (droneInitialState == CellState::Unknown) {
        std::cout << "SparseBuildingMap: drone does not know this cell yet"
                  << std::endl;
    }

    // Simulate the drone discovering that the cell is occupied.
    droneMap.setCell(testPosition, CellState::Occupied);

    CellState droneUpdatedState = droneMap.getCell(testPosition);

    if (droneUpdatedState == CellState::Occupied) {
        std::cout << "SparseBuildingMap: drone discovered this cell is occupied"
                  << std::endl;
    }

    /*
        Create the real current state of the drone.

        DroneState belongs to the simulator side.
        The drone should not directly change it.
        Later, MockMovementDriver will update this state.
    */
    DroneState droneState;
    droneState.pose.position = missionConfig.startPosition;
    droneState.pose.xyAngle = missionConfig.startAngleDeg;

    // Create a mock position sensor.
    // It reads from DroneState but does not change it.
    MockPositionSensor positionSensor(droneState);

    Pose sensedPose = positionSensor.getPose();

    std::cout << "Position sensor says drone is at: "
              << sensedPose.position.x << ", "
              << sensedPose.position.y << ", "
              << sensedPose.position.height
              << " angle "
              << sensedPose.xyAngle
              << " degrees"
              << std::endl;

    // Write the drone's discovered map into map_output.txt.
    std::string mapOutputPath = makePath(inputOutputPath, "map_output.txt");

    bool wroteMap = MapFileWriter::writeSparseMap(mapOutputPath, droneMap);

    if (wroteMap) {
        std::cout << "Wrote output map to: "
                  << mapOutputPath
                  << std::endl;
    }

    // Compare the drone's map with the real hidden world map.
    double score = ScoreCalculator::calculateScore(worldMap, droneMap);

    std::cout << "Mapping score: "
              << score
              << "/100"
              << std::endl;

    return 0;
}