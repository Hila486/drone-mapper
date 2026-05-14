#include "simulator.h"

#include <iostream>

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
#include "MockMovementDriver.h"

/*
    Constructor.

    We only save the folder path here.
    The actual loading and simulation happen inside run().
*/
Simulator::Simulator(const std::string& inputOutputPath)
    : inputOutputPath(inputOutputPath) {
}

/*
    Helper function for building file paths.

    The assignment says the program can receive an optional folder path.
    If no path is given, we use the current folder.

    This function makes the rest of the code cleaner, so instead of writing:
        inputOutputPath + "/" + "map_input.txt"

    every time, we simply write:
        makePath("map_input.txt")
*/
std::string Simulator::makePath(const std::string& fileName) const {
    if (inputOutputPath.empty() || inputOutputPath == ".") {
        return "./" + fileName;
    }

    if (inputOutputPath.back() == '/') {
        return inputOutputPath + fileName;
    }

    return inputOutputPath + "/" + fileName;
}

/*
    Runs the full simulator flow.

    Current version:
    This is still not the final autonomous mapping algorithm.

    For now, it:
    1. Loads the input files.
    2. Creates the real hidden map.
    3. Creates the drone's discovered map.
    4. Creates a mock position sensor.
    5. Marks the starting position as free.
    6. Writes map_output.txt.
    7. Calculates the mapping score.

    Later, we will replace the temporary marking step with a real loop:
        while mapping is not complete:
            ask drone for command
            execute command using mocks
            update drone map
*/
int Simulator::run() {
    /*
        Step 1:
        Build paths to all required files.

        Input files:
        - drone_config.txt
        - mission_config.txt
        - map_input.txt

        Output file:
        - map_output.txt
    */
    std::string droneConfigPath = makePath("drone_config.txt");
    std::string missionConfigPath = makePath("mission_config.txt");
    std::string mapInputPath = makePath("map_input.txt");
    std::string mapOutputPath = makePath("map_output.txt");

    /*
        Step 2:
        Parse the configuration files.

        droneConfig:
        Contains drone capabilities, such as max advance distance.

        missionConfig:
        Contains mission-specific information, such as start position.

        worldMap:
        This is the real hidden map of the building.
        The drone itself should NOT directly use this map.
        Only the simulator and mock sensors may use it.
    */
    DroneConfig droneConfig = ConfigParser::parseDroneConfig(droneConfigPath);
    MissionConfig missionConfig = ConfigParser::parseMissionConfig(missionConfigPath);
    GroundTruthMap worldMap = ConfigParser::parseMapInput(mapInputPath);

    /*
        Debug prints.

        These are useful while we are still building the project.
        Later, we can decide whether to keep them or replace them with logging.
    */
    std::cout << "Drone max advance: "
              << droneConfig.maxAdvanceCm
              << " cm"
              << std::endl;

    std::cout << "Mission start position: "
              << missionConfig.startPosition.x << ", "
              << missionConfig.startPosition.y << ", "
              << missionConfig.startPosition.height
              << std::endl;

    /*
        Step 3:
        Create the drone's internal map.

        worldMap is the real hidden map.
        droneMap is what the drone has discovered so far.

        At the beginning, the droneMap should mostly contain Unmapped cells.
    */
    SparseBuildingMap droneMap(
        worldMap.getSizeX(),
        worldMap.getSizeY(),
        worldMap.getSizeZ()
    );

    /*
        Step 4:
        Create the real drone state inside the simulator.

        DroneState stores the actual current pose of the drone:
        - position: x, y, height
        - xyAngle: direction in the XY plane

        The assignment uses:
        0   = east
        90  = south
        180 = west
        270 = north
    */
    DroneState droneState;
    droneState.pose.position = missionConfig.startPosition;
    droneState.pose.xyAngle = missionConfig.startAngleDeg;

    /*
        Step 5:
        Create the mock position sensor.

        The drone will eventually receive an IPositionSensor interface.
        But in our simulator, we use MockPositionSensor.

        The sensor reads from droneState and returns the current pose.
    */
    MockPositionSensor positionSensor(droneState);

    MockMovementDriver movementDriver(droneState, droneConfig, worldMap);

    /*
        Test the position sensor.

        This confirms that the sensor correctly reads the initial drone position.
    */
    Pose sensedPose = positionSensor.getPose();

    std::cout << "Position sensor says drone is at: "
              << sensedPose.position.x << ", "
              << sensedPose.position.y << ", "
              << sensedPose.position.height
              << " angle "
              << sensedPose.xyAngle
              << " degrees"
              << std::endl;

    // Mark the starting position as free
    droneMap.setCell(missionConfig.startPosition, CellState::Free);

    // Temporary movement test
    bool moved = movementDriver.advance(1);

    if (moved) {
        Pose afterMovePose = positionSensor.getPose();

        std::cout << "After advance, drone is at: "
                << afterMovePose.position.x << ", "
                << afterMovePose.position.y << ", "
                << afterMovePose.position.height
                << " angle "
                << afterMovePose.xyAngle
                << " degrees"
                << std::endl;

        droneMap.setCell(afterMovePose.position, CellState::Free);
    }


    /*
        Step 6:
        Write the drone's discovered map into map_output.txt.

        This file should use the same format as map_input.txt.
    */
    bool wroteMap = MapFileWriter::writeSparseMap(mapOutputPath, droneMap);

    if (!wroteMap) {
        std::cout << "Failed to write map_output.txt" << std::endl;
        return 1;
    }

    std::cout << "Wrote output map to: "
              << mapOutputPath
              << std::endl;

    /*
        Step 7:
        Calculate the mapping score.

        We compare:
        - worldMap: the real hidden map
        - droneMap: the map discovered by the drone

        The score is between 0 and 100.
    */
    double score = ScoreCalculator::calculateScore(worldMap, droneMap);

    std::cout << "Mapping score: "
              << score
              << "/100"
              << std::endl;

    /*
        Simulation finished successfully.
    */
    return 0;
}