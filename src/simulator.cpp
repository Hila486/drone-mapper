#include "simulator.h"

#include <cmath>
#include <iostream>

#include "Types.h"
#include "DroneConfig.h"
#include "MissionConfig.h"
#include "GroundTruthMap.h"
#include "SparseBuildingMap.h"
#include "DroneState.h"
#include "MockPositionSensor.h"
#include "MockMovementDriver.h"
#include "MockLidarSensor.h"
#include "ConfigParser.h"
#include "MapFileWriter.h"
#include "ScoreCalculator.h"

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

    Example:
    inputOutputPath = ".."
    fileName = "map_input.txt"

    Result:
    "../map_input.txt"
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
    - Clears previous input error records.
    - Loads input files.
    - Writes input_errors.txt only if recoverable input errors exist.
    - Creates hidden real world map.
    - Creates drone discovered map.
    - Creates mock position sensor.
    - Creates mock movement driver.
    - Creates mock lidar sensor.
    - Runs a temporary movement + lidar test.
    - Stores lidar hit into the drone map when distance is accurate.
    - Tests collision blocking in MockMovementDriver.
    - Writes map_output.txt.
    - Calculates score.

    Later, the temporary test will be replaced by the real autonomous mapping loop.
*/
int Simulator::run() {
    /*
        Step 0:
        Clear old recoverable input errors before parsing this run.
    */
    ConfigParser::clearInputErrors();

    /*
        Step 1:
        Build paths to all required input/output files.
    */
    std::string droneConfigPath = makePath("drone_config.txt");
    std::string missionConfigPath = makePath("mission_config.txt");
    std::string mapInputPath = makePath("map_input.txt");
    std::string mapOutputPath = makePath("map_output.txt");
    std::string inputErrorsPath = makePath("input_errors.txt");

    /*
        Step 2:
        Parse input files.

        If recoverable input errors are found, ConfigParser stores them.
        After parsing, we write them into input_errors.txt.
    */
    DroneConfig droneConfig = ConfigParser::parseDroneConfig(droneConfigPath);
    MissionConfig missionConfig = ConfigParser::parseMissionConfig(missionConfigPath);
    GroundTruthMap worldMap = ConfigParser::parseMapInput(mapInputPath);

    ConfigParser::writeInputErrors(inputErrorsPath);

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
        Create the drone's internal discovered map.

        worldMap = hidden real map.
        droneMap = what the drone discovered so far.
    */
    SparseBuildingMap droneMap(
        worldMap.getSizeX(),
        worldMap.getSizeY(),
        worldMap.getSizeZ()
    );

    /*
        Step 4:
        Create real drone state inside the simulator.
    */
    DroneState droneState;
    droneState.pose.position = missionConfig.startPosition;
    droneState.pose.xyAngle = missionConfig.startAngleDeg;

    /*
        Step 5:
        Create mocks.

        Position sensor reads the current DroneState.
        Movement driver updates DroneState.
        Lidar sensor reads DroneState through the position sensor
        and scans the hidden GroundTruthMap.
    */
    MockPositionSensor positionSensor(droneState);
    MockMovementDriver movementDriver(droneState, droneConfig, worldMap);
    MockLidarSensor lidarSensor(droneConfig, worldMap, positionSensor);

    /*
        Print initial pose.
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

    /*
        Temporary mapping test.

        Mark the start position as free because the drone starts there.
    */
    droneMap.setCell(missionConfig.startPosition, CellState::Free);

    /*
        Temporary movement test 1:
        Move from (0,0,0) to (1,0,0), assuming the drone starts facing east.
    */
    bool movedFirst = movementDriver.advance(1);

    if (movedFirst) {
        Pose afterFirstMovePose = positionSensor.getPose();

        std::cout << "After first advance, drone is at: "
                  << afterFirstMovePose.position.x << ", "
                  << afterFirstMovePose.position.y << ", "
                  << afterFirstMovePose.position.height
                  << " angle "
                  << afterFirstMovePose.xyAngle
                  << " degrees"
                  << std::endl;

        droneMap.setCell(afterFirstMovePose.position, CellState::Free);
    }

    /*
        Temporary movement test 2:
        Move one more step east.

        If start was (0,0,0), now the drone should be at (2,0,0).
    */
    bool movedSecond = movementDriver.advance(1);

    if (movedSecond) {
        Pose afterSecondMovePose = positionSensor.getPose();

        std::cout << "After second advance, drone is at: "
                  << afterSecondMovePose.position.x << ", "
                  << afterSecondMovePose.position.y << ", "
                  << afterSecondMovePose.position.height
                  << " angle "
                  << afterSecondMovePose.xyAngle
                  << " degrees"
                  << std::endl;

        droneMap.setCell(afterSecondMovePose.position, CellState::Free);
    }

    /*
        Temporary rotation test:
        Turn right by 90 degrees.

        According to the assignment convention:
        0   = east
        90  = south
        180 = west
        270 = north

        So after this rotation, the drone should face south.
    */
    bool rotated = movementDriver.rotate(RotationDirection::Right, 90);

    if (rotated) {
        Pose afterRotatePose = positionSensor.getPose();

        std::cout << "After rotate, drone is at: "
                  << afterRotatePose.position.x << ", "
                  << afterRotatePose.position.y << ", "
                  << afterRotatePose.position.height
                  << " angle "
                  << afterRotatePose.xyAngle
                  << " degrees"
                  << std::endl;
    }

    /*
        Temporary LiDAR test.

        ScanAngle(0, 0) means:
        - xyAngle = 0: scan straight ahead relative to the drone direction.
        - heightAngle = 0: scan at the same height.

        If there is an occupied cell directly south of the drone,
        the LiDAR should return one hit.
    */
    Pose beforeLidarPose = positionSensor.getPose();

    std::cout << "Before lidar test, drone is at: "
              << beforeLidarPose.position.x << ", "
              << beforeLidarPose.position.y << ", "
              << beforeLidarPose.position.height
              << " angle "
              << beforeLidarPose.xyAngle
              << " degrees"
              << std::endl;

    ScanResult scanResult = lidarSensor.scan(ScanAngle(0, 0));

    std::cout << "Lidar scan returned "
              << scanResult.size()
              << " hits"
              << std::endl;

    /*
        Convert every LiDAR hit into a map cell and store it
        in the drone's discovered map.

        If hit.distance == 0, it means the object is below Z-min.
        The LiDAR knows there is something close, but cannot accurately
        measure where it is, so we do not mark a specific occupied cell.
    */
    for (const ScanHit& hit : scanResult) {
        std::cout << "Hit at relative xy angle "
                  << hit.angle.xyAngle
                  << ", height angle "
                  << hit.angle.heightAngle
                  << ", distance "
                  << hit.distance
                  << " cm"
                  << std::endl;

        if (hit.distance <= 0) {
            std::cout << "Hit is too close to map accurately"
                      << std::endl;
            continue;
        }

        /*
            The hit angle is relative to the drone direction,
            so we convert it to an absolute world angle.

            Example:
            drone xyAngle = 90
            hit relative xyAngle = 0

            absolute angle = 90, meaning south.
        */
        Pose currentPose = positionSensor.getPose();

        Degree absoluteXyAngle =
            (currentPose.xyAngle + hit.angle.xyAngle) % 360;

        if (absoluteXyAngle < 0) {
            absoluteXyAngle += 360;
        }

        const double pi = 3.14159265358979323846;

        double xyRadians =
            static_cast<double>(absoluteXyAngle) * pi / 180.0;

        double heightRadians =
            static_cast<double>(hit.angle.heightAngle) * pi / 180.0;

        double horizontalFactor = std::cos(heightRadians);

        double dx = horizontalFactor * std::cos(xyRadians);
        double dy = horizontalFactor * std::sin(xyRadians);
        double dh = std::sin(heightRadians);

        Position hitPosition{
            static_cast<Cm>(
                std::round(currentPose.position.x + dx * hit.distance)
            ),
            static_cast<Cm>(
                std::round(currentPose.position.y + dy * hit.distance)
            ),
            static_cast<Cm>(
                std::round(currentPose.position.height + dh * hit.distance)
            )
        };

        droneMap.setCell(hitPosition, CellState::Occupied);

        std::cout << "Marked LiDAR hit as occupied at: "
                  << hitPosition.x << ", "
                  << hitPosition.y << ", "
                  << hitPosition.height
                  << std::endl;
    }

    /*
        Temporary collision test for MockMovementDriver.

        The drone is currently at (2,0,0), facing south.
        There is an occupied cell at (2,3,0).

        If advance(5) checks every intermediate cell correctly,
        it should fail before entering the occupied cell.
    */
    std::cout << "Testing blocked movement through wall..." << std::endl;

    bool movedThroughWall = movementDriver.advance(5);

    if (!movedThroughWall) {
        std::cout << "Correct: movement was blocked before collision"
                  << std::endl;
    } else {
        std::cout << "Warning: movement succeeded, but it should have been blocked"
                  << std::endl;
    }

    Pose afterBlockedMovePose = positionSensor.getPose();

    std::cout << "After blocked movement test, drone is at: "
              << afterBlockedMovePose.position.x << ", "
              << afterBlockedMovePose.position.y << ", "
              << afterBlockedMovePose.position.height
              << " angle "
              << afterBlockedMovePose.xyAngle
              << " degrees"
              << std::endl;

    /*
        Step 6:
        Write the drone's discovered map into map_output.txt.
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
        Calculate mapping score.
    */
    double score = ScoreCalculator::calculateScore(worldMap, droneMap);

    std::cout << "Mapping score: "
              << score
              << "/100"
              << std::endl;

    return 0;
}