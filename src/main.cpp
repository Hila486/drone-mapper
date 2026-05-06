#include <iostream>

#include "Types.h"
#include "DroneConfig.h"
#include "MissionConfig.h"
#include "GroundTruthMap.h"
#include "SparseBuildingMap.h"
#include "DroneState.h"
#include "MockPositionSensor.h"

int main() {
    DroneConfig droneConfig;
    droneConfig.maxAdvanceCm = 100;
    droneConfig.maxElevateCm = 50;
    droneConfig.maxRotateDeg = 90;
    droneConfig.lidarRangeCm = 500;

    MissionConfig missionConfig;
    missionConfig.startPosition = Position{0, 0, 0};
    missionConfig.startAngleDeg = 0;
    missionConfig.minX = 0;
    missionConfig.maxX = 10;
    missionConfig.minY = 0;
    missionConfig.maxY = 10;
    missionConfig.minZ = 0;
    missionConfig.maxZ = 3;
    missionConfig.resolutionCm = 100;

    std::cout << "Drone max advance: "
              << droneConfig.maxAdvanceCm
              << " cm"
              << std::endl;

    std::cout << "Mission start position: "
              << missionConfig.startPosition.x << ", "
              << missionConfig.startPosition.y << ", "
              << missionConfig.startPosition.z
              << std::endl;

    // Create the real world map.
    // This is the map that only the simulator and mock sensors should know.
    GroundTruthMap worldMap(10, 10, 3);

    Position wallPosition{2, 3, 0};
    worldMap.setCell(wallPosition, CellState::Occupied);

    CellState realState = worldMap.getCell(wallPosition);

    if (realState == CellState::Occupied) {
        std::cout << "GroundTruthMap: cell at "
                  << wallPosition.x << ", "
                  << wallPosition.y << ", "
                  << wallPosition.z
                  << " is occupied"
                  << std::endl;
    }

    Position outsidePosition{100, 100, 100};
    CellState outsideState = worldMap.getCell(outsidePosition);

    if (outsideState == CellState::OutOfBounds) {
        std::cout << "GroundTruthMap: outside position is out of bounds"
                  << std::endl;
    }

    // Create the drone's discovered map.
    // This is the map that starts unknown and will be updated by the drone.
    SparseBuildingMap droneMap(10, 10, 3);

    Position testPosition{2, 3, 0};
    CellState droneInitialState = droneMap.getCell(testPosition);

    if (droneInitialState == CellState::Unmapped) {
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

    // Create the real current state of the drone.
    // This stores the drone's actual position and angle in the simulation.
    DroneState droneState;
    droneState.pose.position = missionConfig.startPosition;
    droneState.pose.angleDegrees = missionConfig.startAngleDeg;

    // Create a mock position sensor.
    // The sensor reads from DroneState, but it does not change it.
    MockPositionSensor positionSensor(droneState);

    // Ask the position sensor for the current pose.
    Pose sensedPose = positionSensor.getPose();

    std::cout << "Position sensor says drone is at: "
              << sensedPose.position.x << ", "
              << sensedPose.position.y << ", "
              << sensedPose.position.z
              << " angle "
              << sensedPose.angleDegrees
              << " degrees"
              << std::endl;

    return 0;
}