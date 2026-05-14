#pragma once

#include <vector>
#include <optional>
#include "Types.h"
#include "ILidarSensor.h"
#include "IPositionSensor.h"
#include "IMovementDriver.h"
#include "IBuildingMap.h"
#include "MissionConfig.h"

class Drone {
public:
    Drone(ILidarSensor& lidar,
          IPositionSensor& positionSensor,
          IMovementDriver& movementDriver,
          IBuildingMap& buildingMap,
          const MissionConfig& missionConfig);

    Command nextCommand();
    bool isFinished() const;

private:
    ILidarSensor& lidar;
    IPositionSensor& positionSensor;
    IMovementDriver& movementDriver;
    IBuildingMap& buildingMap;
    MissionConfig missionConfig;

    bool finished = false;

    // simple deterministic control state
    bool initialScanDone = false;
    std::vector<Position> frontier;
    std::vector<Position> visitedPath;

    // private helper function for the drone's internal logic
    void handleScan(); // asks for position and for the lidar scan ---> update drone internal map
    void handleLocationUpdate(); // if the drone is in current location then update that location as free
    bool tryAdvanceToNeighbor(); // this function will advance to the next cell
    void markCurrentCellFree();  // also update the drones current location as free ?
    void updateMapFromScan(const Pose& pose, const ScanResult& scan); // // from current location, scan and:
    // 1. cells that were hitted by the beams mark as ocupied 2. cells that the beams travels along them mark as free 3. don't touch blind spots

    std::vector<Position> candidateNeighbors(const Position& p) const; // return nearby postions next to the drone
    bool isKnownFree(const Position& p) const; // between the cells that are discovered, is that cell is free ?
    bool isKnownUnmapped(const Position& p) const; // checks wether a given postion in the drone's map is considred as umapped
    bool inMissionBounds(const Position& p) const; // checkes wether a postion is in the mission bounds.
};