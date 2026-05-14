#include "../include/drone.h"
#include "../include/MissionConfig.h"

Drone::Drone(ILidarSensor& lidar, 
             IPositionSensor& positionSensor,
             IMovementDriver& movementDriver,
             IBuildingMap& buildingMap,
             const MissionConfig& missionConfig)
    : lidar(lidar), 
      positionSensor(positionSensor),
      movementDriver(movementDriver),
      buildingMap(buildingMap),
      missionConfig(missionConfig),
      finished(false),
      initialScanDone(false) {
}
/*
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
*/

/*
 Command nextCommand();
    bool isFinished() const;
*/

bool Drone::isFinished() const { // Did the drone finish the mission?
    return finished;
}

Command Drone::nextCommand() {
    Pose pose;

    if (finished) {
        return Command{CommandType::Finished}; // return Command object and define the filed CommandType (other fileds's values are default values)
    }

    handleLocationUpdate();

    if (!initialScanDone) {
        initialScanDone = true;
        handleScan();
        return Command{CommandType::Scan};
    }

    handleScan();

    if (tryAdvanceToNeighbor()) {
        return Command{CommandType::Advance};
    }

    finished = true;
    return Command{CommandType::Finished};
}

void Drone::handleScan() {
    Pose pose = positionSensor.getPose();

    /* 
       Adjust this call to your real ILidarSensor API.
       According to your design doc, lidar scan may take orientation.
    */
    ScanResult scan = lidar.scan(pose.orientation);

    updateMapFromScan(pose, scan);
}

void Drone::handleLocationUpdate() {
    markCurrentCellFree();
}

void Drone::markCurrentCellFree() {
    Pose pose = positionSensor.getPose();
    buildingMap.set(pose.position, CellState::Free);
}

void Drone::updateMapFromScan(const Pose& pose, const ScanResult& scan) {
    /*
      This is intentionally conservative.

      What this function SHOULD do in your real implementation:
      1. For each lidar hit:
         - convert the beam direction + distance into world coordinates
         - mark cells before the hit as Free
         - mark the hit cell as Occupied
      2. Do NOT guess blind spots.
      3. If distance == 0 (too close), be careful and do not mark a long free segment.

      Because I do not know your exact ScanResult structure, this is left as a skeleton.
    */

    (void)pose;
    (void)scan;

    /* Example shape only:

    for (const auto& hit : scan.hits) {
        std::vector<Position> cellsOnBeam = ...;

        for (std::size_t i = 0; i + 1 < cellsOnBeam.size(); ++i) {
            if (inMissionBounds(cellsOnBeam[i])) {
                buildingMap.set(cellsOnBeam[i], CellState::Free);
            }
        }

        if (!cellsOnBeam.empty() && inMissionBounds(cellsOnBeam.back())) {
            buildingMap.set(cellsOnBeam.back(), CellState::Occupied);
        }
    }
    */
}

std::vector<Position> Drone::candidateNeighbors(const Position& p) const {
    std::vector<Position> neighbors;

    /*
      Deterministic order.
      Change z/height field name if your Position uses height instead of z.
    */
    neighbors.push_back(Position{p.x + 1, p.y, p.z});
    neighbors.push_back(Position{p.x, p.y + 1, p.z});
    neighbors.push_back(Position{p.x - 1, p.y, p.z});
    neighbors.push_back(Position{p.x, p.y - 1, p.z});
    neighbors.push_back(Position{p.x, p.y, p.z + 1});
    neighbors.push_back(Position{p.x, p.y, p.z - 1});

    return neighbors;
}

bool Drone::isKnownFree(const Position& p) const {
    return buildingMap.get(p) == CellState::Free;
}

bool Drone::isKnownUnmapped(const Position& p) const {
    return buildingMap.get(p) == CellState::Unmapped;
}

bool Drone::inMissionBounds(const Position& p) const {
    /*
      Adjust these field names to your real MissionConfig.
      This is based on the assignment: min/max X, Y, and height.
    */
    return p.x >= missionConfig.minX &&
           p.x <= missionConfig.maxX &&
           p.y >= missionConfig.minY &&
           p.y <= missionConfig.maxY &&
           p.z >= missionConfig.minHeight &&
           p.z <= missionConfig.maxHeight;
}

bool Drone::tryAdvanceToNeighbor() {
    Pose pose = positionSensor.getPose();
    std::vector<Position> neighbors = candidateNeighbors(pose.position);
    std::size_t i;

    for (i = 0; i < neighbors.size(); ++i) {
        if (!inMissionBounds(neighbors[i])) {
            continue;
        }

        /*
          For a first simple version:
          - prefer unmapped neighbors
          - also allow known free neighbors
        */
        if (isKnownUnmapped(neighbors[i]) || isKnownFree(neighbors[i])) {
            /*
              This is only a placeholder idea.
              In a real implementation you should:
              1. compare current pose to target neighbor
              2. rotate if needed
              3. elevate if z differs
              4. advance if horizontal neighbor is ahead
            */

            return movementDriver.advance(/* one step distance */);
        }
    }

    return false;
}