#include "MockLidarSensor.h"

#include <cmath>

/*
    Constructor.

    We store references because:
    - droneConfig is shared configuration
    - worldMap is the hidden real map
    - positionSensor gives us the current drone pose
*/
MockLidarSensor::MockLidarSensor(
    const DroneConfig& droneConfig,
    const GroundTruthMap& worldMap,
    const IPositionSensor& positionSensor
)
    : droneConfig(droneConfig),
      worldMap(worldMap),
      positionSensor(positionSensor) {
}

/*
    Converts any angle to [0, 359].

    Examples:
    360  -> 0
    -90  -> 270
    450  -> 90
*/
Degree MockLidarSensor::normalizeAngle(Degree angle) const {
    Degree result = angle % 360;

    if (result < 0) {
        result += 360;
    }

    return result;
}

/*
    Main scan function.

    The input scanAngle is RELATIVE to the drone direction.

    The mock sensor internally converts it to an absolute angle using:
        absolute angle = drone current angle + requested relative scan angle

    First version:
    - We still scan one central beam.
    - Next improvement can add outer beam circles using FOVC and D.
*/
ScanResult MockLidarSensor::scan(const ScanAngle& scanAngle) const {
    ScanResult result;

    Pose currentPose = positionSensor.getPose();

    Degree absoluteXyAngle =
        normalizeAngle(currentPose.xyAngle + scanAngle.xyAngle);

    Cm hitDistance = 0;

    bool hit = traceBeam(
        absoluteXyAngle,
        scanAngle.heightAngle,
        hitDistance
    );

    if (hit) {
        result.push_back(ScanHit(scanAngle, hitDistance));
    }

    return result;
}

/*
    Traces one beam through the map.

    Current model:
    - one central beam
    - one sample every 1 cm
    - first occupied cell stops the beam
    - outside map means the beam leaves the known world, so scanning stops

    Assignment behavior:
    - If the hit is closer than Z-min, return distance 0.
    - If no hit is found up to Z-max, return no hit.

    Angle convention:
    0   = east  => +x
    90  = south => +y
    180 = west  => -x
    270 = north => -y

    heightAngle:
    0   = same height
    positive = upward
    negative = downward
*/
bool MockLidarSensor::traceBeam(
    Degree absoluteXyAngle,
    Degree heightAngle,
    Cm& hitDistance
) const {
    Pose currentPose = positionSensor.getPose();
    Position origin = currentPose.position;

    /*
        Use the new assignment field first: lidarMaxRangeCm = Z-max.

        If it was not parsed yet, fall back to our old field:
        lidarRangeCm.

        If both are missing, use a safe default for testing.
    */
    Cm maxDistance = droneConfig.lidarMaxRangeCm;

    if (maxDistance <= 0) {
        maxDistance = droneConfig.lidarRangeCm;
    }

    if (maxDistance <= 0) {
        maxDistance = 100;
    }

    /*
        Z-min.

        If lidarMinRangeCm is 0, then every detected distance is considered accurate.
    */
    Cm minAccurateDistance = droneConfig.lidarMinRangeCm;

    const double pi = 3.14159265358979323846;

    double xyRadians =
        static_cast<double>(absoluteXyAngle) * pi / 180.0;

    double heightRadians =
        static_cast<double>(heightAngle) * pi / 180.0;

    /*
        Convert angle direction to a 3D direction vector.

        cos(height) keeps horizontal movement smaller when the beam points up/down.
    */
    double horizontalFactor = std::cos(heightRadians);

    double dx = horizontalFactor * std::cos(xyRadians);
    double dy = horizontalFactor * std::sin(xyRadians);
    double dh = std::sin(heightRadians);

    for (Cm distance = 1; distance <= maxDistance; ++distance) {
        Position sample{
            static_cast<Cm>(std::round(origin.x + dx * distance)),
            static_cast<Cm>(std::round(origin.y + dy * distance)),
            static_cast<Cm>(std::round(origin.height + dh * distance))
        };

        /*
            Do not report the drone's own current cell as a hit.
        */
        if (sample.x == origin.x &&
            sample.y == origin.y &&
            sample.height == origin.height) {
            continue;
        }

        CellState cell = worldMap.getCell(sample);

        if (cell == CellState::OutOfBounds) {
            return false;
        }

        if (cell == CellState::Occupied) {
            /*
                Assignment rule:
                If the object is below Z-min, the lidar detects it,
                but cannot accurately measure the distance.
                Therefore return distance 0.
            */
            if (minAccurateDistance > 0 && distance < minAccurateDistance) {
                hitDistance = 0;
            } else {
                hitDistance = distance;
            }

            return true;
        }
    }

    return false;
}