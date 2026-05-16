#include "MockLidarSensor.h"

#include <cmath>
#include <set>
#include <utility>
#include <vector>

/*
    Small helper only for this file.

    Converts radians to degrees.
*/
namespace {
    double radiansToDegrees(double radians) {
        const double pi = 3.14159265358979323846;
        return radians * 180.0 / pi;
    }
}

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

    The input scanAngle is the CENTER direction of the scan,
    relative to the drone direction.

    Assignment LiDAR model:
    - Circle 0: one central beam.
    - Circle 1: 4 beams.
    - Circle 2: 16 beams.
    - Circle 3: 64 beams.
    - etc.

    FOVC controls how many circles exist.
    FOVC = 1 means only Circle 0.
*/
ScanResult MockLidarSensor::scan(const ScanAngle& scanAngle) const {
    ScanResult result;

    Pose currentPose = positionSensor.getPose();

    int circleCount = droneConfig.lidarFovCircleCount;

    if (circleCount <= 0) {
        circleCount = 1;
    }

    int spacingCm = droneConfig.lidarBeamSpacingCm;

    if (spacingCm <= 0) {
        spacingCm = 1;
    }

    /*
        Z-min is used to convert the circle radius into an angular offset.

        The assignment defines D as spacing between beam circles at distance Z-min.

        If Z-min is 0 in our simple test config, use 1 to avoid division by zero.
    */
    int zMin = droneConfig.lidarMinRangeCm;

    if (zMin <= 0) {
        zMin = 1;
    }

    /*
        Prevent duplicate beams.

        Because our Degree type is currently int, small angular offsets may round
        to the same angle. The set avoids scanning the same beam direction twice.
    */
    std::set<std::pair<Degree, Degree>> usedRelativeAngles;

    /*
        Circle 0:
        central beam exactly in the requested direction.
    */
    usedRelativeAngles.insert({
        scanAngle.xyAngle,
        scanAngle.heightAngle
    });

    Degree absoluteCenterXyAngle =
        normalizeAngle(currentPose.xyAngle + scanAngle.xyAngle);

    Cm hitDistance = 0;

    bool centerHit = traceBeam(
        absoluteCenterXyAngle,
        scanAngle.heightAngle,
        hitDistance
    );

    if (centerHit) {
        result.push_back(ScanHit(scanAngle, hitDistance));
    }

    /*
        Outer circles.

        Circle 1 has 4 beams.
        Circle 2 has 16 beams.
        Circle 3 has 64 beams.
        In general: beams = 4^circleIndex.
    */
    int beamsInCircle = 1;

    for (int circleIndex = 1; circleIndex < circleCount; ++circleIndex) {
        beamsInCircle *= 4;

        /*
            Radius of this circle at distance Z-min.
            Assignment: Circle 1 radius = D, Circle 2 radius = 2D, etc.
        */
        double radiusAtZMin =
            static_cast<double>(circleIndex * spacingCm);

        /*
            Convert physical circle radius into approximate angular offset.

            angle = atan(radius / zMin)
        */
        double angleOffsetDegrees =
            radiansToDegrees(std::atan(radiusAtZMin / static_cast<double>(zMin)));

        const double pi = 3.14159265358979323846;

        for (int beamIndex = 0; beamIndex < beamsInCircle; ++beamIndex) {
            double phase =
                2.0 * pi * static_cast<double>(beamIndex) /
                static_cast<double>(beamsInCircle);

            /*
                Approximate the 3D circle around the central beam using:
                - cos phase for XY angle offset
                - sin phase for height angle offset
            */
            Degree relativeXyOffset =
                static_cast<Degree>(std::round(angleOffsetDegrees * std::cos(phase)));

            Degree relativeHeightOffset =
                static_cast<Degree>(std::round(angleOffsetDegrees * std::sin(phase)));

            ScanAngle beamRelativeAngle{
                scanAngle.xyAngle + relativeXyOffset,
                scanAngle.heightAngle + relativeHeightOffset
            };

            std::pair<Degree, Degree> key{
                beamRelativeAngle.xyAngle,
                beamRelativeAngle.heightAngle
            };

            if (usedRelativeAngles.find(key) != usedRelativeAngles.end()) {
                continue;
            }

            usedRelativeAngles.insert(key);

            Degree beamAbsoluteXyAngle =
                normalizeAngle(currentPose.xyAngle + beamRelativeAngle.xyAngle);

            Cm beamHitDistance = 0;

            bool hit = traceBeam(
                beamAbsoluteXyAngle,
                beamRelativeAngle.heightAngle,
                beamHitDistance
            );

            if (hit) {
                result.push_back(ScanHit(beamRelativeAngle, beamHitDistance));
            }
        }
    }

    return result;
}

/*
    Traces one beam through the map.

    Current model:
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
        Use assignment field first:
        lidarMaxRangeCm = Z-max.

        If not parsed yet, fall back to older field:
        lidarRangeCm.

        If both are missing, use a safe default.
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

        If lidarMinRangeCm is 0, every detected distance is considered accurate.
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
                but cannot accurately measure distance.
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