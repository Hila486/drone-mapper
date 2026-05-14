#pragma once

#include "ILidarSensor.h"
#include "IPositionSensor.h"
#include "GroundTruthMap.h"
#include "DroneConfig.h"

/*
    MockLidarSensor

    This class simulates the drone's LiDAR sensor.

    Important design rule:
    - The mock sensor CAN see the hidden GroundTruthMap.
    - The drone algorithm itself should NOT see GroundTruthMap directly.

    The sensor:
    1. Gets the current drone pose from IPositionSensor.
    2. Converts the requested scan angle into an absolute world angle.
    3. Ray-marches through GroundTruthMap.
    4. Returns the first occupied cell hit by the beam.

    First version:
    - Only one central beam.
    - Later we can add the full FOV circle logic like the TA version.
*/
class MockLidarSensor : public ILidarSensor {
public:
    MockLidarSensor(
        const DroneConfig& droneConfig,
        const GroundTruthMap& worldMap,
        const IPositionSensor& positionSensor
    );

    /*
        Scans relative to the drone's current direction.

        Example:
        - drone faces east, xyAngle = 0
        - scanAngle.xyAngle = 0
        => scan east

        - drone faces east, scanAngle.xyAngle = 90
        => scan south

        Returns:
        - empty ScanResult if nothing was hit
        - one ScanHit if the central beam hit an occupied cell
    */
    ScanResult scan(const ScanAngle& scanAngle) const override;

private:
    const DroneConfig& droneConfig;
    const GroundTruthMap& worldMap;
    const IPositionSensor& positionSensor;

    Degree normalizeAngle(Degree angle) const;

    /*
        Traces one beam through the hidden map.

        Returns true if the beam hit an occupied cell.
        hitDistance will contain the distance from the drone to the hit.
    */
    bool traceBeam(
        Degree absoluteXyAngle,
        Degree heightAngle,
        Cm& hitDistance
    ) const;
};