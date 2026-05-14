#ifndef ILIDAR_SENSOR_H
#define ILIDAR_SENSOR_H

#include "Types.h"

class ILidarSensor {
public:
    virtual ~ILidarSensor() = default;

    // Scan relative to the drone's current direction.
    virtual ScanResult scan(const ScanAngle& scanAngle) const = 0;
};

#endif