#ifndef ILIDAR_SENSOR_H
#define ILIDAR_SENSOR_H

#include "Types.h"
class ILidarSensor {
public:
    virtual ScanResult scan() const = 0;
    virtual ~ILidarSensor() = default;
};

#endif
