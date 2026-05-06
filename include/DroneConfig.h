#pragma once

// Defines the DroneConfig struct, which stores the drone's capabilities
// such as movement limits, lidar settings, and minimum passage size.

struct DroneConfig {
    // Movement limits
    int maxAdvanceCm = 0;
    int maxElevateCm = 0;
    int maxRotateDeg = 0;

    // Lidar settings
    int lidarRangeCm = 0;
    int lidarResolutionCm = 0;

    // Minimum passage size the drone can pass through
    int minPassWidthCm = 0;
    int minPassHeightCm = 0;
};