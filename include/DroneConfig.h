#pragma once

/*
    DroneConfig

    Stores the drone capabilities loaded from drone_config.txt.

    Assignment terms:
    - Z-min: minimum lidar distance for accurate measurement
    - Z-max: maximum lidar detection distance
    - D: spacing between beam circles
    - FOVC: number of lidar beam circles

    For compatibility with our earlier code, we keep:
    - lidarRangeCm
    - lidarResolutionCm

    Later, ConfigParser should fill the newer fields directly.
*/
struct DroneConfig {
    // Movement limits
    int maxAdvanceCm = 0;
    int maxElevateCm = 0;
    int maxRotateDeg = 0;

    /*
        Lidar settings from the assignment.

        lidarMinRangeCm = Z-min
        If an object is detected below this distance,
        the lidar returns distance 0.

        lidarMaxRangeCm = Z-max
        Beyond this distance, lidar detects nothing.

        lidarBeamSpacingCm = D
        Spacing between lidar beam circles.

        lidarFovCircleCount = FOVC
        Number of beam circles.
        FOVC = 1 means only the central beam.
    */
    int lidarMinRangeCm = 0;
    int lidarMaxRangeCm = 0;
    int lidarBeamSpacingCm = 1;
    int lidarFovCircleCount = 1;

    /*
        Older fields from our first implementation.

        Keep these for now so older config parsing still works.
        Later we can remove them after ConfigParser is updated.
    */
    int lidarRangeCm = 0;
    int lidarResolutionCm = 1;

    /*
        Minimum passage size.

        Updated assignment says in Ex1 we may assume the drone
        is a perfect sphere.

        For now, we keep width/height because our project already uses them.
        Later we can replace them with one diameter/radius value.
    */
    int minPassWidthCm = 0;
    int minPassHeightCm = 0;
};