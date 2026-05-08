#ifndef UNITS_H
#define UNITS_H

// Units.h
// Shared constants and simple unit aliases for the Drone Mapper project.

// --------------------
// Cell values
// --------------------
// These values are used in map_input.txt and map_output.txt.

constexpr int UNKNOWN_CELL = -1;
constexpr int FREE_CELL = 0;
constexpr int OCCUPIED_CELL = 1;
constexpr int OUT_OF_BOUNDS = -2;

// --------------------
// Units
// --------------------
// Project convention:
// - distances are in centimeters
// - angles are in degrees

// Input/output files use centimeters and degrees.


using Cm = int;
using Degree = int;


#endif