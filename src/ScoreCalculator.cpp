#include "ScoreCalculator.h"

#include <algorithm>

/*
    calculateScore

    Compares the real hidden world map with the drone's discovered map.

    groundTruthMap:
        The real map loaded from map_input.txt.
        This is the "answer".

    droneMap:
        The map discovered by the drone.
        This is what we wrote to map_output.txt.

    Simple score formula:

        score = correct cells / total cells * 100

    Example:
        real map:  Occupied
        drone map: Occupied
        => correct

        real map:  Free
        drone map: Unknown
        => wrong

    The function returns a number between 0 and 100.
*/
double ScoreCalculator::calculateScore(
    const GroundTruthMap& groundTruthMap,
    const SparseBuildingMap& droneMap
) {
    /*
        We use the smaller size in each dimension.

        Usually both maps should have the same size.
        But using std::min protects us if something is different by mistake.
    */
    int sizeX = std::min(groundTruthMap.getSizeX(), droneMap.getSizeX());
    int sizeY = std::min(groundTruthMap.getSizeY(), droneMap.getSizeY());
    int sizeZ = std::min(groundTruthMap.getSizeZ(), droneMap.getSizeZ());

    int totalCells = sizeX * sizeY * sizeZ;

    if (totalCells == 0) {
        return 0.0;
    }

    int correctCells = 0;

    /*
        Go over every cell in the map.

        Loop order:
        1. height layer
        2. y row
        3. x column
    */
    for (int height = 0; height < sizeZ; ++height) {
        for (int y = 0; y < sizeY; ++y) {
            for (int x = 0; x < sizeX; ++x) {
                Position pos{x, y, height};

                CellState realState = groundTruthMap.getCell(pos);
                CellState droneState = droneMap.getCell(pos);

                if (realState == droneState) {
                    ++correctCells;
                }
            }
        }
    }

    return (static_cast<double>(correctCells) / totalCells) * 100.0;
}