# Defines the C++ project, the executable name, source files, and include folder used by CMake.

cmake_minimum_required(VERSION 3.10)
project(cpp_course)


add_executable(my_exe
    src/main.cpp
    src/GroundTruthMap.cpp
    src/SparseBuildingMap.cpp
    src/MockPositionSensor.cpp
    src/ConfigParser.cpp
    src/MapFileWriter.cpp
    src/ScoreCalculator.cpp
    src/simulator.cpp
    src/MockMovementDriver.cpp
    src/MockLidarSensor.cpp
)


target_include_directories(my_exe PRIVATE include)