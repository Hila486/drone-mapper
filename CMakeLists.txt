# Defines the C++ project, the executable name, source files, and include folder used by CMake.

cmake_minimum_required(VERSION 3.10)
project(cpp_course)


add_executable(my_exe
    src/main.cpp
    src/GroundTruthMap.cpp
    src/SparseBuildingMap.cpp
    src/MockPositionSensor.cpp
)


target_include_directories(my_exe PRIVATE include)