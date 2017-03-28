project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/FusionEKF.cpp
        src/main.cpp
    src/tools.cpp src/MeasurementUnit.cpp src/MeasurementUnit.h src/RadarMeasurementUnit.cpp src/RadarMeasurementUnit.h src/LinearMeasurementUnit.cpp src/LinearMeasurementUnit.h)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")
add_executable(ExtendedKF ${sources})
