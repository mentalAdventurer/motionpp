cmake_minimum_required(VERSION 3.10)
project(MotionPlanningLibrary)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find the GSL package
find_package(GSL REQUIRED)
find_package(Gtest REQUIRED)

# Include directories
include_directories(include)

# Specify the target library and set output name to libsimulator.so
add_library(simulator SHARED src/gantry.cpp src/simulator.cpp)
set_target_properties(simulator PROPERTIES OUTPUT_NAME "libsimulator")
