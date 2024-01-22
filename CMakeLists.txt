cmake_minimum_required(VERSION 3.10)
project(MotionPlanningLibrary)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Enabel testing 
include(CTest)

# Include directories
include_directories(include)

# Add subdirectories
add_subdirectory(external)
add_subdirectory(tests)

# Find the GSL package
find_package(GSL REQUIRED)

# Specify the target library and set output name to libsimulator.so
add_library(simulator SHARED src/gantry.cpp src/simulator.cpp)
set_target_properties(simulator PROPERTIES OUTPUT_NAME "simulator")

# linke the GSL library
target_link_libraries(simulator PRIVATE GSL::gsl GSL::gslcblas)
