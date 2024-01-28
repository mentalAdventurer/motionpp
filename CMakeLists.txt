cmake_minimum_required(VERSION 3.10)
project(MotionPlanningLibrary)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Posiiton independent code for shared libraries
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Enabel testing 
include(CTest)

find_package(Boost REQUIRED)

# Add subdirectories
add_subdirectory(external)
#add_subdirectory(tests)

#incldue directories
include_directories(external/boost)

# Create Dynamic Library 
add_library(simulator SHARED src/gantry.cpp src/simulator.cpp src/cspace.cpp) 

# Linking Libraries
target_include_directories(simulator PUBLIC include)
#target_link_directories(simulator PUBLIC)
target_link_libraries(simulator PUBLIC KDTree ${Boost_LIBRARIES}) 
