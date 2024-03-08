cmake_minimum_required(VERSION 3.10)
project(Motionpp)

# Options
option(MOTIONPP_BUILD_EXAMPLES "Build MotionPP examples" OFF)
option(MOTIONPP_BUILD_TESTS "Build MotionPP tests" OFF)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Posiiton independent code for shared libraries
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set Warning Level
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wno-unused-parameter -Wextra -pedantic)
endif()

# Enabel testing 
include(CTest)

# Add subdirectories
add_subdirectory(external)

if (MOTIONPP_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if (MOTIONPP_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Create Dynamic Library 
add_library(${PROJECT_NAME} STATIC 
    src/cell_based_search.cpp
    src/cspace.cpp 
    src/queue.cpp
    src/graph.cpp) 

# Linking Libraries
target_include_directories(${PROJECT_NAME} PUBLIC include)
#target_link_directories(simulator PUBLIC)
target_link_libraries(${PROJECT_NAME} PUBLIC KDTree) 
