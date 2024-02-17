cmake_minimum_required(VERSION 3.10)
project(MotionPlanningLibrary)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Posiiton independent code for shared libraries
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set Warning Level
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -pedantic)
endif()

# Enabel testing 
include(CTest)

find_package(Boost REQUIRED)

# Add subdirectories
add_subdirectory(external)
add_subdirectory(tests)
add_subdirectory(examples)

#incldue directories
include_directories(external/boost)

# Create Dynamic Library 
add_library(CellBasedSearch SHARED 
    src/gantry.cpp 
    src/simulator.cpp 
    src/cspace.cpp 
    src/queue.cpp
    src/graph.cpp) 

# Linking Libraries
target_include_directories(CellBasedSearch PUBLIC include)
#target_link_directories(simulator PUBLIC)
target_link_libraries(CellBasedSearch PUBLIC KDTree ${Boost_LIBRARIES}) 
