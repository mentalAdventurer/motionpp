cmake_minimum_required(VERSION 3.10)
project(MotionPrimitive)

set(CMAKE_CXX_STANDARD 20)

find_package(Matplot++ REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)

include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

add_executable(MotionPrimitive main.cpp inputShaping.cpp model.cpp)
target_link_libraries(MotionPrimitive CellBasedSearch Matplot++::matplot ${Python3_LIBRARIES} ${Python3_NumPy_LIBRARIES})

