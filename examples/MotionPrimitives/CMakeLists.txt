cmake_minimum_required(VERSION 3.10)
project(MotionPrimitive)

set(CMAKE_CXX_STANDARD 20)

add_executable(MotionPrimitive main.cpp inputShaping.cpp model.cpp)
target_link_libraries(MotionPrimitive PUBLIC Motionpp matplot)

