cmake_minimum_required(VERSION 3.10)
project(2DMotionPrimitivesAccel)

set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME} main.cpp model.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC Motionpp matplot)

