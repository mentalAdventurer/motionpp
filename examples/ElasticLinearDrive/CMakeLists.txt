cmake_minimum_required(VERSION 3.10)
project(ElasticLinearDrive)

set(CMAKE_CXX_STANDARD 20)

add_executable(ElasticLinearDriveSearch main.cpp ElasticLinearDrive.cpp)
target_link_libraries(ElasticLinearDriveSearch Motionpp matplot)

