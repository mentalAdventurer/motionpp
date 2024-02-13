cmake_minimum_required(VERSION 3.10)
project(ElasticLinearDrive)

set(CMAKE_CXX_STANDARD 23)

add_executable(ElasticLinearDrive main.cpp ElasticLinearDrive.cpp)
target_link_libraries(ElasticLinearDrive CellBasedSearch)

