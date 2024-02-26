cmake_minimum_required(VERSION 3.10)
project(ElasticLinearDrive)

set(CMAKE_CXX_STANDARD 20)

find_package(Matplot++ REQUIRED)

add_executable(ElasticLinearDrive test_model.cpp ElasticLinearDrive.cpp)
add_executable(ElasticLinearDriveSearch main.cpp ElasticLinearDrive.cpp)
target_link_libraries(ElasticLinearDrive CellBasedSearch Matplot++::matplot)
target_link_libraries(ElasticLinearDriveSearch CellBasedSearch Matplot++::matplot)

