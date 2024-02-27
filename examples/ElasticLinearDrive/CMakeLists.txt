cmake_minimum_required(VERSION 3.10)
project(ElasticLinearDrive)

set(CMAKE_CXX_STANDARD 20)

find_package(Matplot++ REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)

include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

add_executable(ElasticLinearDrive test_model.cpp ElasticLinearDrive.cpp)
add_executable(ElasticLinearDriveSearch main.cpp ElasticLinearDrive.cpp)
target_link_libraries(ElasticLinearDrive CellBasedSearch Matplot++::matplot ${Python3_LIBRARIES} ${Python3_NumPy_LIBRARIES})
target_link_libraries(ElasticLinearDriveSearch CellBasedSearch Matplot++::matplot ${Python3_LIBRARIES} ${Python3_NumPy_LIBRARIES})

