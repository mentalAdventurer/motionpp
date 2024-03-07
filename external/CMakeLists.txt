set(CMAKE_CXX_STANDARD 17)
add_subdirectory(KDTree)

if (MOTIONPP_BUILD_EXAMPLES)
add_subdirectory(matplotplusplus)
endif()

if (MOTIONPP_BUILD_TESTS)
add_subdirectory(googletest)
endif()
