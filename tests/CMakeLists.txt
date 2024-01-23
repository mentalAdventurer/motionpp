if(BUILD_TESTING)
    find_package(Boost REQUIRED)
    add_executable(tests test_simulator.cpp test_gantry.cpp utils_test.cpp)

    target_link_libraries(tests PRIVATE GTest::gtest_main)
    target_link_libraries(tests PRIVATE simulator)
    target_link_libraries(tests PRIVATE ${Boost_LIBRARIES})

    include(GoogleTest)
    gtest_discover_tests(tests)
endif()
