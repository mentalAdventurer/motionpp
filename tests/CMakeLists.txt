if(BUILD_TESTING)
    add_executable(tests test_simulator.cpp)
    target_link_libraries(tests PRIVATE GTest::gtest_main)
    target_link_libraries(tests PRIVATE simulator)

    include(GoogleTest)
    gtest_discover_tests(tests)
endif()
