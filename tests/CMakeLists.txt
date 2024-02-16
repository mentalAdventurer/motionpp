if(BUILD_TESTING)
    find_package(Boost REQUIRED)
    add_executable(tests utils_test.cpp test_graph.cpp)

    target_link_libraries(tests PUBLIC GTest::gtest_main CellBasedSearch)


    include(GoogleTest)
    gtest_discover_tests(tests)
endif()
