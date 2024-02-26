if(BUILD_TESTING)
    add_executable(tests 
      test_graph.cpp 
      test_queue.cpp 
      test_ReachedSet.cpp
      test_voronoi.cpp)

    target_link_libraries(tests PUBLIC GTest::gtest_main CellBasedSearch)


    include(GoogleTest)
    gtest_discover_tests(tests)
endif()
