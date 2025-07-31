# Tests CMakeLists.txt

# Find required packages for testing
find_package(GTest QUIET)

if(GTest_FOUND)
    enable_testing()
    
    # Test sources
    set(TEST_SOURCES
        test_common_types.cpp
    )
    
    # Create test executable
    add_executable(hybrid_astar_core_tests ${TEST_SOURCES})
    target_link_libraries(hybrid_astar_core_tests 
        hybrid_astar_core
        GTest::GTest 
        GTest::Main
    )
    
    # Add tests to CTest
    include(GoogleTest)
    gtest_discover_tests(hybrid_astar_core_tests)
    
    message(STATUS "Google Test found - building tests")
else()
    message(WARNING "Google Test not found - skipping tests")
endif()
