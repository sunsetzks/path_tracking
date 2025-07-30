# CMakeLists.txt for tests directory
cmake_minimum_required(VERSION 3.14)

# Include parent directory to find the library
get_filename_component(TESTS_PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

# Find the hybrid_astar library
if(NOT TARGET hybrid_astar_lib)
    # If we're building as part of the main project, the library should already be defined
    # Otherwise, we need to find it as a package
    find_package(HybridAStar REQUIRED)
endif()

# Create test executable
add_executable(hybrid_astar_test test_main.cpp)

# Link with the hybrid_astar library
if(TARGET hybrid_astar_lib)
    target_link_libraries(hybrid_astar_test hybrid_astar_lib)
else()
    target_link_libraries(hybrid_astar_test HybridAStar::hybrid_astar_lib)
endif()

# Include directories
target_include_directories(hybrid_astar_test PRIVATE
    ${TESTS_PARENT_DIR}/include
)

# Add test definitions
target_compile_definitions(hybrid_astar_test PRIVATE
    TEST_BUILD
)

# Testing
enable_testing()

# Add test for basic functionality
add_test(NAME BasicFunctionality COMMAND hybrid_astar_test)

# Add test for obstacle avoidance
add_test(NAME ObstacleAvoidance COMMAND hybrid_astar_test --test=obstacle)

# Add test for performance
add_test(NAME PerformanceTest COMMAND hybrid_astar_test --test=performance)

# Configure test output
set_tests_properties(BasicFunctionality ObstacleAvoidance PerformanceTest PROPERTIES
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Custom target for running tests
add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --verbose
    COMMENT "Running all tests"
)

# Print configuration message
message(STATUS "Tests configured successfully:")
message(STATUS "  Test executable: hybrid_astar_test")
message(STATUS "  Available tests: BasicFunctionality, ObstacleAvoidance, PerformanceTest")