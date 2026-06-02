cmake_minimum_required(VERSION 3.14)
project(hybrid_astar_cpp VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set compile options
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -O0)
    add_definitions(-DDEBUG)
else()
    add_compile_options(-O3 -DNDEBUG)
endif()

# Enable warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(pybind11 REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Source files
set(SOURCES
    src/hybrid_astar.cpp
)

# Create a library
add_library(hybrid_astar_lib STATIC ${SOURCES})
target_include_directories(hybrid_astar_lib PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Create Python module using pybind11
pybind11_add_module(hybrid_astar_cpp python/pybind_module.cpp ${SOURCES})

# Set properties for the Python module
target_compile_definitions(hybrid_astar_cpp PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
target_include_directories(hybrid_astar_cpp PRIVATE include)

# Create standalone executable for testing
add_executable(hybrid_astar_test tests/test_main.cpp ${SOURCES})
target_include_directories(hybrid_astar_test PRIVATE include)

# Enable testing
enable_testing()

# Add the test executable as a test
add_test(NAME hybrid_astar_unit_tests COMMAND hybrid_astar_test)

# Installation rules
install(TARGETS hybrid_astar_lib
    EXPORT hybrid_astar_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include
)

install(EXPORT hybrid_astar_targets
    FILE hybrid_astar_targets.cmake
    DESTINATION lib/cmake/hybrid_astar
)

# Create example executable
add_executable(hybrid_astar_example examples/demo.cpp ${SOURCES})
target_include_directories(hybrid_astar_example PRIVATE include)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
