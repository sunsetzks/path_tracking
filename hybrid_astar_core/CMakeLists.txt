cmake_minimum_required(VERSION 3.16)
project(hybrid_astar_core VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Find required packages
find_package(Threads REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Core library sources
set(CORE_SOURCES
    src/common_types.cpp
    src/vehicle_model.cpp
    src/obstacle_map.cpp
    src/collision_detector.cpp
    src/hybrid_astar.cpp
)

# Create core library
add_library(hybrid_astar_core ${CORE_SOURCES})
target_link_libraries(hybrid_astar_core Threads::Threads)

# Set target properties
target_include_directories(hybrid_astar_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Compiler options
target_compile_options(hybrid_astar_core PRIVATE
    -Wall -Wextra -Wpedantic
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Debug>:-g -O0>
)

# Add examples subdirectory (optional)
option(BUILD_EXAMPLES "Build examples" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Add tests subdirectory (optional)
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Install targets
install(TARGETS hybrid_astar_core
    EXPORT hybrid_astar_core_targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

# Install cmake config files
install(EXPORT hybrid_astar_core_targets
    FILE hybrid_astar_core_targets.cmake
    NAMESPACE hybrid_astar_core::
    DESTINATION lib/cmake/hybrid_astar_core
)

# Generate config file
include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/hybrid_astar_core_config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/hybrid_astar_core_config.cmake"
    INSTALL_DESTINATION lib/cmake/hybrid_astar_core
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/hybrid_astar_core_config_version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/hybrid_astar_core_config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/hybrid_astar_core_config_version.cmake"
    DESTINATION lib/cmake/hybrid_astar_core
)
