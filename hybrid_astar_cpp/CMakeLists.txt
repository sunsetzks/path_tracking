cmake_minimum_required(VERSION 3.14)
project(HybridAStar VERSION 1.0.0 LANGUAGES CXX)

# -----------------------------------------------------------------------------
# Project Configuration
# -----------------------------------------------------------------------------

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Generate compile_commands.json for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler-specific options
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------

# Find required packages
find_package(Threads REQUIRED)

# -----------------------------------------------------------------------------
# Library Configuration
# -----------------------------------------------------------------------------

# Add include directories
include_directories(include)

# Create library
add_library(hybrid_astar_lib STATIC
    src/hybrid_astar.cpp
)

# Set library properties
set_target_properties(hybrid_astar_lib PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
    PUBLIC_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar.hpp"
)

# Add library include directories
target_include_directories(hybrid_astar_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link library with threads
target_link_libraries(hybrid_astar_lib Threads::Threads)

# Add subdirectories
add_subdirectory(examples)
add_subdirectory(tests)

# -----------------------------------------------------------------------------
# Installation Configuration
# -----------------------------------------------------------------------------

# Install library targets
install(TARGETS hybrid_astar_lib
    EXPORT HybridAStarTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include
)

# Install examples
install(DIRECTORY examples/ DESTINATION examples
    PATTERN "CMakeLists.txt" EXCLUDE
    PATTERN "*.cmake" EXCLUDE
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Create export set
install(EXPORT HybridAStarTargets
    FILE HybridAStarTargets.cmake
    NAMESPACE HybridAStar::
    DESTINATION lib/cmake/HybridAStar
)

# Create config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    HybridAStarConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/HybridAStarConfigVersion.cmake
    DESTINATION lib/cmake/HybridAStar
)

# Create package configuration file
configure_package_config_file(
    HybridAStarConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/HybridAStarConfig.cmake
    INSTALL_DESTINATION lib/cmake/HybridAStar
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/HybridAStarConfig.cmake
    DESTINATION lib/cmake/HybridAStar
)

# Add custom targets
# Note: run_tests target is defined in tests/CMakeLists.txt to avoid conflicts



# Create a symlink to compile_commands.json in the source directory
add_custom_target(SymlinkCompileCommands
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${CMAKE_SOURCE_DIR}/compile_commands.json
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Explicitly generate compile_commands.json
add_custom_target(GenerateCompileCommands
    COMMAND ${CMAKE_COMMAND} -E echo "${CMAKE_EXPORT_COMPILE_COMMANDS}"
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/cmake/generate_compile_commands.cmake
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Add generate_compile_commands as a dependency to SymlinkCompileCommands
add_dependencies(SymlinkCompileCommands GenerateCompileCommands)

# Print configuration
message(STATUS "Hybrid A* C++ Project Configuration:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  Compile commands: ${CMAKE_BINARY_DIR}/compile_commands.json")