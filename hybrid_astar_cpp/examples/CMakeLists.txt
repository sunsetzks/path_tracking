# CMakeLists.txt for examples directory
cmake_minimum_required(VERSION 3.14)

# Include parent directory to find the library
get_filename_component(EXAMPLES_PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

# Find the hybrid_astar library
if(NOT TARGET hybrid_astar_lib)
    # If we're building as part of the main project, the library should already be defined
    # Otherwise, we need to find it as a package
    find_package(HybridAStar REQUIRED)
endif()

# Basic demo executable
add_executable(demo demo.cpp)

# Link with the hybrid_astar library
if(TARGET hybrid_astar_lib)
    target_link_libraries(demo hybrid_astar_lib)
else()
    target_link_libraries(demo HybridAStar::hybrid_astar_lib)
endif()

# Include directories
target_include_directories(demo PRIVATE
    ${EXAMPLES_PARENT_DIR}/include
)

# Advanced demo executable
add_executable(advanced_demo advanced_demo.cpp)

# Link with the hybrid_astar library
if(TARGET hybrid_astar_lib)
    target_link_libraries(advanced_demo hybrid_astar_lib)
else()
    target_link_libraries(advanced_demo HybridAStar::hybrid_astar_lib)
endif()

# Include directories
target_include_directories(advanced_demo PRIVATE
    ${EXAMPLES_PARENT_DIR}/include
)

# Python demo (if Python is available)
find_package(Python COMPONENTS Interpreter REQUIRED)

if(Python_FOUND)
    # Create a custom target for the Python demo instead of an executable
    add_custom_target(python_demo
        COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/python_demo.py
        COMMENT "Running Python demo"
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
endif()

# Installation targets
install(TARGETS demo advanced_demo
    RUNTIME DESTINATION examples
)

# Install Python demo script if Python is found
if(Python_FOUND)
    install(FILES python_demo.py
        DESTINATION examples
    )
endif()

# Custom targets for running examples
add_custom_target(run_demo
    COMMAND demo
    COMMENT "Running basic demo"
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

add_custom_target(run_advanced_demo
    COMMAND advanced_demo
    COMMENT "Running advanced demo"
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

if(Python_FOUND)
    add_custom_target(run_python_demo
        COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/python_demo.py
        COMMENT "Running Python demo"
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
endif()

# Print configuration message
message(STATUS "Examples configured successfully:")
message(STATUS "  Basic demo: demo")
message(STATUS "  Advanced demo: advanced_demo")
if(Python_FOUND)
    message(STATUS "  Python demo: python_demo")
else()
    message(STATUS "  Python demo: skipped (Python not found)")
endif()