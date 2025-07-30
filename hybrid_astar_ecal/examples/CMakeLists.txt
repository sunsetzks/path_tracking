# Examples CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Find required packages
find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)

# List of example executables and their source files
set(EXAMPLES
    demo
    ecal_demo
    integrated_planner_demo
    scene_update_visualization_example
    simple_demo
    timing_demo
)

# Create executables for each example
foreach(EXAMPLE ${EXAMPLES})
    set(EXAMPLE_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/${EXAMPLE}.cpp)
    
    if(EXISTS ${EXAMPLE_SOURCE})
        add_executable(${EXAMPLE} ${EXAMPLE_SOURCE})
        
        # Link required libraries
        target_link_libraries(${EXAMPLE}
            hybrid_astar_core
            hybrid_astar_visualization
            eCAL::core
            ${Protobuf_LIBRARIES}
        )
        
        # Set C++ standard
        set_target_properties(${EXAMPLE} PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
        )
        
        # Add to install targets
        install(TARGETS ${EXAMPLE}
            RUNTIME DESTINATION bin/examples
        )
        
        message(STATUS "Added example: ${EXAMPLE}")
    else()
        message(WARNING "Example source file not found: ${EXAMPLE_SOURCE}")
    endif()
endforeach()

# Special handling for examples that might need additional dependencies
# or different linking requirements

# Demo executable (basic example)
if(TARGET demo)
    target_compile_definitions(demo PRIVATE DEMO_VERSION)
endif()

# eCAL demo (with eCAL specific features)
if(TARGET ecal_demo)
    target_compile_definitions(ecal_demo PRIVATE ECAL_DEMO_VERSION)
endif()

# Integrated planner demo (might need additional libraries)
if(TARGET integrated_planner_demo)
    target_compile_definitions(integrated_planner_demo PRIVATE INTEGRATED_DEMO_VERSION)
endif()

# Scene update visualization example
if(TARGET scene_update_visualization_example)
    target_compile_definitions(scene_update_visualization_example PRIVATE SCENE_VIZ_VERSION)
endif()

# Simple demo (minimal dependencies)
if(TARGET simple_demo)
    target_compile_definitions(simple_demo PRIVATE SIMPLE_DEMO_VERSION)
endif()

# Timing demo (for performance testing)
if(TARGET timing_demo)
    target_compile_definitions(timing_demo PRIVATE TIMING_DEMO_VERSION)
endif()

# Create a custom target to build all examples
add_custom_target(build_all_examples
    DEPENDS ${EXAMPLES}
    COMMENT "Building all hybrid A* examples"
)

# Add option to build examples separately
option(BUILD_EXAMPLES "Build example executables" ON)

if(NOT BUILD_EXAMPLES)
    message(STATUS "Examples are disabled. Use -DBUILD_EXAMPLES=ON to enable.")
    # Remove all example targets if disabled
    foreach(EXAMPLE ${EXAMPLES})
        if(TARGET ${EXAMPLE})
            set_target_properties(${EXAMPLE} PROPERTIES EXCLUDE_FROM_ALL TRUE)
        endif()
    endforeach()
endif()

# Install example source files for reference
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION share/hybrid_astar_ecal/examples
    FILES_MATCHING PATTERN "*.cpp"
    PATTERN "CMakeLists.txt" EXCLUDE
)
