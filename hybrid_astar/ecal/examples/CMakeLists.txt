# eCAL Examples CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Find required packages
find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)

# Only eCAL-specific examples
set(EXAMPLES
    ecal_demo
)

# Create executables for each example
foreach(EXAMPLE ${EXAMPLES})
    set(EXAMPLE_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/${EXAMPLE}.cpp)
    
    if(EXISTS ${EXAMPLE_SOURCE})
        add_executable(${EXAMPLE} ${EXAMPLE_SOURCE})
        
        # Link required libraries
        if(TARGET hybrid_astar_core::hybrid_astar_core)
            target_link_libraries(${EXAMPLE}
                hybrid_astar_core::hybrid_astar_core
                hybrid_astar_visualization
                hybrid_astar_proto
                eCAL::core
                ${Protobuf_LIBRARIES}
            )
        else()
            # If built as subdirectory
            target_link_libraries(${EXAMPLE}
                hybrid_astar_core
                hybrid_astar_visualization
                hybrid_astar_proto
                eCAL::core
                ${Protobuf_LIBRARIES}
            )
        endif()
        
        # Include directories for workspace builds
        target_include_directories(${EXAMPLE} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../../hybrid_astar_core/include
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
        
        message(STATUS "Added eCAL example: ${EXAMPLE}")
    else()
        message(WARNING "Example source file not found: ${EXAMPLE_SOURCE}")
    endif()
endforeach()

# eCAL demo (with eCAL specific features)
if(TARGET ecal_demo)
    target_compile_definitions(ecal_demo PRIVATE ECAL_DEMO_VERSION)
endif()

# Create a custom target to build all examples
add_custom_target(build_all_ecal_examples
    DEPENDS ${EXAMPLES}
    COMMENT "Building all hybrid A* eCAL examples"
)

# Install example source files for reference
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION share/hybrid_astar_ecal/examples
    FILES_MATCHING PATTERN "*.cpp"
    PATTERN "CMakeLists.txt" EXCLUDE
)
