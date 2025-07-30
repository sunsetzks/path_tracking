cmake_minimum_required(VERSION 3.16)
project(hybrid_astar_ecal VERSION 1.0.0 LANGUAGES CXX)

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

# Try to find eCAL and Protobuf (optional for development)
find_package(eCAL QUIET)
find_package(Protobuf QUIET)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Proto files directory
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto)

# Core library sources
set(CORE_SOURCES
    src/common_types.cpp
    src/vehicle_model.cpp
    src/obstacle_map.cpp
    src/hybrid_astar.cpp
)

# Create core library
add_library(hybrid_astar_core ${CORE_SOURCES})
target_link_libraries(hybrid_astar_core Threads::Threads)

# If eCAL and Protobuf are available, build full version
if(eCAL_FOUND AND Protobuf_FOUND)
    message(STATUS "Building with eCAL and Protobuf support")
    
    # Define macro for conditional compilation
    add_compile_definitions(ECAL_PROTOBUF_AVAILABLE)
    
    # Generate protobuf sources
    file(GLOB_RECURSE PROTO_FILES "${PROTO_DIR}/*.proto")
    
    # Create generated directory
    set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
    file(MAKE_DIRECTORY ${GENERATED_DIR})
    
    # Generate C++ files from proto files
    set(PROTO_GENERATED_SOURCES "")
    foreach(PROTO_FILE ${PROTO_FILES})
        get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
        get_filename_component(PROTO_DIR_PATH ${PROTO_FILE} DIRECTORY)
        
        # Calculate relative path from proto directory
        file(RELATIVE_PATH PROTO_REL_PATH ${PROTO_DIR} ${PROTO_DIR_PATH})
        
        # Output files
        set(PROTO_CC ${GENERATED_DIR}/${PROTO_REL_PATH}/${PROTO_NAME}.pb.cc)
        set(PROTO_H ${GENERATED_DIR}/${PROTO_REL_PATH}/${PROTO_NAME}.pb.h)
        
        # Generate protobuf files
        add_custom_command(
            OUTPUT ${PROTO_CC} ${PROTO_H}
            COMMAND ${Protobuf_PROTOC_EXECUTABLE}
            ARGS --cpp_out=${GENERATED_DIR}
                 --proto_path=${PROTO_DIR}
                 ${PROTO_FILE}
            DEPENDS ${PROTO_FILE}
            COMMENT "Generating protobuf files for ${PROTO_FILE}"
        )
        
        list(APPEND PROTO_GENERATED_SOURCES ${PROTO_CC})
    endforeach()
    
    # Include generated directory
    include_directories(${GENERATED_DIR})
    
    # Visualization library with eCAL
    add_library(hybrid_astar_visualization 
        src/visualization_publisher.cpp
        ${PROTO_GENERATED_SOURCES}
    )
    
    target_link_libraries(hybrid_astar_visualization
        hybrid_astar_core
        eCAL::core
        ${Protobuf_LIBRARIES}
    )
    
    # Demo with eCAL
    add_executable(hybrid_astar_ecal_demo
        examples/ecal_demo.cpp
    )
    
    target_link_libraries(hybrid_astar_ecal_demo
        hybrid_astar_core
        hybrid_astar_visualization
        eCAL::core
        ${Protobuf_LIBRARIES}
    )
    
    # Add demo executable with visualization support
    add_executable(hybrid_astar_demo_with_viz
        examples/demo.cpp
    )
    
    target_link_libraries(hybrid_astar_demo_with_viz
        hybrid_astar_core
        hybrid_astar_visualization
        eCAL::core
        ${Protobuf_LIBRARIES}
    )
    
    # Install targets
    install(TARGETS hybrid_astar_core hybrid_astar_visualization hybrid_astar_ecal_demo hybrid_astar_demo_with_viz
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
    
else()
    message(STATUS "Building standalone version (eCAL or Protobuf not found)")
    
    # Simple demo executable (standalone)
    add_executable(hybrid_astar_simple_demo
        examples/simple_demo.cpp
    )
    
    target_link_libraries(hybrid_astar_simple_demo
        hybrid_astar_core
    )
    
    # Conditional demo executable (compatible with both modes)
    add_executable(hybrid_astar_demo
        examples/demo.cpp
    )
    
    target_link_libraries(hybrid_astar_demo
        hybrid_astar_core
    )
    
    # Timing performance demo
    add_executable(hybrid_astar_timing_demo
        examples/timing_demo.cpp
    )
    
    target_link_libraries(hybrid_astar_timing_demo
        hybrid_astar_core
    )
    
    # Install targets
    install(TARGETS hybrid_astar_core hybrid_astar_simple_demo hybrid_astar_demo hybrid_astar_timing_demo
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
endif()

# Install headers
install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

# Install proto files
install(DIRECTORY proto/
    DESTINATION share/hybrid_astar_ecal/proto
    FILES_MATCHING PATTERN "*.proto"
)
