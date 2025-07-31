# Proto files CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

find_package(Protobuf REQUIRED)

# Set output directory for generated files
set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${PROTO_GEN_DIR}/foxglove)

# Find all proto files
file(GLOB_RECURSE ALL_PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

# Generate protobuf files
set(PROTO_SRCS)
set(PROTO_HDRS)

foreach(PROTO_FILE ${ALL_PROTO_FILES})
    # Get relative path to preserve directory structure
    file(RELATIVE_PATH PROTO_REL_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${PROTO_FILE})
    get_filename_component(PROTO_DIR ${PROTO_REL_PATH} DIRECTORY)
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    
    # Set output paths
    if(PROTO_DIR)
        set(PROTO_SRC "${PROTO_GEN_DIR}/${PROTO_DIR}/${PROTO_NAME}.pb.cc")
        set(PROTO_HDR "${PROTO_GEN_DIR}/${PROTO_DIR}/${PROTO_NAME}.pb.h")
    else()
        set(PROTO_SRC "${PROTO_GEN_DIR}/${PROTO_NAME}.pb.cc")
        set(PROTO_HDR "${PROTO_GEN_DIR}/${PROTO_NAME}.pb.h")
    endif()
    
    add_custom_command(
        OUTPUT ${PROTO_SRC} ${PROTO_HDR}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        ARGS --cpp_out=${PROTO_GEN_DIR} --proto_path=${CMAKE_CURRENT_SOURCE_DIR} ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
        COMMENT "Generating protobuf for ${PROTO_REL_PATH}"
        VERBATIM
    )
    
    list(APPEND PROTO_SRCS ${PROTO_SRC})
    list(APPEND PROTO_HDRS ${PROTO_HDR})
endforeach()

# Create library and configure
add_library(hybrid_astar_proto STATIC ${PROTO_SRCS} ${PROTO_HDRS})

target_include_directories(hybrid_astar_proto PUBLIC 
    $<BUILD_INTERFACE:${PROTO_GEN_DIR}>
    $<INSTALL_INTERFACE:include/hybrid_astar_ecal/proto>
)
target_link_libraries(hybrid_astar_proto PUBLIC protobuf::libprotobuf)
set_target_properties(hybrid_astar_proto PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Export variables to parent scope
set(PROTO_GENERATED_SOURCES ${PROTO_SRCS} PARENT_SCOPE)
set(PROTO_GENERATED_HEADERS ${PROTO_HDRS} PARENT_SCOPE)
set(PROTO_GENERATED_DIR ${PROTO_GEN_DIR} PARENT_SCOPE)

# Install targets
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION share/hybrid_astar_ecal/proto
    FILES_MATCHING PATTERN "*.proto"
)
install(FILES ${PROTO_GENERATED_HEADERS}
    DESTINATION include/hybrid_astar_ecal/proto
)
