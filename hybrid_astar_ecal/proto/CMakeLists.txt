# Proto files CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Use vcpkg's Protobuf package to avoid system/global version conflicts
find_package(Protobuf REQUIRED)

# Set output directory for generated files
set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)

# Ensure the output directory exists
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

# Set the path to foxglove proto files
set(FOXGLOVE_PROTO_PATH ${CMAKE_CURRENT_SOURCE_DIR}/foxglove)

# Find all proto files
file(GLOB_RECURSE PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

# Generate protobuf files
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
    ${PROTO_FILES}
    PROTOC_OUT_DIR ${PROTO_GEN_DIR}
    IMPORT_DIRS ${FOXGLOVE_PROTO_PATH}
)

# Create a library for all generated proto files
add_library(hybrid_astar_proto STATIC
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

target_include_directories(hybrid_astar_proto
    PUBLIC
        ${PROTO_GEN_DIR}
)

target_link_libraries(hybrid_astar_proto
    PUBLIC
        protobuf::libprotobuf
)

# Set properties for the proto library
set_target_properties(hybrid_astar_proto PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Export the variables to parent scope
set(PROTO_GENERATED_SOURCES ${PROTO_SRCS} PARENT_SCOPE)
set(PROTO_GENERATED_HEADERS ${PROTO_HDRS} PARENT_SCOPE)
set(PROTO_GENERATED_DIR ${PROTO_GEN_DIR} PARENT_SCOPE)

# Install proto files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION share/hybrid_astar_ecal/proto
    FILES_MATCHING PATTERN "*.proto"
)

# Install generated headers
install(FILES ${PROTO_GENERATED_HEADERS}
    DESTINATION include/hybrid_astar_ecal/proto
)
