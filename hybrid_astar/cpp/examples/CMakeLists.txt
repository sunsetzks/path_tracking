# Examples CMakeLists.txt

# Basic demo example
add_executable(basic_demo demo.cpp)
target_link_libraries(basic_demo hybrid_astar_core)

# Collision detector example
add_executable(collision_detector_example collision_detector_example.cpp)
target_link_libraries(collision_detector_example hybrid_astar_core)

# Install examples (optional)
install(TARGETS basic_demo collision_detector_example
    RUNTIME DESTINATION bin/examples
)
