#!/bin/bash

# Build script for Hybrid A* eCAL project

set -e

PROJECT_DIR=$(dirname "$(realpath "$0")")
BUILD_DIR="$PROJECT_DIR/build"

echo "=== Building Hybrid A* eCAL Project ==="
echo "Project directory: $PROJECT_DIR"
echo "Build directory: $BUILD_DIR"

# Create build directory
if [ ! -d "$BUILD_DIR" ]; then
    echo "Creating build directory..."
    mkdir -p "$BUILD_DIR"
fi

# Navigate to build directory
cd "$BUILD_DIR"

# Run CMake
echo "Running CMake..."
cmake ..

# Build the project
echo "Building project..."
make -j$(nproc)

echo "Build completed successfully!"
echo ""
echo "Executables:"
echo "  $BUILD_DIR/hybrid_astar_demo"
echo ""
echo "To run the demo:"
echo "  cd $BUILD_DIR && ./hybrid_astar_demo"
