#!/bin/bash

# Build script for hybrid_astar_core

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Building Hybrid A* Core...${NC}"

# Create build directory
BUILD_DIR="build"
if [ -d "$BUILD_DIR" ]; then
    echo -e "${YELLOW}Removing existing build directory...${NC}"
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure
echo -e "${GREEN}Configuring CMake...${NC}"
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_TESTS=ON

# Build
echo -e "${GREEN}Building...${NC}"
make -j$(nproc)

# Test (if requested)
if [ "$1" = "test" ]; then
    echo -e "${GREEN}Running tests...${NC}"
    ctest --verbose
fi

echo -e "${GREEN}Build completed successfully!${NC}"
echo "Executables are in: $(pwd)"
echo "Examples are in: $(pwd)/examples/"
