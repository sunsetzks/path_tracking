#!/bin/bash

# Test build script for the new CMake structure
set -e

echo "Testing new CMake structure with separate proto and examples subdirectories..."

# Clean previous build
if [ -d "build" ]; then
    echo "Cleaning previous build..."
    rm -rf build
fi

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo "Configuring CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON

# Build the project
echo "Building project..."
make -j$(nproc)

# List generated executables
echo ""
echo "Generated executables:"
find . -name "demo" -o -name "ecal_demo" -o -name "*_demo" -o -name "*example*" | sort

echo ""
echo "Generated libraries:"
find . -name "*.so" -o -name "*.a" | sort

echo ""
echo "Proto generated files:"
find . -name "*.pb.h" -o -name "*.pb.cc" | sort

echo ""
echo "Build completed successfully!"
