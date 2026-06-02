#!/bin/bash

# Hybrid A* eCAL Integration Setup Script
# This script helps set up the development environment

set -e

echo "🚀 Hybrid A* eCAL Integration Setup"
echo "=================================="

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check system
echo "📋 Checking system requirements..."

# Check CMake
if command_exists cmake; then
    CMAKE_VERSION=$(cmake --version | head -n1 | grep -o '[0-9]\+\.[0-9]\+')
    echo "✅ CMake $CMAKE_VERSION found"
else
    echo "❌ CMake not found. Please install CMake 3.16+"
    exit 1
fi

# Check C++ compiler
if command_exists g++; then
    GCC_VERSION=$(g++ --version | head -n1 | grep -o '[0-9]\+\.[0-9]\+')
    echo "✅ GCC $GCC_VERSION found"
elif command_exists clang++; then
    CLANG_VERSION=$(clang++ --version | head -n1 | grep -o '[0-9]\+\.[0-9]\+')
    echo "✅ Clang $CLANG_VERSION found"
else
    echo "❌ No C++ compiler found. Please install g++ or clang++"
    exit 1
fi

# Check for eCAL
ECAL_AVAILABLE=false
if command_exists ecal_info || [ -f "/usr/local/lib/cmake/eCAL/eCALConfig.cmake" ] || [ -f "/usr/lib/cmake/eCAL/eCALConfig.cmake" ]; then
    echo "✅ eCAL found"
    ECAL_AVAILABLE=true
else
    echo "⚠️  eCAL not found (will build basic version)"
fi

# Check for protobuf
PROTOBUF_AVAILABLE=false
if command_exists protoc; then
    PROTOC_VERSION=$(protoc --version | grep -o '[0-9]\+\.[0-9]\+')
    echo "✅ Protocol Buffers $PROTOC_VERSION found"
    PROTOBUF_AVAILABLE=true
else
    echo "⚠️  Protocol Buffers not found (will build basic version)"
fi

echo ""
echo "🔧 Building project..."

# Create build directory
mkdir -p build
cd build

# Configure build based on available dependencies
if [ "$ECAL_AVAILABLE" = true ] && [ "$PROTOBUF_AVAILABLE" = true ]; then
    echo "🎯 Building with full eCAL and Protobuf support"
    cmake ..
else
    echo "🎯 Building basic version (no eCAL/Protobuf)"
    cmake -DECAL_FOUND=OFF -DProtobuf_FOUND=OFF ..
fi

# Build
echo "⚙️  Compiling..."
make -j$(nproc)

echo ""
echo "✅ Build completed successfully!"
echo ""

# Check what was built
if [ -f "hybrid_astar_demo" ]; then
    echo "📦 Available executables:"
    echo "   ./hybrid_astar_demo          - Basic demo"
fi

if [ -f "hybrid_astar_ecal_demo" ]; then
    echo "   ./hybrid_astar_ecal_demo     - eCAL integrated demo"
fi

echo ""
echo "🧪 Running basic test..."
if [ -f "hybrid_astar_demo" ]; then
    echo "Testing basic demo..."
    timeout 10s ./hybrid_astar_demo || echo "Demo completed or timeout"
elif [ -f "hybrid_astar_ecal_demo" ]; then
    echo "Testing eCAL demo..."
    timeout 10s ./hybrid_astar_ecal_demo || echo "Demo completed or timeout"
fi

echo ""
echo "🎉 Setup complete!"
echo ""
echo "📖 Usage:"
echo "   cd build"
if [ -f "hybrid_astar_demo" ]; then
    echo "   ./hybrid_astar_demo          # Run basic demo"
fi
if [ -f "hybrid_astar_ecal_demo" ]; then
    echo "   ./hybrid_astar_ecal_demo     # Run eCAL demo"
fi

echo ""
echo "📚 For more information, see README.md"

if [ "$ECAL_AVAILABLE" = false ]; then
    echo ""
    echo "💡 To enable eCAL support:"
    echo "   sudo apt update && sudo apt install ecal-dev"
    echo "   Then re-run this script"
fi

if [ "$PROTOBUF_AVAILABLE" = false ]; then
    echo ""
    echo "💡 To enable Protobuf support:"
    echo "   sudo apt update && sudo apt install protobuf-compiler libprotobuf-dev"
    echo "   Then re-run this script"
fi
