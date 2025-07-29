#!/bin/bash

# Build script for Hybrid A* C++ project
# This script builds both the C++ library/tests and Python bindings

set -e  # Exit on any error

echo "Building Hybrid A* C++ Project"
echo "==============================="

# Get the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# Check for required tools
echo "Checking prerequisites..."

if ! command -v cmake &> /dev/null; then
    echo "Error: CMake is required but not installed."
    exit 1
fi

if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is required but not installed."
    exit 1
fi

# Create and activate virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv venv
fi

echo "Activating virtual environment..."
source venv/bin/activate

# Install Python dependencies
echo "Installing Python dependencies..."
pip install --upgrade pip
pip install pybind11[global] numpy pytest pytest-benchmark setuptools wheel build

# Build C++ library and tests
echo "Building C++ library and tests..."
mkdir -p build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo "Running C++ tests..."
if [ -f "./hybrid_astar_test" ]; then
    ./hybrid_astar_test
else
    echo "Warning: C++ test executable not found"
fi

cd ..

# Build Python bindings
echo "Building Python bindings..."
pip install -e .

# Test Python bindings
echo "Testing Python bindings..."
python -c "
try:
    import hybrid_astar_cpp as ha
    print('✓ Python bindings imported successfully')
    
    # Quick test
    vehicle = ha.VehicleModel(2.5, 3.14159/4)
    print(f'✓ VehicleModel created: wheelbase={vehicle.wheelbase}, max_steer={vehicle.max_steer}')
    
    state = ha.State(1.0, 2.0, 0.5, ha.DirectionMode.FORWARD, 0.1)
    print(f'✓ State created: x={state.x}, y={state.y}, yaw={state.yaw}')
    
    planner = ha.HybridAStar(vehicle)
    print('✓ HybridAStar planner created')
    
    print('✓ All basic functionality tests passed')
    
except Exception as e:
    print(f'✗ Error testing Python bindings: {e}')
    exit(1)
"

# Run Python tests if pytest is available
echo "Running Python tests..."
if python -c "import pytest" 2>/dev/null; then
    python -m pytest tests/test_python_bindings.py -v || echo "Some Python tests failed"
else
    echo "pytest not available, skipping detailed Python tests"
fi

# Run demo if built
echo "Running C++ demo..."
cd build
if [ -f "./hybrid_astar_example" ]; then
    echo "Running C++ demo application..."
    ./hybrid_astar_example
else
    echo "Warning: C++ demo executable not found"
fi

cd ..

echo ""
echo "Build completed successfully!"
echo ""
echo "Usage:"
echo "  C++ library: build/libhybrid_astar_lib.a"
echo "  C++ tests:   build/hybrid_astar_test"
echo "  C++ demo:    build/hybrid_astar_example"
echo "  Python:      import hybrid_astar_cpp"
echo ""
echo "To use Python bindings:"
echo "  source venv/bin/activate"
echo "  python -c 'import hybrid_astar_cpp as ha'"
