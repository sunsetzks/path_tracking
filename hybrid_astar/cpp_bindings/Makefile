# Makefile for Hybrid A* C++ project

# Configuration
BUILD_DIR = build
PYTHON_ENV = venv
CMAKE_BUILD_TYPE = Release

# Default target
.PHONY: all
all: cpp python

# Create virtual environment
.PHONY: venv
venv:
	@echo "Creating Python virtual environment..."
	python3 -m venv $(PYTHON_ENV)
	@echo "Activating and installing dependencies..."
	$(PYTHON_ENV)/bin/pip install --upgrade pip
	$(PYTHON_ENV)/bin/pip install -r requirements.txt

# Build C++ library and tests
.PHONY: cpp
cpp:
	@echo "Building C++ library and tests..."
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE)
	cd $(BUILD_DIR) && make -j$$(nproc)

# Build Python bindings
.PHONY: python
python: venv
	@echo "Building Python bindings..."
	$(PYTHON_ENV)/bin/pip install -e .

# Run C++ tests
.PHONY: test-cpp
test-cpp: cpp
	@echo "Running C++ tests..."
	cd $(BUILD_DIR) && ./hybrid_astar_test

# Run Python tests  
.PHONY: test-python
test-python: python
	@echo "Running Python tests..."
	$(PYTHON_ENV)/bin/python -m pytest tests/test_python_bindings.py -v

# Run all tests
.PHONY: test
test: test-cpp test-python

# Run C++ demo
.PHONY: demo-cpp
demo-cpp: cpp
	@echo "Running C++ demo..."
	cd $(BUILD_DIR) && ./hybrid_astar_example

# Run Python demo
.PHONY: demo-python
demo-python: python
	@echo "Running Python demo..."
	$(PYTHON_ENV)/bin/python examples/python_demo.py

# Run both demos
.PHONY: demo
demo: demo-cpp demo-python

# Benchmark performance
.PHONY: benchmark
benchmark: python
	@echo "Running performance benchmarks..."
	$(PYTHON_ENV)/bin/python -m pytest tests/test_python_bindings.py::TestPerformance -v --benchmark-only

# Clean build artifacts
.PHONY: clean
clean:
	@echo "Cleaning build artifacts..."
	rm -rf $(BUILD_DIR)
	rm -rf *.egg-info
	rm -rf dist
	find . -name "*.pyc" -delete
	find . -name "__pycache__" -delete

# Clean everything including virtual environment
.PHONY: distclean
distclean: clean
	@echo "Cleaning virtual environment..."
	rm -rf $(PYTHON_ENV)

# Install in development mode
.PHONY: install
install: python
	@echo "Package installed in development mode"

# Build distribution packages
.PHONY: dist
dist: python
	@echo "Building distribution packages..."
	$(PYTHON_ENV)/bin/python -m build

# Format code
.PHONY: format
format: venv
	@echo "Formatting Python code..."
	$(PYTHON_ENV)/bin/black examples/ tests/ setup.py
	$(PYTHON_ENV)/bin/isort examples/ tests/ setup.py

# Lint code
.PHONY: lint
lint: venv
	@echo "Linting Python code..."
	$(PYTHON_ENV)/bin/flake8 examples/ tests/ setup.py

# Show help
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  all          - Build both C++ and Python components"
	@echo "  cpp          - Build C++ library and tests"
	@echo "  python       - Build Python bindings"
	@echo "  venv         - Create Python virtual environment"
	@echo "  test         - Run all tests"
	@echo "  test-cpp     - Run C++ tests only"
	@echo "  test-python  - Run Python tests only"
	@echo "  demo         - Run both C++ and Python demos"
	@echo "  demo-cpp     - Run C++ demo only"
	@echo "  demo-python  - Run Python demo only"
	@echo "  benchmark    - Run performance benchmarks"
	@echo "  install      - Install in development mode"
	@echo "  dist         - Build distribution packages"
	@echo "  format       - Format Python code"
	@echo "  lint         - Lint Python code"
	@echo "  clean        - Clean build artifacts"
	@echo "  distclean    - Clean everything including venv"
	@echo "  help         - Show this help message"

# Quick start target
.PHONY: quickstart
quickstart: all test demo
	@echo ""
	@echo "✓ Quickstart completed successfully!"
	@echo ""
	@echo "To use the C++ library:"
	@echo "  Include: #include \"hybrid_astar.hpp\""
	@echo "  Link: build/libhybrid_astar_lib.a"
	@echo ""
	@echo "To use Python bindings:"
	@echo "  source venv/bin/activate"
	@echo "  python -c 'import hybrid_astar_cpp as ha'"
