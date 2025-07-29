"""
Setup script for hybrid_astar_cpp Python bindings
"""

from pybind11.setup_helpers import Pybind11Extension, build_ext
from pybind11 import get_cmake_dir
import pybind11
from setuptools import setup, Extension
import os

# Define the extension module
ext_modules = [
    Pybind11Extension(
        "hybrid_astar_cpp",
        [
            "python/pybind_module.cpp",
            "src/hybrid_astar.cpp",
        ],
        include_dirs=[
            "include",
            pybind11.get_include(),
        ],
        language='c++',
        cxx_std=17,
    ),
]

setup(
    name="hybrid_astar_cpp",
    version="1.0.0",
    author="Path Tracking Team",
    author_email="your.email@example.com",
    description="Hybrid A* Path Planning Algorithm - C++ implementation with Python bindings",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
    install_requires=[
        "numpy>=1.20.0",
        "pybind11>=2.6.0",
    ],
    extras_require={
        "test": [
            "pytest>=6.0",
            "pytest-benchmark",
            "matplotlib",
        ],
        "examples": [
            "matplotlib",
            "numpy",
        ],
    },
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: C++",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
)
