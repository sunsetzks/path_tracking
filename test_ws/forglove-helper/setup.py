#!/usr/bin/env python3
"""
Setup script for forglove-helper with automatic proto compilation.

This script automatically compiles protocol buffer files during the build process.
"""

import subprocess
import sys
from pathlib import Path
from setuptools import setup, find_packages
from setuptools.command.build_py import build_py


class BuildWithProtos(build_py):
    """Custom build command that compiles proto files before building."""

    def run(self):
        """Run proto compilation and then standard build."""
        print("Building forglove-helper with proto compilation...")

        # Compile proto files
        if not self._compile_protos():
            print("Proto compilation failed. Aborting build.")
            sys.exit(1)

        # Run standard build
        build_py.run(self)

    def _compile_protos(self) -> bool:
        """Compile protocol buffer files."""
        try:
            script_path = Path(__file__).parent / "scripts" / "compile_protos.py"
            if not script_path.exists():
                print(f"Warning: Proto compilation script not found at {script_path}")
                return True  # Continue build even if script is missing

            print("Compiling protocol buffer files...")
            result = subprocess.run([sys.executable, str(script_path)],
                                  capture_output=True, text=True, check=True)
            print(result.stdout)
            return True
        except subprocess.CalledProcessError as e:
            print(f"Proto compilation failed: {e}")
            print(f"stdout: {e.stdout}")
            print(f"stderr: {e.stderr}")
            return False
        except Exception as e:
            print(f"Error during proto compilation: {e}")
            return False


# Read the pyproject.toml for project metadata
# For now, we'll use basic metadata since pyproject.toml parsing is complex
setup(
    name="forglove-helper",
    version="0.1.0",
    description="Foxglove helper utilities with automatic proto compilation",
    author="kszheng",
    author_email="2436809937@qq.com",
    python_requires=">=3.10",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    entry_points={
        "console_scripts": [
            "forglove-helper=forglove_helper:main",
        ],
    },
    cmdclass={
        'build_py': BuildWithProtos,
    },
    install_requires=[
        # Add your dependencies here
    ],
    extras_require={
        "dev": [
            "protobuf>=3.20.0",  # For protoc compiler
        ],
    },
)
