#!/usr/bin/env python3
"""
Protocol Buffer Compilation Script

This script automatically compiles all .proto files in the project to Python files
using protoc. It should be run during the build process.

Usage:
    python scripts/compile_protos.py
"""

import os
import subprocess
import sys
from pathlib import Path


def find_proto_files(base_path: Path) -> list[Path]:
    """Find all .proto files in the project."""
    proto_files = []
    for proto_file in base_path.rglob("*.proto"):
        proto_files.append(proto_file)
    return proto_files


def compile_proto(proto_file: Path, proto_include_path: Path, output_path: Path) -> bool:
    """Compile a single .proto file to Python."""
    try:
        cmd = [
            "protoc",
            f"--proto_path={proto_include_path}",
            f"--python_out={output_path}",
            str(proto_file)
        ]

        print(f"Compiling {proto_file} -> {output_path}")
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error compiling {proto_file}: {e}")
        print(f"protoc stdout: {e.stdout}")
        print(f"protoc stderr: {e.stderr}")
        return False
    except FileNotFoundError:
        print("Error: protoc not found. Please install protobuf compiler.")
        print("On Ubuntu/Debian: sudo apt-get install protobuf-compiler")
        print("On macOS: brew install protobuf")
        return False


def main():
    """Main compilation function."""
    # Get the project root (parent of scripts directory)
    script_dir = Path(__file__).parent
    project_root = script_dir.parent

    print("Starting proto compilation...")

    # Find all proto files
    proto_files = find_proto_files(project_root)
    if not proto_files:
        print("No .proto files found in the project.")
        return 0

    print(f"Found {len(proto_files)} .proto files:")
    for proto_file in proto_files:
        print(f"  - {proto_file}")

    # Create output directory if it doesn't exist
    output_dir = project_root / "src" / "forglove_helper" / "protos"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Compile each proto file
    success_count = 0
    for proto_file in proto_files:
        # Calculate relative path for proto_path (include directory)
        proto_include_path = proto_file.parent

        if compile_proto(proto_file, proto_include_path, output_dir):
            success_count += 1

    print(f"\nCompilation complete: {success_count}/{len(proto_files)} files compiled successfully.")

    if success_count != len(proto_files):
        print("Some proto files failed to compile.")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
