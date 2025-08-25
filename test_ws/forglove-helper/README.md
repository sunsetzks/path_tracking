# forglove-helper

A Python library providing utilities for working with Foxglove WebSocket connections, including automatic Protocol Buffer compilation during installation.

## Features

- **Automatic Proto Compilation**: Protocol Buffer files are automatically compiled during installation
- **Foxglove Integration**: Easy-to-use utilities for Foxglove WebSocket communication
- **Custom Protobuf Support**: Helper functions for creating Foxglove channels with custom protobuf messages
- **Visualization Utilities**: Plotting and visualization helpers for robotics applications

## Installation

### Prerequisites

Before installing forglove-helper, ensure you have the Protocol Buffer compiler installed:

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install protobuf-compiler
```

**macOS:**
```bash
brew install protobuf
```

**Windows:**
Download and install from [protobuf releases](https://github.com/protocolbuffers/protobuf/releases)

### Install from Source

Clone the repository and install:

```bash
git clone <repository-url>
cd forglove-helper
pip install .
```

The Protocol Buffer files will be automatically compiled during installation.

### Development Installation

For development with additional dependencies:

```bash
pip install -e ".[dev]"
```

## Protocol Buffer Support

### Automatic Compilation

When you install or build the package, all `.proto` files in the project are automatically compiled to Python using `protoc`. The generated files are placed in the `src/forglove_helper/protos/` directory.

### Adding New Proto Files

1. Add your `.proto` files to the `src/forglove_helper/protos/` directory
2. Reinstall the package: `pip install .`
3. The new proto files will be automatically compiled

### Manual Compilation

You can also compile proto files manually:

```bash
python scripts/compile_protos.py
```

### Using Compiled Protos

After compilation, you can import and use the generated Python classes:

```python
from forglove_helper.protos.custom_person_pb2 import Person

# Create a message
person = Person()
person.name = "John Doe"
person.id = 123
person.email = "john@example.com"

# Use with forglove-helper
from forglove_helper.proto_helper import create_proto_channel
channel = create_proto_channel("person_topic", Person)
```

## Usage Examples

### Basic Channel Management

```python
from forglove_helper import ChannelManager

# Create channel manager
manager = ChannelManager()

# Add channels and log data
# ... see examples/ for detailed examples
```

### Custom Protobuf Messages

```python
from forglove_helper.proto_helper import create_proto_channel, log_proto_message
from forglove_helper.protos.custom_person_pb2 import Person
import time

# Create a channel for custom protobuf messages
channel = create_proto_channel("person_data", Person)

# Create and log a message
person = Person(name="Alice", id=456, email="alice@example.com")
log_proto_message(channel, person, int(time.time() * 1e9))
```

## Examples

See the `examples/` directory for detailed usage examples:

- `channel_demo.py` - Basic channel management
- `custom_frame_example.py` - Custom frame IDs for visualizations
- `plot_demo.py` - Plotting utilities
- `dna_helix_advanced_demo.py` - Advanced 3D visualization

## Project Structure

```
forglove-helper/
├── src/forglove_helper/
│   ├── __init__.py
│   ├── channel_manager.py     # Channel management utilities
│   ├── channel_utils.py       # Channel utility functions
│   ├── channels.py           # Channel definitions
│   ├── proto_helper.py       # Protobuf helper functions
│   └── protos/              # Protocol Buffer files and generated code
│       ├── custom_person.proto
│       ├── custom_person_pb2.py  # Auto-generated
│       └── README.txt
├── examples/                 # Usage examples
├── scripts/
│   └── compile_protos.py    # Proto compilation script
├── pyproject.toml           # Project configuration
├── setup.py                 # Build configuration with auto-compilation
└── README.md
```

## Development

### Adding New Proto Files

1. Add your `.proto` file to `src/forglove_helper/protos/`
2. Run `python scripts/compile_protos.py` to compile manually
3. Or reinstall the package to trigger automatic compilation

### Build Process

The build process automatically:

1. Compiles all `.proto` files using `protoc`
2. Generates Python files in `src/forglove_helper/protos/`
3. Includes the generated files in the package

### Dependencies

- `protobuf>=3.20.0` - Protocol Buffer support
- `foxglove-websocket>=0.0.5` - Foxglove WebSocket client

## Troubleshooting

### Proto Compilation Fails

If proto compilation fails during installation:

1. Ensure `protoc` is installed and in your PATH
2. Check that your `.proto` files have valid syntax
3. Run `python scripts/compile_protos.py` manually for detailed error messages

### Import Errors

If you get import errors for generated proto files:

1. Ensure the package was installed correctly with proto compilation
2. Try reinstalling: `pip uninstall forglove-helper && pip install .`
3. Check that all required dependencies are installed

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure proto compilation works correctly
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.
