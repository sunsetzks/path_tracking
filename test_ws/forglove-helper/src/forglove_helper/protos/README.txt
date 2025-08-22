This directory contains .proto files for demo purposes.

To generate Python bindings run:

  python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. custom_person.proto

Or with protoc installed:

  protoc --proto_path=. --python_out=. custom_person.proto

The generated file will be `custom_person_pb2.py` which `foxglove_helper.py` will import.


