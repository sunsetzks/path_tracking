from foxglove import Channel, Schema
from google.protobuf import descriptor_pb2, message
from typing import Type

def create_proto_channel(topic: str, proto_msg_cls: Type[message.Message]) -> Channel:
    """
    Create a Foxglove Channel for a generic protobuf message class.

    Args:
        topic (str): The topic name for the channel.
        proto_msg_cls (Type[message.Message]): The protobuf message class (e.g., fruit_pb2.Apple).

    Returns:
        Channel: Configured Foxglove Channel for sending custom protobuf messages.
    """
    proto_fds = descriptor_pb2.FileDescriptorSet()
    proto_msg_cls.DESCRIPTOR.file.CopyToProto(proto_fds.file.add())
    descriptor = proto_msg_cls.DESCRIPTOR
    proto_chan = Channel(
        topic=topic,
        message_encoding="protobuf",
        schema=Schema(
            name=f"{descriptor.file.package}.{descriptor.name}",
            encoding="protobuf",
            data=proto_fds.SerializeToString(),
        ),
    )
    return proto_chan

def log_proto_message(proto_chan: Channel, msg: message.Message, log_time: int):
    """
    Log any protobuf message to the given Foxglove Channel.

    Args:
        proto_chan (Channel): The channel returned by create_proto_channel.
        msg (message.Message): The protobuf message instance.
        log_time (int): Log time to use for the message.
    """
    proto_chan.log(msg.SerializeToString(), log_time=log_time)