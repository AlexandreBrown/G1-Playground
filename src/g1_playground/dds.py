from unitree_sdk2py.core.channel import ChannelFactoryInitialize


def create_dds_topic_to_communicate_with_g1(dds_channel_id: int = 1, network_interface: str | None = None):
    if network_interface:
        ChannelFactoryInitialize(dds_channel_id, network_interface)
    else:
        ChannelFactoryInitialize(dds_channel_id)
