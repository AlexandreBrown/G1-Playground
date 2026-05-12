from unitree_sdk2py.core.channel import ChannelFactoryInitialize


def create_dds_topic_to_communicate_with_g1(argv: list):
    channel_id = 0
    if len(argv)>1:
        network_interface = argv[1]
        ChannelFactoryInitialize(channel_id, network_interface)
    else:
        ChannelFactoryInitialize(channel_id)
