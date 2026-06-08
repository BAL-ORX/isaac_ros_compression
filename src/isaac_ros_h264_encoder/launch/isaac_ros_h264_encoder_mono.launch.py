import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def load_config(config_path: str) -> dict:
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)['encoder']


def generate_launch_description():
    pkg_share = get_package_share_directory('isaac_ros_h264_encoder')
    config_path = os.path.join(pkg_share, 'config', 'zed_mono.yaml')
    cfg = load_config(config_path)

    input_width = cfg['input_width']
    input_height = cfg['input_height']
    encoder_config = cfg['config']
    image_qos = cfg['image_qos']
    in_topic = cfg['image_topic']
    out_topic = cfg['compressed_topic']

    encoder_params = {
        'input_width': input_width,
        'input_height': input_height,
        'config': encoder_config,
    }

    # QoS overrides must match the camera driver publisher or no frames arrive.
    qos_overrides = {
        'qos_overrides./image_raw.subscription.reliability':
            'best_effort' if image_qos == 'SENSOR_DATA' else 'reliable',
        'qos_overrides./image_raw.subscription.durability': 'volatile',
    }

    encoder_node = ComposableNode(
        name='encoder_node',
        package='isaac_ros_h264_encoder',
        plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        parameters=[encoder_params, qos_overrides],
        remappings=[
            ('image_raw', in_topic),
            ('image_compressed', out_topic),
        ],
    )

    container = ComposableNodeContainer(
        name='stereo_encoder_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[encoder_node],
        output='screen',
    )

    return launch.LaunchDescription([container])
