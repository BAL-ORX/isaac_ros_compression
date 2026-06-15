import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path').perform(context)

    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)['encoder']

    encoder_params = {
        'input_width': cfg['input_width'],
        'input_height': cfg['input_height'],
        'config': cfg['config'],
    }

    reliability = 'best_effort' if cfg['image_qos'] == 'SENSOR_DATA' else 'reliable'
    qos_overrides = {
        'qos_overrides./image_raw.subscription.reliability': reliability,
        'qos_overrides./image_raw.subscription.durability': 'volatile',
    }

    encoder_node = ComposableNode(
        name='encoder_node',
        package='isaac_ros_h264_encoder',
        plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        parameters=[encoder_params, qos_overrides],
        remappings=[
            ('image_raw', cfg['image_topic']),
            ('image_compressed', cfg['compressed_topic']),
        ],
    )

    container = ComposableNodeContainer(
        name='encoder_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[encoder_node],
        output='screen',
    )

    return [container]


def generate_launch_description():
    default_config = os.path.join('/workspaces/isaac_ros-dev', 'config_orx.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config,
            description='Path to the ORX configuration YAML. '
                        'Defaults to config_orx.yaml at the workspace root.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
