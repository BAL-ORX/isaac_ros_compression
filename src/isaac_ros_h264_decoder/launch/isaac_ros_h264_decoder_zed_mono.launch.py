import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# ros2 launch isaac_ros_h264_decoder isaac_ros_h264_decoder_zed_mono.launch.py



def generate_launch_description():
    decoder_node = ComposableNode(
        name='decoder',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        remappings=[
            ('image_compressed', '/zed/zed_node/rgb/color/rect/image_h264'),
            ('image_uncompressed', '/zed/zed_node/rgb/color/rect/image_raw'),
        ])

    container = ComposableNodeContainer(
        name='decoder_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[decoder_node],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return launch.LaunchDescription([container])
