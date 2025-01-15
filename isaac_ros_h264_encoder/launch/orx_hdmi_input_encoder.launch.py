# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch import LaunchContext

import yaml


def launch_setup(context, *args, **kwargs):
    encoder_config = LaunchConfiguration("encoder_config")
    config_file = LaunchConfiguration("config_file")

    with open(config_file.perform(context), "r") as file:
        config = yaml.safe_load(file)
        datahub_name = config["datahub_name"]
        device_name = config["device_name"]
        input_height = config["input_height"]
        input_width = config["input_width"]
        frame_rate = config["frame_rate"]

    # datahub_name comes from node namespace
    hdmi_input_raw_topic = PathJoinSubstitution([device_name, "image_raw"])
    hdmi_input_compressed_topic = PathJoinSubstitution([device_name, "image_raw", "h264"])

    encoder_node_rgb = ComposableNode(
        name=f"encoder_{device_name}_rgb",
        namespace=datahub_name,
        package="isaac_ros_h264_encoder",
        plugin="nvidia::isaac_ros::h264_encoder::EncoderNode",
        parameters=[
            {
                "input_height": input_height,
                "input_width": input_width,
                "config": encoder_config,
                "iframe_interval": frame_rate,
                "use_intra_process_comms": True,
            }
        ],
        remappings=[
            ("image_raw", hdmi_input_raw_topic),
            ("image_compressed", hdmi_input_compressed_topic),
        ],
    )

    container = ComposableNodeContainer(
        namespace=datahub_name,
        name=f"encoder_container_{device_name}",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[encoder_node_rgb],
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
    )

    return [container]


def generate_launch_description():
    """Launch the H.264 Encoder Node."""
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("encoder_config", default_value="custom", description="Config of encoder"),
            DeclareLaunchArgument(
                "config_file",
                default_value="/hdmi_input_config.yaml",
                description="The path to the config file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
