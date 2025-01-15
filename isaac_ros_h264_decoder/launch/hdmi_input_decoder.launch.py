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
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import yaml


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file")

    with open(config_file.perform(context), "r") as file:
        config = yaml.safe_load(file)
        datahub_name = config["datahub_name"]
        device_name = config["device_name"]

    hdmi_input_compressed_topic = PathJoinSubstitution([datahub_name, device_name, "image_raw", "h264"])
    hdmi_input_decompressed_topic = PathJoinSubstitution([datahub_name, device_name, "image_raw", "decompressed"])

    rgb_decoder_node = ComposableNode(
        name=f"decoder_{device_name}_rgb",
        namespace=datahub_name,
        plugin="nvidia::isaac_ros::h264_decoder::DecoderNode",
        remappings=[
            ("image_compressed", hdmi_input_compressed_topic),
            ("image_uncompressed", hdmi_input_decompressed_topic),
        ],
    )

    container = ComposableNodeContainer(
        namespace=datahub_name,
        name=f"decode_container_{device_name}",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[rgb_decoder_node],
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
    )
    return [container]


def generate_launch_description():
    """Launch the H.264 Decoder Node."""
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value="/hdmi_input_config.yaml",
                description="The path to the config file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
