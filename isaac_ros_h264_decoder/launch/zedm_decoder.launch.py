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
    ros_params_override_path = LaunchConfiguration("ros_params_override_path")

    with open(ros_params_override_path.perform(context), "r") as file:
        ros_params_override_config = yaml.safe_load(file)
        datahub_name = ros_params_override_config["/**"]["ros__parameters"]["general"]["datahub_name"]
        camera_name = ros_params_override_config["/**"]["ros__parameters"]["general"]["camera_name"]

    # datahub_name comes from node namespace
    left_encoded_topic = PathJoinSubstitution([camera_name, "left", "image_rect_color", "h264"])
    left_decompressed_topic = PathJoinSubstitution([camera_name, "left", "image_rect_color", "decompressed"])
    right_encoded_topic = PathJoinSubstitution([camera_name, "right", "image_rect_color", "h264"])
    right_decompressed_topic = PathJoinSubstitution([camera_name, "right", "image_rect_color", "decompressed"])

    left_decoder_node = ComposableNode(
        name=f"decoder_{camera_name}_left",
        namespace=datahub_name,
        package="isaac_ros_h264_decoder",
        plugin="nvidia::isaac_ros::h264_decoder::DecoderNode",
        remappings=[("image_compressed", left_encoded_topic), ("image_uncompressed", left_decompressed_topic)],
    )

    right_decoder_node = ComposableNode(
        name=f"decoder_{camera_name}_right",
        namespace=datahub_name,
        package="isaac_ros_h264_decoder",
        plugin="nvidia::isaac_ros::h264_decoder::DecoderNode",
        remappings=[("image_compressed", right_encoded_topic), ("image_uncompressed", right_decompressed_topic)],
    )

    container = ComposableNodeContainer(
        namespace=datahub_name,
        name=f"decode_container_{camera_name}",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[left_decoder_node, right_decoder_node],
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
    )
    return [container]


def generate_launch_description():
    """Launch the H.264 Decoder Node."""
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "ros_params_override_path",
                default_value="/zed_mini_ros_config.yaml",
                description="The path to an additional parameters file to override the defaults",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
