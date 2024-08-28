# Copyright 2020 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


class GroundSegmentationPipeline:
    def __init__(self, context):
        self.context = context
        self.vehicle_info = self.get_vehicle_info()
        ground_segmentation_param_path = os.path.join(
            LaunchConfiguration("obstacle_segmentation_ground_segmentation_param_path").perform(
                context
            ),
        )
        with open(ground_segmentation_param_path, "r") as f:
            self.ground_segmentation_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        self.single_frame_obstacle_seg_output = (
            "/perception/obstacle_segmentation/single_frame/pointcloud_raw"
        )

    def get_vehicle_info(self):
        gp = self.context.launch_configurations.get("ros_params", {})
        if not gp:
            gp = dict(self.context.launch_configurations.get("global_params", {}))
        p = {}
        p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
        p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
        p["min_longitudinal_offset"] = -gp["rear_overhang"]
        p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
        p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
        p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
        p["min_height_offset"] = 0.0
        p["max_height_offset"] = gp["vehicle_height"]
        return p

    def create_common_pipeline(self, input_topic, output_topic):
        components = []
        components.append(
            ComposableNode(
                package="pointcloud_preprocessor",
                plugin="pointcloud_preprocessor::CropBoxFilterComponent",
                name="crop_box_filter",
                remappings=[
                    ("input", input_topic),
                    ("output", "range_cropped/pointcloud"),
                ],
                parameters=[
                    {
                        "input_frame": LaunchConfiguration("base_frame"),
                        "output_frame": LaunchConfiguration("base_frame"),
                    },
                    self.ground_segmentation_param["common_crop_box_filter"]["parameters"],
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

        components.append(
            ComposableNode(
                package="ground_segmentation",
                plugin=self.ground_segmentation_param["common_ground_filter"]["plugin"],
                name="common_ground_filter",
                remappings=[
                    ("input", "range_cropped/pointcloud"),
                    ("output", output_topic),
                ],
                parameters=[
                    self.ground_segmentation_param["common_ground_filter"]["parameters"],
                    self.vehicle_info,
                    {"input_frame": "base_link"},
                    {"output_frame": "base_link"},
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )
        return components

    def create_single_frame_obstacle_segmentation_components(self, input_topic, output_topic):
        components = self.create_common_pipeline(
            input_topic=input_topic,
            output_topic=output_topic,
        )
        return components


def launch_setup(context, *args, **kwargs):
    pipeline = GroundSegmentationPipeline(context)

    components = pipeline.create_single_frame_obstacle_segmentation_components(
        input_topic=LaunchConfiguration("input/pointcloud"),
        output_topic=pipeline.single_frame_obstacle_seg_output,
    )

    individual_container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=components,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )
    pointcloud_container_loader = LoadComposableNodes(
        composable_node_descriptions=components,
        target_container=LaunchConfiguration("container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )
    return [individual_container, pointcloud_container_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "True")
    add_launch_arg("use_pointcloud_container", "False")
    add_launch_arg("container_name", "perception_pipeline_container")
    add_launch_arg("input/pointcloud", "/sensing/lidar/concatenated/pointcloud")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
