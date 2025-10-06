#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 把“深度相机驱动”与“颜色检测节点”串起来
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def launch_setup(context):
    # get this package's share directory
    pkg_share   = get_package_share_directory('robot_cam')
    contrib_dir = os.path.join(pkg_share, 'launch')

    # 1) 启动深度相机流水线
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(contrib_dir, 'depth_camera.launch.py')
        ),
    )

    # 2) 声明显示参数：普通图像 & ROI 叠加
    display_arg = DeclareLaunchArgument(
        'enable_display', default_value='true',
        description='Enable OpenCV display'
    )
    display_roi_arg = DeclareLaunchArgument(
        'enable_roi_display', default_value='true',
        description='Enable ROI overlay'
    )
    display_target_colors_arg = DeclareLaunchArgument(
        'target_colors',
        default_value='["red"]',
        description='Detect target color like ["red","green"]'
    )
    # display_detect_type_arg = DeclareLaunchArgument(
    #     'detect_type',
    #     default_value='color',
    #     description='Detect type, can be "color" or "depth"'
    # )
    # 3) 启动颜色检测节点
    roi_yaml = os.path.join(pkg_share, 'config', 'roi.yaml')
    # lab_yaml = os.path.join(pkg_share, 'config', 'lab_config.yaml')
    color_detect_node = Node(
        package='robot_cam',
        executable='color_detect',
        name='color_detect',
        output='screen',
        parameters=[
            roi_yaml,
            # lab_yaml,
            {
                'enable_display': LaunchConfiguration('enable_display'),
                'enable_roi_display': LaunchConfiguration('enable_roi_display'),
                'target_colors': LaunchConfiguration('target_colors'),
                # 'detect_type': LaunchConfiguration('detect_type'),
            }
        ]
    )

    return [
        display_arg,
        display_roi_arg,
        display_target_colors_arg,
        # display_detect_type_arg,
        depth_camera_launch,
        color_detect_node,
    ]

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld

if __name__ == '__main__':
    generate_launch_description()
