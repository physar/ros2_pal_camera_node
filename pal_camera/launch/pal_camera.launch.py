#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    package_dir = get_package_share_directory('pal_camera')


    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    cam_pos_x = LaunchConfiguration('cam_pos_x')
    cam_pos_y = LaunchConfiguration('cam_pos_y')
    cam_pos_z = LaunchConfiguration('cam_pos_z')
    cam_roll = LaunchConfiguration('cam_roll')
    cam_pitch = LaunchConfiguration('cam_pitch')
    cam_yaw = LaunchConfiguration('cam_yaw')


    default_camera_model = 'pal_usb'
    default_camera_name  = '/dreamvu/pal/'

    config_camera_path = os.path.join(
            package_dir,
            'config',
            default_camera_model + '.yaml'
        )

    default_config_camera_path = os.path.join(
            package_dir,
            'config',
            default_camera_model + '.yaml'
        )

    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        default_value=default_camera_name,
        description='The name of the camera. Handy for a robot with multiple cameras. It can be different from the camera model and it will be used as node `namespace`.')

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=default_camera_model,
        description='The model of the camera. Currently only the `pal`-model implemented based on the `pal_usb`, but the `pal_mini` and `pal_alia` have quite different dimensions and max_depth. Valid models: `pal`, `pal_usb`, `pal_mini`, `pal_alia`.')

    declare_config_camera_path_cmd = DeclareLaunchArgument(
        'config_camera_path',
        default_value=default_config_camera_path,
        description='Path to the `<camera_model>.yaml` file.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value='0.0',
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value='0.0',
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value='0.06',
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value='0.0',
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value='0.0',
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value='0.0',
        description='Yaw orientation of the camera with respect to the base frame.')


   # capture node
    capture_node = Node(
        package='pal_camera',
        namespace=camera_name,
        executable='capture',
        name='pal_camera_capture',
        output='screen',
        parameters=[
            config_camera_path,
        # Overriding
        {
        #            'camera_name': camera_name, 
        #            'camera_model': camera_model,
        #            'cam_pos_x': cam_pos_x,
        #            'cam_pos_y': cam_pos_y,
        #            'cam_pos_z': cam_pos_z,
        #            'cam_roll': cam_roll,
        #            'cam_pitch': cam_pitch,
        #            'cam_yaw': cam_yaw
        }
        ]
    )

# Define LaunchDescription variable and return it

    ld = LaunchDescription()

    ld.add_action(declare_camera_name_cmd)
    ld.add_action(declare_camera_model_cmd)

    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    ld.add_action(capture_node)

    return ld

