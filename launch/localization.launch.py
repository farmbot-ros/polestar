import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import yaml
import argparse
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    param_file = os.path.join(get_package_share_directory('farmbot_localization'), 'config', 'params.yaml')

    nodes_array = []

    single_antenna = Node(
        package='farmbot_localization',
        namespace=namespace,
        executable='single_antenna',
        name='single_antenna',
        parameters=[
            yaml.safe_load(open(param_file))['single_antenna']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )

    dual_antenna = Node(
        package='farmbot_localization',
        namespace=namespace,
        executable='dual_antenna',
        name='dual_antenna',
        parameters=[
            yaml.safe_load(open(param_file))['dual_antenna']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )

    fix_n_bearing = Node(
        package='farmbot_localization',
        namespace=namespace,
        executable='fix_n_bearing',
        name='fix_n_bearing',
        parameters=[
            yaml.safe_load(open(param_file))['fix_n_bearing']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )


    # "single_antenna" or "dual_antenna" or "fix_n_bearing"
    localization_type = yaml.safe_load(open(param_file))['global']['ros__parameters']['localization_type']

    if localization_type == "fix_n_bearing":
        nodes_array.append(fix_n_bearing)
    elif localization_type == "single_antenna":
        nodes_array.append(single_antenna)
        nodes_array.append(dual_antenna)
    elif localization_type == "dual_antenna":
        nodes_array.append(dual_antenna)

    using_enu = Node(
        package='farmbot_localization',
        namespace=namespace,
        executable='using_enu',
        name='using_enu',
        parameters=[
            yaml.safe_load(open(param_file))['using_enu']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters'],
        ]
    )
    nodes_array.append(using_enu)

    odom_n_path = Node(
        package='farmbot_localization',
        namespace=namespace,
        executable='odom_n_path',
        name='odom_n_path',
        parameters=[
            yaml.safe_load(open(param_file))['odom_n_path']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    nodes_array.append(odom_n_path)

    transform_pub = Node(
        package='farmbot_localization',
        namespace=namespace,
        executable='transform_pub',
        name='transform_pub',
        parameters=[
            yaml.safe_load(open(param_file))['transform_pub']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    nodes_array.append(transform_pub)

    cord_convert = Node(
        package='farmbot_localization',
        namespace=namespace,
        executable='cord_convert',
        name='cord_convert',
        parameters=[
            yaml.safe_load(open(param_file))['cord_convert']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    nodes_array.append(cord_convert)

    static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", namespace+"/base_link", namespace+"/gps"],
        name="base_link_to_base_footprint"
    )
    nodes_array.append(static_transform)

    return nodes_array


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument('namespace', default_value='fbot')
    antena_arg = DeclareLaunchArgument('double_antenna', default_value='True')

    return LaunchDescription([
        namespace_arg,
        antena_arg,
        OpaqueFunction(function = launch_setup)
    ])
