import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import yaml
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    audodatum = LaunchConfiguration("autodatum").perform(context)
    param_file = os.path.join(
        get_package_share_directory("farmbot_polestar"), "config", "params.yaml"
    )

    nodes_array = []

    fix_n_bearing = Node(
        package="farmbot_polestar",
        namespace=namespace,
        executable="fix_n_bearing",
        name="fix_n_bearing",
        parameters=[
            yaml.safe_load(open(param_file))["global"]["ros__parameters"],
            {"autodatum": audodatum} if audodatum != "" else {},
        ],
    )
    nodes_array.append(fix_n_bearing)

    using_enu = Node(
        package="farmbot_polestar",
        namespace=namespace,
        executable="using_enu",
        name="using_enu",
        parameters=[
            yaml.safe_load(open(param_file))["global"]["ros__parameters"],
            {"autodatum": audodatum} if audodatum != "" else {},
        ],
    )
    nodes_array.append(using_enu)

    odom_n_path = Node(
        package="farmbot_polestar",
        namespace=namespace,
        executable="odom_n_path",
        name="odom_n_path",
        parameters=[
            yaml.safe_load(open(param_file))["global"]["ros__parameters"],
            {"autodatum": audodatum} if audodatum != "" else {},
        ],
    )
    nodes_array.append(odom_n_path)

    transform_pub = Node(
        package="farmbot_polestar",
        namespace=namespace,
        executable="transform_pub",
        name="transform_pub",
        parameters=[
            yaml.safe_load(open(param_file))["global"]["ros__parameters"],
            {"autodatum": audodatum} if audodatum != "" else {},
        ],
    )
    nodes_array.append(transform_pub)

    cord_convert = Node(
        package="farmbot_polestar",
        namespace=namespace,
        executable="cord_convert",
        name="cord_convert",
        parameters=[
            yaml.safe_load(open(param_file))["global"]["ros__parameters"],
            {"autodatum": audodatum} if audodatum != "" else {},
        ],
    )
    nodes_array.append(cord_convert)

    static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "1",
            namespace + "/base_link",
            namespace + "/gps",
        ],
        name="base_link_to_base_footprint",
    )
    nodes_array.append(static_transform)

    return nodes_array


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="fbot")
    autodatum_arg = DeclareLaunchArgument("autodatum", default_value="")

    return LaunchDescription(
        [namespace_arg, autodatum_arg, OpaqueFunction(function=launch_setup)]
    )
