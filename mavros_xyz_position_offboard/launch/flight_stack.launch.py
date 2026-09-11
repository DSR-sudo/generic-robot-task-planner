from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    params_file = LaunchConfiguration("params_file")
    start_mavros = LaunchConfiguration("start_mavros")
    udp_enabled = LaunchConfiguration("udp_enabled")
    udp_bind_ip = LaunchConfiguration("udp_bind_ip")
    udp_bind_port = LaunchConfiguration("udp_bind_port")
    udp_remote_ip = LaunchConfiguration("udp_remote_ip")
    udp_remote_port = LaunchConfiguration("udp_remote_port")
    udp_whitelist_ip = LaunchConfiguration("udp_whitelist_ip")
    udp_whitelist_port = LaunchConfiguration("udp_whitelist_port")
    enable_control = LaunchConfiguration("enable_control")
    mission_xml_path = LaunchConfiguration("mission_xml_path")

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            get_package_share_directory("mavros") + "/launch/px4.launch"
        ),
        launch_arguments={"fcu_url": fcu_url, "gcs_url": gcs_url}.items(),
        condition=IfCondition(start_mavros),
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("lslidar_driver") + "/launch/lsn10p_launch.py"
        )
    )
    application_arguments = [
        "--range-topic", "/mavros/px4flow/ground_distance",
        "--optical-flow-topic", "/mavros/px4flow/raw/optical_flow_rad",
    ]
    application_parameters = [
        params_file,
        {
            "udp.enabled": ParameterValue(udp_enabled, value_type=bool),
            "udp.bind_ip": udp_bind_ip,
            "udp.bind_port": ParameterValue(udp_bind_port, value_type=int),
            "udp.remote_ip": udp_remote_ip,
            "udp.remote_port": ParameterValue(udp_remote_port, value_type=int),
            "udp.whitelist_ip": udp_whitelist_ip,
            "udp.whitelist_port": ParameterValue(udp_whitelist_port, value_type=int),
            "mission.xml_path": mission_xml_path,
        },
    ]
    monitoring_application = Node(
        package="mavros_xyz_position_offboard",
        executable="mavros_xyz_position_node",
        arguments=application_arguments,
        parameters=application_parameters,
        condition=UnlessCondition(enable_control),
        output="screen",
    )
    flight_application = Node(
        package="mavros_xyz_position_offboard",
        executable="mavros_xyz_position_node",
        arguments=application_arguments + [
            "--enable-control",
        ],
        parameters=application_parameters,
        condition=IfCondition(enable_control),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "fcu_url",
            default_value="/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:2000000",
        ),
        DeclareLaunchArgument("gcs_url", default_value="udp://127.0.0.1:14551@"),
        DeclareLaunchArgument("start_mavros", default_value="true"),
        DeclareLaunchArgument("udp_enabled", default_value="true"),
        DeclareLaunchArgument("udp_bind_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("udp_bind_port", default_value="5005"),
        DeclareLaunchArgument("udp_remote_ip", default_value="192.168.10.22"),
        DeclareLaunchArgument("udp_remote_port", default_value="5005"),
        DeclareLaunchArgument("udp_whitelist_ip", default_value="192.168.10.22"),
        DeclareLaunchArgument("udp_whitelist_port", default_value="5005"),
        DeclareLaunchArgument("enable_control", default_value="false"),
        DeclareLaunchArgument(
            "mission_xml_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("mavros_xyz_position_offboard"),
                "config",
                "generic_flight_mission.xml",
            ]),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("mavros_xyz_position_offboard"),
                "config",
                "udp_ground_station.yaml",
            ]),
        ),
        mavros_launch,
        lidar_launch,
        monitoring_application,
        flight_application,
    ])
