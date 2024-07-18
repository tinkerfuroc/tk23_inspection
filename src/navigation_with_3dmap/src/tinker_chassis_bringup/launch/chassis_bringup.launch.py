import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Declare parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    sim_time_declare_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true')

    ros2_control_declare_cmd = DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true')  

    pkg_path = os.path.join(get_package_share_directory('tinker_description'))
    xacro_file = os.path.join(pkg_path,'src/description','tinker_chassis.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
        ]
    )

    # Get params for ros2 control
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory("tinker_description"), 'config', 'robot_controller_config.yaml')

    # Setup controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    tinker_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tinker_controller"],
    )

    delayed_tinker_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[tinker_controller_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    return LaunchDescription([
        sim_time_declare_cmd,
        ros2_control_declare_cmd,
        node_robot_state_publisher,
        delayed_controller_manager,
        delayed_tinker_controller_spawner,
        delayed_joint_broad_spawner,
    ])
