import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    chassis_bringup = get_package_share_directory('tinker_chassis_bringup')
    chassis_launch_dir = os.path.join(chassis_bringup, 'launch')
    lidar_launch_dir = os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch')
    local_bringup_dir = get_package_share_directory('tinker_nav_bringup')

    start_rviz = LaunchConfiguration("start_rviz") 
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_teleop = LaunchConfiguration("use_teleop")
    slam_params_file = LaunchConfiguration('slam_params_file')
    localization_params_file = LaunchConfiguration('localization_params_file')


    declare_rviz_cmd = DeclareLaunchArgument(
        'start_rviz',
        default_value='True',
        description='Start RViz2 automatically with this launch file.')

    declare_teleop_cmd = DeclareLaunchArgument(
        'use_teleop',
        default_value='True',
        description='Use teleop use control Tinker')
  
    # Localization and SLAM
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(local_bringup_dir,
                                   'params', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_localization_params_file_cmd = DeclareLaunchArgument(
        'localization_params_file',
        default_value=os.path.join(local_bringup_dir,
                                   'params', 'ekf.yaml'),
        description='Full path to the ROS2 parameters file to use for the localization')

    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(local_bringup_dir,
                                   'config', 'slam_config.rviz'),
        description='Full path to the rviz config file')

    declared_arguments = [
        declare_rviz_cmd,
        declare_teleop_cmd,
        declare_localization_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_rviz_config_file
    ] 

    # Localization is used to publish tf2 message
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[localization_params_file]
    )
    start_async_slam_toolbox_node = Node(
        parameters=[slam_params_file],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    # Robot
    chassis_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(chassis_launch_dir, 'chassis_bringup.launch.py'))
        )
    
    lidar_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_launch_dir, 'laserscan_MID360_launch.py'))
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
    )

    lidar_odom_node = Node(
       package='lidar_odom',
       executable='lidar_odom',
       name='lidar_odom',
       output='screen'
    )

    teleop_node= Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop",
        prefix = 'gnome-terminal --',
        remappings=[   
            ('/cmd_vel','/tinker_controller/cmd_vel_unstamped')
            ],
    )


    
    nodes = [rviz_node, 
             lidar_bringup_cmd, 
             chassis_bringup_cmd, 
             robot_localization_node, 
             lidar_odom_node,
            # teleop_node,
             start_async_slam_toolbox_node]
    return LaunchDescription(declared_arguments +  nodes)