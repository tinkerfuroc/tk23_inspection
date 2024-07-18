gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 launch tinker_chassis_bringup chassis_bringup.launch.py"

sleep 10

gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 launch livox_ros_driver2 laserscan_MID360_launch.py"

sleep 5

# Provide filtered Pointcloud
gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run lidar_odom lidar_odom"

sleep 5

# gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml"

# sleep 5

gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 launch tinker_nav_bringup tinker_navigation.launch.py"

sleep 8

gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 launch pcl_localization_ros2 pcl_localization.launch.py"
sleep 10

