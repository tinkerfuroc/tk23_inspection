gnome-terminal --tab -- zsh -c ". install/setup.zsh; rviz2 -d src/lidar_localization_ros2/rviz/localization.rviz"
sleep 4
# Use xbox joystick
gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 launch pcl_localization_ros2 pcl_localization.launch.py"
sleep 10