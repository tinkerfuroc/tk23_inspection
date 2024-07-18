gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 launch tinker_chassis_bringup chassis_bringup.launch.py"

sleep 10

gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/tinker_controller/cmd_vel_unstamped
"

sleep 3

gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 launch livox_ros_driver2 laserscan_MID360_launch.py"

# sleep 5

# gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run rviz2 rviz2 -d /home/qxy/tk23_navigation/src/tinker_nav_bringup/config/chassis_lidar.rviz
# "