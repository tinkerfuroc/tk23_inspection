gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 launch tinker_nav_bringup tinker_slam.launch.py"
sleep 10
# Use xbox joystick
gnome-terminal --tab -- zsh -c "sudo xboxdrv --silent --dpad-as-button "
sleep 10
gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' joy_vel:="/tinker_controller/cmd_vel_unstamped""
