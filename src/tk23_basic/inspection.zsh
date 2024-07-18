
gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true depth_module.profile:=1280x720x15 rgb_camera.profile:=1280x720x15"

sleep 5

gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run object_detection service"

sleep 5

gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run robocup_tasks inspection"

