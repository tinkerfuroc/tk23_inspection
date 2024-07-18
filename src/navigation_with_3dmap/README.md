# tk23_ws
The new workspace for 2023-2024


# Navigation

Navigation is based on the [`nav2`](https://navigation.ros.org/) based on ROS2. To deploy the navigation stack, the dependencies should be installed locally first following [nav2 dependency installation](https://navigation.ros.org/getting_started/index.html).

## Packages

- `tinker_description`: The descrption files for tinker, including `.stl` file, `.urdf` file and a python launch file `display.launch.py`.
- `livox_ros_driver2`: Driver for Livox Lidar MID360, modified to be compiled with `humble` only, see package `README.md` for details.
- `realsense-ros`: Driver for realsense, see package `README.md` for details.
- `tinker_chassis_hardware`: Define the hardware interface, to be accessed by ros2 control resources manager.
- `tinker_chassis_controller`: A simple chassis controller.
- `tinker_chassis_bringup`: Bring up the chassis.

## Chassis Setup
Setup usb can communication:
```sh
sudo chmod +x ./setupcan.sh
sudo ./setupcan.sh
```
Launch the interface and controller:
```sh
ros2 launch tinker_chassis_bringup tinker_bringup.launch.py
```
To send command to chassis, publish on `cmd_vel` topic.

## Lidar setup
1. Attach the ip address in `livox_ros_driver2/config/MID360_config.json` `host_net_info` to the NIC which lidar is connected.
2. Launch lidar:
    ```sh
    ros2 launch livox_ros_driver2 rviz_MID360_launch.py
    ```
3. The pointcloud and imu message can be accessed via topic `livox/lidar` and `livox/imu`.
