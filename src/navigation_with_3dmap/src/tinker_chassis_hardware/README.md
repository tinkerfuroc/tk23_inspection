# Migration Notes

## Structure
1. `include/libcan`: definition for can frames and can adapter
2. `include/config.hpp`: the parameters of motors, directly moved from the original repo.
3. `include/hardware.hpp`: define CAN frame id and frame format, also transmit the can frame.

## Class relationship
- `RobotHardware` is the actual hardware access layer, it receives can frames and transmit can frames, it is instantiated only once, For each motor, `RobotHardware` has a entry for it to transmit the target pwm value.
- `Motor`: `Motor` is not a class oriented to hardware, it is used to compute the real pwm value using pid controller. It is instantiated 4 times for chassis motor. Each is instantiated in `MotorInterface`, and receives a setpoint, computes the target pwm, and then `RobotHardware` transmit the frame to the real motor. Technically the implementation of `Motor` class is a violation of philosophy of `ros2_control`. However, this is a legacy framework. 

## Parameters
- `motor_id`: take 0, 1, 2, 3
- `kp,ki,kd`: pid parameters
## Troubleshooting

SInce `hardware_interface` is not a node, it is not possible to get the clock of node. So `rclcpp::Clock` is used instead of `rclcpp:::Time`