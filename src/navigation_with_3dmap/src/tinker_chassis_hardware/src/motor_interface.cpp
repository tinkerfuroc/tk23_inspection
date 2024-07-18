#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "motor_interface/motor_interface.hpp"
#include "motor_interface/motor.hpp"

using namespace tinker_motor;

hardware_interface::CallbackReturn MotorInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    hardware_interface::CallbackReturn baseResult = hardware_interface::SystemInterface::on_init(hardware_info);
    if (baseResult != hardware_interface::CallbackReturn::SUCCESS) {
        return baseResult;
    }

    motor_ids_.resize(info_.joints.size());
    kps_.resize(info_.joints.size());
    kds_.resize(info_.joints.size());
    kis_.resize(info_.joints.size());
    is_reverse.resize(info_.joints.size());
    position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_saved_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    
    for (hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.parameters["motor_id"].empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("MotorInterface"), "Motor id not defined for joint %s", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.parameters["kp"].empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("MotorInterface"), "No kp defined for joint %s", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.parameters["ki"].empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("MotorInterface"), "No ki defined for joint %s", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.parameters["kd"].empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("MotorInterface"), "No kd defined for joint %s", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("MotorInterface"), "Invalid number of command interfaces (expected: 1)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MotorInterface"), "Invalid joint command interface 0 type (expected: velocity)");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("MotorInterface"), "Invalid number of state interfaces (expected: 2)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("MotorInterface"), "Invalid joint state interface 0 type (expected: position)");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("MotorInterface"), "Invalid joint state interface 1 type (expected: velocity)");
            return hardware_interface::CallbackReturn::ERROR;
        }

    }

    for (size_t i = 0; i < info_.joints.size(); i++) {
        motor_ids_[i] = (uint8_t)std::stoi(info_.joints[i].parameters["motor_id"]);
        kps_[i] = std::stof(info_.joints[i].parameters["kp"]); 
        kis_[i] = std::stof(info_.joints[i].parameters["ki"]); 
        kds_[i] = std::stof(info_.joints[i].parameters["kd"]); 
        is_reverse[i] = info_.joints[i].parameters["reverse"] == "true" ? -1.0 : 1.0;
        motors_.emplace_back(Motor(MOTOR_CHASSIS_ID_START + motor_ids_[i], &MOTOR_CHASSIS, MOTOR_CHASSIS_PARAMTER));
        motors_[i].setCoefficients(kps_[i], kis_[i], kds_[i], 0, 1000, 1);
        RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "%s mapped to motor %d", info_.joints[i].name.c_str(), motor_ids_[i]);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotorInterface::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Adding position state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]
            )
        );
    }
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Adding velocity state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]
            )
        );
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorInterface::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Adding velocity command interface: %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]
            )
        );
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn MotorInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Chassis motor hardware starting ...");

    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (std::isnan(position_states_[i])) {
            position_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_states_[i])) {
            velocity_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_commands_[i])) {
            velocity_commands_[i] = 0.0f;
        }
        velocity_commands_saved_[i] = velocity_commands_[i];
    }
    for (int i = 0; i < info_.joints.size(); i++) motors_[i].update();

    RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Chassis motor started");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Chassis motor stopping ...");

    motors_.clear();

    RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Chassis motor stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MotorInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    for (size_t i = 0; i < info_.joints.size(); i++){

        position_states_[i] = is_reverse[i] * motors_[i].getPosition();
        velocity_states_[i] = is_reverse[i] * motors_[i].getVelocity();
        // Important Debug
        // RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Got position %.5f, velocity %.5f for joint %ld!", position_states_[i], velocity_states_[i], i);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    for (size_t i = 0; i < info_.joints.size(); i++) {
        // Only send motor commands if the velocity changed
        if (velocity_commands_[i] != velocity_commands_saved_[i]) {

            // RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Motor velocity changed: %.5f", velocity_commands_[i]);
            // Send the motor command
            motors_[i].Setpoint = is_reverse[i] * velocity_commands_[i];

            // Store the current velocity
            velocity_commands_saved_[i] = velocity_commands_[i];
        }
    }
    for (size_t i = 0; i < info_.joints.size(); i++) motors_[i].update();
    return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(
    tinker_motor::MotorInterface,
    hardware_interface::SystemInterface
)
