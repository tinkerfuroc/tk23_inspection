#ifndef __TINKER_MOTOR_HARDWARE__MOTOR_HARDWARE_H__
#define __TINKER_MOTOR_HARDWARE__MOTOR_HARDWARE_H__

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>
#include <vector>
#include "motor_interface/motor.hpp"
namespace tinker_motor
{
     class MotorInterface : public hardware_interface::SystemInterface
     {
        private:
            std::vector<uint8_t> motor_ids_;
            std::vector<double> kps_, kis_, kds_;
            std::vector<double> position_states_;
            std::vector<double> velocity_states_;
            std::vector<double> velocity_commands_;
            std::vector<double> velocity_commands_saved_;
            std::vector<Motor> motors_;
            std::vector<double> is_reverse;
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(MotorInterface)

            virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
                
            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
                
            virtual hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
                
            virtual hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
                
            virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
                
            virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
     };
     

     
}











#endif