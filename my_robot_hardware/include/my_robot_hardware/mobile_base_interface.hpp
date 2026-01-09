#ifndef MOBILE_BASE_INTERFACE_HPP
#define MOBILE_BASE_INTERFACE_HPP

#include <hardware_interface/hardware_interface/system_interface.hpp>
#include <my_robot_hardware/servo_driver.hpp>

namespace my_robot_hardware{

class MobileBaseInterface : public hardware_interface::SystemInterface
{
public:

    //LifecycleNode override

    hardware_interface::CallbackReturn on_configure(const rclcpp::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp::State & previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(const rclcpp::State & previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(const rclcpp::State & previous_state) override;

    // SystemInterface override

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::CallbackReturn write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    //drivers
    std::shared_ptr<SMS4306RDriver> driver_right_;
    std::shared_ptr<SMS4306RDriver> driver_left_; 

    //config pin params
    int left_motor_gpio_pin_;
    int right_motor_gpio_pin_;

    //states
    double left_motor_position_;
    double right_motor_position_;
    double left_motor_velocity_;
    double right_motor_velocity_;
    double left_motor_effort_;
    double right_motor_effort_;
    double command_angle_left_;
    double command_angle_right_;
    
    std::string port_;
};//class MobileBaseInterface

}//namespace my_robot_hardware


#endif  // MOBILE_BASE_INTERFACE_HPP_