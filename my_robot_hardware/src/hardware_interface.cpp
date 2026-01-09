#include <my_robot_hardware/mobile_base_interface.hpp>

namespace my_robot_hardware
{
    hardware_interface::CallbackReturn MobileBaseInterface::on_init
        (const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != 
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        info_ = info;
        // Use the passed parameters if available, otherwise use defaults
        left_motor_gpio_pin_  = info_.hardware_parameters.count("left_motor_gpio_pin") > 0 ?
        std::stoi(info_.hardware_parameters.at("left_motor_gpio_pin")) :
        13;  // Default: GPIO13 (pin 33 on Pi header)

        right_motor_gpio_pin_ = info_.hardware_parameters.count("right_motor_gpio_pin") > 0 ?
        std::stoi(info_.hardware_parameters.at("right_motor_gpio_pin")) :
        19;  // Example default: GPIO19 (pin 35 on Pi header) 

        port_ = info_.hardware_parameters.count("port") > 0 ?
        info_.hardware_parameters.at("port") :
        "";  // If you’re only using GPIO, port can default to empty

        driver_left_ = std::make_shared<SMS4306RDriver>(left_motor_gpio_pin_);
        driver_right_ = std::make_shared<SMS4306RDriver>(right_motor_gpio_pin_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn MobileBaseInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
            RCLCPP_INFO(rclcpp::get_logger("MobileBaseInterface"), "Configuring hardware...");

    // Initialize left motor
    if (driver_left_->init() < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("MobileBaseInterface"),
                     "Failed to initialize left motor on GPIO %d", left_motor_gpio_pin_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize right motor
    if (driver_right_->init() < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("MobileBaseInterface"),
                     "Failed to initialize right motor on GPIO %d", right_motor_gpio_pin_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Reset state variables
    left_motor_position_ = right_motor_position_ = 0.0;
    left_motor_velocity_ = right_motor_velocity_ = 0.0;
    left_motor_effort_   = right_motor_effort_   = 0.0;

    RCLCPP_INFO(rclcpp::get_logger("MobileBaseInterface"), "Hardware configured successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MobileBaseInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        driver_left_->activate();
        driver_right_->activate();
    }

    hardware_interface::CallbackReturn MobileBaseInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        driver_left_->deactivate();
        driver_right_->deactivate();

        RCLCPP_INFO(rclcpp::get_logger("MobileBaseInterface"), "Hardware deactivated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MobileBaseInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
    {
            
    }

    hardware_interface::CallbackReturn MobileBaseInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
    {
    driver_left_->setTargetPositionDegree(command_angle_left_);
    driver_right_->setTargetPositionDegree(command_angle_right_);

    }
}