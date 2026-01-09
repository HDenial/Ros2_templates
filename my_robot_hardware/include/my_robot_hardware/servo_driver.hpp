#ifndef SMS4306R_DRIVER_HPP
#define SMS4306R_DRIVER_HPP

#include <iostream>
#include <pigpio.h>
#include <cmath>

// Constants for the SM-S4306R servo
constexpr int MIN_PULSE_WIDTH = 1000;  // Corresponds to 0 degrees
constexpr int MAX_PULSE_WIDTH = 2000; // Corresponds to 360 degrees
constexpr double DEGREE_RANGE = 360.0;

class SMS4306RDriver {
public:
    explicit SMS4306RDriver(int gpio_pin)
        : gpio_pin_(gpio_pin), is_initialized_(false) {}

    ~SMS4306RDriver() {
        if (is_initialized_) {
            deactivate();
            gpioTerminate();
        }
    }

    inline int init() {
        std::cout << "Initializing connection with servo." << std::endl;
        if (gpioInitialise() < 0) {
            std::cerr << "Failed to initialize pigpio!" << std::endl;
            return -1;
        }
        is_initialized_ = true;
        std::cout << "pigpio initialized successfully." << std::endl;
        return 0;
    }

    inline void activate() {
        std::cout << "Activating servo on GPIO pin " << gpio_pin_ << std::endl;

        gpioServo(gpio_pin_, 1500); // Set to neutral position (1500us pulse width)
    }

    inline void deactivate() {
        std::cout << "Deactivating servo on GPIO pin " << gpio_pin_ << std::endl;
        if (is_initialized_) {
            gpioServo(gpio_pin_, 0); // Stop sending servo pulses
        }
    }

    inline void setTargetPositionDegree(double angle) {
        if (!is_initialized_) {
            std::cerr << "Driver not initialized!" << std::endl;
            return;
        }

        // Clamp the angle to the valid range (0-360 degrees)
        if (angle < 0.0) {
            angle = 0.0;
        } else if (angle > DEGREE_RANGE) {
            angle = DEGREE_RANGE;
        }

        // Map the angle to a pulse width
        int pulse_width = static_cast<int>(MIN_PULSE_WIDTH + (angle / DEGREE_RANGE) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH));

        gpioServo(gpio_pin_, pulse_width);
    }

private:
    int gpio_pin_;
    bool is_initialized_;
};

#endif // SMS4306R_DRIVER_HPP

