#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"
using namespace std::chrono_literals; 
     
class HwStatusNode : public rclcpp::Node // MODIFY NAME
{
    public:
    // characteristics of the node
    HwStatusNode() : Node("robot_publisher") // MODIFY NAME
    {
        publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hw_status", 10); // MODIFY TOPIC NAME
        timer_= this->create_wall_timer(0.5s, std::bind(&HwStatusNode::publish_news, this)); // MODIFY TIME
        RCLCPP_INFO(this->get_logger(), "Robot Publisher has been started");
    }
 
    private:
        void publish_news()
        {
            auto msg = my_robot_interfaces::msg::HardwareStatus();
            msg.temperature = 43.0;
            msg.are_motors_read = true;
            msg.status = "OK";
            publisher_->publish(msg);
        }
    //declaring variables
rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
rclcpp::TimerBase::SharedPtr timer_; 
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HwStatusNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}