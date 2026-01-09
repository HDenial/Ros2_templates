#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp" 

using namespace std::chrono_literals;

class RobotPubNode : public rclcpp::Node // MODIFY NAME
{
public:
    // characteristics of the node
    RobotPubNode() : Node("robot_publisher") // MODIFY NAME
    {
        publisher_ = this->create_publisher<example_interfaces::msg::String>("News", 10); // MODIFY TOPIC NAME
        timer_= this->create_wall_timer(0.5s, std::bind(&RobotPubNode::publish_news, this)); // MODIFY TIME
        RCLCPP_INFO(this->get_logger(), "Robot Publisher has been started");
    }
 
private:
    void publish_news()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = "Hello, world!";
        publisher_->publish(msg);
    }
    //declaring variables
   rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
   rclcpp::TimerBase::SharedPtr timer_; 
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPubNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}