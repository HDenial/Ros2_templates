#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp" 
     
class RobotListenerNode : public rclcpp::Node // MODIFY NAME
{
public:
    // characteristics of the node
    RobotListenerNode() : Node("robot_listener") // MODIFY NAME
    {
        listener_ = this->create_subscription<example_interfaces::msg::String>(
            "News", 10, std::bind(&RobotListenerNode::listener_callback, this, std::placeholders::_1)); // MODIFY TOPIC NAME and add new placeholders for every argument on callback
        RCLCPP_INFO(this->get_logger(), "Robot Listener has been started");
    }
 
private:
    //private functions here
    void listener_callback(const example_interfaces::msg::String::SharedPtr msg) // MODIFY TOPIC NAME
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }
    //declaring variables
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr listener_; // MODIFY TOPIC NAME
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotListenerNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}