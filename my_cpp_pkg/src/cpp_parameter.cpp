#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
    // characteristics of the node
    MyCustomNode() : Node("node_name") // MODIFY NAME
    {
        
        this->declare_parameter("param_name", "<default_value>"); // MODIFY NAME
        variable_ = this->get_parameter("param_name").as_int(); // MODIFY NAME
        
        this->declare_parameter("timer_period",1.0);
        double timer_period = this->get_parameter("timer_period").as_double();
        
            number_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period),
            std::bind(&MyCustomNode::timer_callback, this)); // MODIFY NAME
    }

 
private:
    //private functions here
    
    void timer_callback()
    {
        auto msg = example_interfaces::msg::Int64(); // MODIFY NAME
        msg.data = variable_; // MODIFY NAME
    } 
   
    int variable_;
    rclcpp::TimerBase::SharedPtr number_timer_;
    // MODIFY NAME
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

//dont forget to add exec on CMakeLists.txt