#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("CPP_Test"), counter_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Hello World");
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MyNode::timerCallback, this)); // Create a timer that calls timerCallback every second
  }
private:
  void timerCallback()
  {
    RCLCPP_INFO(this->get_logger(), "%d seconds have passed", counter_ );
    counter_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>(); //Create a node of the class MyNode"
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}