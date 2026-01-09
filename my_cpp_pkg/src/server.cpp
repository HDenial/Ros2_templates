#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp" // MODIFY NAME
     
class CServerNode : public rclcpp::Node // MODIFY NAME
{
public:
    // characteristics of the node
    CServerNode() : Node("SumServer") // MODIFY NAME
    {
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "NumServer", // MODIFY NAME
            std::bind(&CServerNode::Callbackadd, this, std::placeholders::_1, std::placeholders::_2)); // creates a service called "add_two_ints"
            RCLCPP_INFO(this->get_logger(), "Service server is ready to add two integers");
    }  
    // creates a service called "NumServer" 
    // the service will call the Callbackadd function when a request is received
 
private:
    void Callbackadd(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request, 
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Incoming request: a=%ld b=%ld", request->a, request->b);
        response->sum = request->a + request->b; // recieve the request and add the two integers
        RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", response->sum);
    }
   rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;  
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CServerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}