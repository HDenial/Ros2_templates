#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp" 

using namespace std::chrono_literals;
     
class CClientNode : public rclcpp::Node 
{
public:
    // characteristics of the node
    CClientNode() : Node("CClient") 
    {
     client_ = this->create_client<example_interfaces::srv::AddTwoInts>("NumServer"); // creates a client to the server : "NumServer"
    }

    void call_service(int a, int b)
    {
        while(!client_->wait_for_service(1s)) // wait for the service to be available
        {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        }
        request_ = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request_->a = a; // set the value of a
        request_->b = b; // set the value of b

        client_->async_send_request(request_, std::bind(&CClientNode::callbackadd, this, std::placeholders::_1)); // send the request to the server
    }
 
private:
    void callbackadd(
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
        auto response = future.get(); // get the response from the server
        RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %ld", response->sum); // print the result
    }
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CClientNode>(); // MODIFY NAME
    node->call_service(2,3); // call the service with the values 2 and 3
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}