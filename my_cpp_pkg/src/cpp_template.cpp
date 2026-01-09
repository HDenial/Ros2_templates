#include "rclcpp/rclcpp.hpp"
     
class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
    // characteristics of the node
    MyCustomNode() : Node("node_name") // MODIFY NAME
    {
        //code here
    }
 
private:
    //private functions here
    //{
        //code here
    //}
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