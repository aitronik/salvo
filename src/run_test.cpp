
#include <iostream>
#include "impl/ros2_subscriber.h"
#include "salvo/ros2_log.h"
#include <rclcpp/rclcpp.hpp> 

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    std::string node_name = "test";
    rclcpp::Node::SharedPtr m_nh = rclcpp::Node::make_shared(node_name);

    Ros2Log<rclcpp::Node,rclcpp::Publisher>::init(m_nh.get(),node_name);
    
    PRINT_LOG(salvo::Severity::INFO, "Init Complete");
    
    
    rclcpp::shutdown();

     
    return 0;
}