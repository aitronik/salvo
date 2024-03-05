#include "impl/ros2_subscriber.h"

#include <chrono>

Ros2Subscriber::Ros2Subscriber(int argc, char * argv[])   {
    m_isRunning = false;
    rclcpp::init(argc, argv);
    m_nh = rclcpp::Node::make_shared("Ros2SubscriberNode");

    m_nh->declare_parameter("redirect_log_to", "/rosout");
    std::string topic_name = m_nh->get_parameter("redirect_log_to").as_string();
    m_logSub  = m_nh->create_subscription<rcl_interfaces::msg::Log>(topic_name, 10000,
                    std::bind(&Ros2Subscriber::logCallback, this, std::placeholders::_1));
    m_pingSub = m_nh->create_subscription<std_msgs::msg::String>("/ping", 10000, 
                    std::bind(&Ros2Subscriber::pingCallback, this, std::placeholders::_1));
    
    m_spinThread = std::thread(&Ros2Subscriber::threadFun,this);
}
/****************************************************/
Ros2Subscriber::~Ros2Subscriber(){
    stop();
}
/****************************************************/
void Ros2Subscriber::stop(){
    if (m_isRunning){
        m_isRunning = false;
        rclcpp::shutdown();
    }
    spin();
}
/****************************************************/
void Ros2Subscriber::spin(){ 
    if (m_spinThread.joinable())
        m_spinThread.join();
}
/****************************************************/
void Ros2Subscriber::threadFun(){
    m_isRunning = true;
    rclcpp::spin(m_nh);
    if (m_isRunning){
        rclcpp::shutdown();
    }
    m_isRunning = false;
}
/****************************************************/
void Ros2Subscriber::logCallback(const rcl_interfaces::msg::Log::SharedPtr msg){


    std::shared_ptr<LogMessage>  message = std::make_shared<LogMessage>();

    message->name = msg->name;
    message->msg = msg->msg;
    message->file = msg->file;
    message->function = msg->function;
    message->line = std::to_string(msg->line);
    message->level = msg->level;
    message->stamp = rclcpp::Time(msg->stamp).nanoseconds();

    updateNodeInfo(msg->name);
    m_logPublisher.notify(message);
}
/****************************************************/
void Ros2Subscriber::pingCallback(const std_msgs::msg::String::SharedPtr msg){

    updateNodeInfo(msg->data);

   
}
 