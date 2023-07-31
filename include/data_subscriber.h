#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <list>
#include <thread>

#include "LogMessage.h"

#include <rclcpp/rclcpp.hpp> 
#include <rcl_interfaces/msg/log.hpp>


#include "Subject.h"

class DataSubscriber : public Subject<std::shared_ptr<LogMessage>> {
 
  std::thread m_spinThread;
  public:
    DataSubscriber()   {
        m_isRunning = false;
        rclcpp::init(0, nullptr);
        m_nh = rclcpp::Node::make_shared("DataSubscriberNode");
        m_sub = m_nh->create_subscription<rcl_interfaces::msg::Log>("/rosout", 10000, std::bind(&DataSubscriber::callback, this, std::placeholders::_1));
        
        m_spinThread = std::thread(&DataSubscriber::threadFun,this);
    }
    ~DataSubscriber(){
        
    }

    virtual std::shared_ptr<LogMessage> getState(){
        return message;
    }
    bool isRunning(){
        return m_isRunning;
    }
 private:
    rclcpp::Node::SharedPtr m_nh;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr m_sub;
    std::shared_ptr<LogMessage> message;
    bool m_isRunning;
    void threadFun(){
        m_isRunning = true;
        rclcpp::spin(m_nh);
        rclcpp::shutdown();
        m_isRunning = false;
        m_spinThread.join();
    }
 
    void callback(const rcl_interfaces::msg::Log::SharedPtr msg){
   //     std::cout << "[ROS] received a message from rosout " << std::endl;
        message = std::make_shared<LogMessage>();

        message->name = msg->name;
        message->msg = msg->msg;
        message->file = msg->file;
        message->function = msg->function;
        message->line = msg->line;
        message->level = msg->level;
        message->stamp = rclcpp::Time(msg->stamp).nanoseconds();
   
        notify(message);
    }
   
};
 