#pragma once

#include "salvo/log.h"
#include "rcl_interfaces/msg/log.hpp"
#include "std_msgs/msg/string.hpp"

// #include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/**
 * @brief This class is meant to be used both with rclcpp::Node and rclcpp_lifecycle::LifeCycleNode.
 * We need to provide both the Node type and the publisher type. For example, 
 * Case 1: <rclcpp::Node,rclcpp::Publisher>
 * Case 2: <rclcpp_lifecycle::LifecycleNode,rclcpp_lifecycle::LifecyclePublisher>
 * We define the second class template as a template template parameters, otherwise is not possible to
 * get in the proper way the shared pointer.
 * More info at:
 * https://en.cppreference.com/w/cpp/language/template_parameters#Template_template_parameter
 * https://stackoverflow.com/questions/610245/where-and-why-do-i-have-to-put-the-template-and-typename-keywords/613132#613132
 * 
 * In our case we also need to declare PublisherTraits. Also. Whenever we need to use this class we also need to use
 * the "template" keyword before the function. This is seen in the create_publisher funciton:
 * m_logPublisher  = nh->template create_publisher<rcl_interfaces::msg::Log>(topic_name, 10);
 * Note that the word template and the space after (just before create_publisher) are meant to be there for this reason.
 * 
 * A drawback of this implementation is that we need to pay attention when we want to activate the publisher of the 
 * lifecycle node. the function activate works on lifecycles only. 
 * 
 * @tparam genericNode 
 * @tparam genericPublisher 
 */
template<class genericNode,template <typename> class genericPublisher>
class Ros2Log {
// 
// we need to declare a template template parameter for the publisher.
  public:
    
    /** Log Sender qos */
    const rclcpp::QoS m_qosLogSend = rclcpp::QoS(10)
        .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    template <typename MsgT>
    struct PublisherTraits {
        using SharedPtr = typename genericPublisher<MsgT>::SharedPtr;
    };

    rclcpp::TimerBase::SharedPtr m_pingTimer;
    typename PublisherTraits<rcl_interfaces::msg::Log>::SharedPtr m_logPublisher;
    typename PublisherTraits<std_msgs::msg::String>::SharedPtr m_pingPublisher;
 
    std_msgs::msg::String m_pingString;
    std::string m_nodeName;
    genericNode* m_nh;
     
  
    static Ros2Log& init(genericNode* nh, std::string name){

        static Ros2Log instance(nh,name);
        return instance;
    }
    void activate(){
        m_logPublisher->on_activate();
        m_pingPublisher->on_activate();
    }
  private:
    Ros2Log(){}
    
    Ros2Log(genericNode* nh, std::string name){
   
        m_nh = nh;
        nh->declare_parameter("redirect_log_to", "/rosout");
        std::string topic_name = nh->get_parameter("redirect_log_to").as_string();
 
 
        m_logPublisher  = nh->template create_publisher<rcl_interfaces::msg::Log>(topic_name, m_qosLogSend);
        m_pingPublisher = nh->template create_publisher<std_msgs::msg::String>("/ping", m_qosLogSend);
        m_nodeName = name;
        m_pingString.data = name;
        m_pingTimer = nh->create_wall_timer(2000ms, std::bind(&Ros2Log::timerCallback, this));
   
        // initializing logInstance with an instance
        salvo::Log& logInstance = salvo::Log::getInstance();
        logInstance.bindPrintLog(std::bind(&Ros2Log::printLogToRos2,this,
                                                                std::placeholders::_1,
                                                                std::placeholders::_2,
                                                                std::placeholders::_3,
                                                                std::placeholders::_4,
                                                                std::placeholders::_5));
    }
    Ros2Log(Ros2Log const&);        // Don't forget to disable copy
    void operator=(Ros2Log const&);   // Don't forget to disable copy

    void timerCallback(){
        m_pingPublisher->publish(m_pingString);
    }


    /*-------------------------------------------------------------------------*/
    void printLogToRos2(salvo::Severity level, std::string msg, std::string file, std::string funcName, uint32_t line){

        switch (level){
            case (salvo::Severity::ERROR):
            case (salvo::Severity::FATAL):
                std::cout<<to_bg_red("["+funcName+"]");
                std::cout<<to_fg_red(" "+msg)<<std::endl;
                break;
            case (salvo::Severity::DEBUG):
                std::cout<<to_bg_blue("["+funcName+"]");
                std::cout<<to_fg_blue(" "+msg)<<std::endl;
                break;
            case (salvo::Severity::WARN):
                std::cout<<to_bg_magenta("["+funcName+"]");
                std::cout<<to_fg_magenta(" "+msg)<<std::endl;
                break;
            // case (salvo::Severity::EVENT):
            //     std::cout<<to_bg_green("["+funcName+"]");
            //     std::cout<<to_fg_green(" "+msg)<<std::endl;
            //     break;    
            default:
                std::cout<<"["<<funcName<<"] "<<msg<<std::endl;
                break;
        }

        rcl_interfaces::msg::Log logMsg;

        logMsg.stamp    = m_nh->now();
        logMsg.level    = level;
        logMsg.name     = m_nodeName;
        logMsg.msg      = msg;
        logMsg.file     = file;
        logMsg.function = funcName;
        logMsg.line     = line;

        m_logPublisher->publish(logMsg);

    }
};