#pragma once

#include <string>
#include <thread> 

#include <rclcpp/rclcpp.hpp> 
#include <rcl_interfaces/msg/log.hpp>
#include <std_msgs/msg/string.hpp>

#include "core/data_subscriber.h"

/**
 * @brief Ros2 Subscriber to ping and rosout.
 * 
 * This class extends DataSubscriber and acts as a subscriber for ROS 2 messages, 
 * including ping and rosout messages. It utilizes the Observer-Subject design pattern
 * to publish received messages.
 */
class Ros2Subscriber : public DataSubscriber {
 
public:
    /**
     * @brief Construct a new Ros2Subscriber object.
     * 
     * @param argc The number of command line arguments.
     * @param argv The command line arguments.
     */
    Ros2Subscriber(int argc, char * argv[]);

    /**
     * @brief Destroy the Ros 2 Subscriber object.
     */
    ~Ros2Subscriber();

    /**
     * @brief Spins the Ros2Subscriber, waiting for threads to exit.
     */
    void spin();

    /**
     * @brief Stops the executing threads.
     */
    void stop();

private:
    std::thread m_spinThread; /**< Thread where the rclcpp::spin is executed. */
    bool m_isRunning; /**< Variable for checking if the thread is running. */

    rclcpp::Node::SharedPtr m_nh; /**< Pointer to the ROS 2 node. */
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr m_logSub; /**< Subscription for rosout messages. */
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_pingSub; /**< Subscription for ping messages. */
    
    /**
     * @brief Function executed by the spin thread.
     */
    void threadFun();

    /**
     * @brief Callback function for rosout messages.
     * 
     * @param msg The received rosout message.
     */
    void logCallback(const rcl_interfaces::msg::Log::SharedPtr msg);

    /**
     * @brief Callback function for ping messages.
     * 
     * @param msg The received ping message.
     */
    void pingCallback(const std_msgs::msg::String::SharedPtr msg);
};