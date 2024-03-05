#pragma once

#include <string>

/**
 * @brief Class representing a subscription choice.
 * 
 * This class encapsulates information about a subscription choice, including the chosen data and the type of subscription.
 */
class SubscriptionChoice {
public:
    /**
     * @brief Enumeration representing the type of subscription choice.
     */
    enum Choice {
        FILE = 0, /**< Subscription choice for file data. */
        PLAYBACK = 1, /**< Subscription choice for playback data. */
        ROS1 = 2, /**< Subscription choice for ROS 1 data. */
        ROS2 = 3, /**< Subscription choice for ROS 2 data. */
        ZMQ = 4 /**< Subscription choice for ZMQ data. */
    };

    std::string data; /**< Chosen data for subscription. */
    Choice type; /**< Type of subscription choice. */
};