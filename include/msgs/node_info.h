#pragma once
#include <string>

/**
 * @brief Class representing information about a node.
 * 
 * This class encapsulates information about a node, including its name, timestamp of the last ping,
 * last message received, and current state.
 */
class NodeInfo {
public:
    /**
     * @brief Enumeration representing the state of the node.
     */
    enum status{
        ACTIVE, /**< Node is active. */
        WARNING, /**< Node is in a warning state. */
        PING_LOST, /**< Node has lost ping communication. */
        LOG /**< Node is logging messages. */
    };

    std::string name; /**< Name of the node. */
    uint64_t pingInfo; /**< Timestamp of the last ping received from the node. */
    std::string lastMsg; /**< Last message received from the node. */
    status state; /**< Current state of the node. */
};