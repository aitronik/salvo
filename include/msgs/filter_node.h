#pragma once

#include <string>

/**
 * @brief Class representing a filter node.
 * 
 * This class encapsulates information about a filter node, including its name, whether it's active or not,
 * and whether it's selected for all nodes or not.
 */
class FilterNode {
public:
    std::string node; /**< Name of the filter node. */
    bool active; /**< Flag indicating whether the filter node is active. */  
    bool selectAll; /**< Flag indicating whether the filter node is selected for all nodes. */
};