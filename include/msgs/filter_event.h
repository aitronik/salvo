#pragma once

/**
 * @brief Class representing a filter event.
 * 
 * This class encapsulates information about a filter event, including its ID and whether it's active or not.
 */
class FilterEvent {
public:
    uint32_t filterEvent; /**< ID of the filter event. */
    bool active; /**< Flag indicating whether the filter event is active. */  
};
