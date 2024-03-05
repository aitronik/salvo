#pragma once

#include <memory>
#include "msgs/log_message.h"

/**
 * @brief Class representing a log item.
 * 
 * This class holds a shared pointer to a LogMessage object and a flag indicating whether the log item should be shown.
 */
class LogItem {
public:
    std::shared_ptr<LogMessage> msg; /**< Shared pointer to a LogMessage object. */
    bool show; /**< Flag indicating whether the log item should be shown. */

    /**
     * @brief Default constructor for LogItem.
     */
    LogItem() {}

    /**
     * @brief Constructor for LogItem.
     * 
     * Constructs a LogItem object with the specified LogMessage.
     * 
     * @param msg Shared pointer to a LogMessage object.
     */
    LogItem(std::shared_ptr<LogMessage> msg) {
        this->msg = msg;
        show = true;
    }
};