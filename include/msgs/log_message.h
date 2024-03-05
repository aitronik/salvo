#pragma once

#include <iostream>

/**
 * @brief Class representing a log message.
 * 
 * This class encapsulates information about a log message, including its timestamp, severity level,
 * source name, message content, file name, function name, and line number.
 */
class LogMessage{
public:
    uint64_t stamp; /**< Timestamp of the log message. */
    unsigned char level; /**< Severity level of the log message. */
    std::string name; /**< Source name of the log message. */
    std::string msg; /**< Content of the log message. */
    std::string file; /**< File name where the log message originated. */
    std::string function; /**< Function name where the log message originated. */
    std::string line; /**< Line number where the log message originated. */
};