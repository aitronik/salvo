#pragma once

#include <memory>
#include <chrono>
#include <iostream>
#include <functional>

#include "salvo/colors.h" 
#include <mutex>

#ifndef USE_CUSTOM_LEVELS
namespace salvo {   
    /**
     * @brief Enumeration representing severity levels for logging.
     */
    enum Severity : uint32_t {
        DEBUG = 10, /**< Debugging level. */
        INFO  = 20, /**< Information level. */
        WARN  = 30, /**< Warning level. */
        ERROR = 40, /**< Error level. */
        FATAL = 50 /**< Fatal error level. */
    };
}
#endif

/**
 * @brief Macro for printing log messages with a specified severity level.
 * 
 * This macro simplifies the logging process by providing a quick way to print log messages with a specified severity level.
 * It takes the severity level and the message as arguments.
 */
#define PRINT_LOG(level,msg) \
    salvo::Log::getInstance().printLog(level,msg, __FILE__,__func__, __LINE__);

/**
 * @brief Macro for printing log messages using stream operations with a specified severity level.
 * 
 * This macro simplifies the logging process by providing a quick way to print log messages using stream operations
 * with a specified severity level. It takes the severity level and the message as arguments.
 */
#define PRINT_STREAM(level,msg) \
    {std::stringstream ss; \
    ss << msg; \
    salvo::Log::getInstance().printLog(level, ss.str(), __FILE__,__func__, __LINE__);}

namespace salvo {

    /**
     * @brief Class for logging messages.
     * 
     * This class provides functionality for logging messages with specified severity levels.
     */
    class Log {
    private:
        std::function<void(Severity, std::string, std::string, std::string, uint32_t)> publish; /**< Function pointer to the actual publishing function. */
        std::mutex m_mutex; /**< Mutex to avoid multiple threads calling the publish function simultaneously. */

        /**
         * @brief Private constructor for Log class.
         * 
         */
        Log(){}

    public:
        Log(const Log& ) = delete; /**< Deleted copy constructor to prevent copying of Log objects. */
        void operator=(Log const&)  = delete; /**< Deleted copy assignment operator to prevent assignment of Log objects. */

        /**
         * @brief Get the singleton instance of the Log class.
         * 
         * @return Log& Reference to the singleton Log instance.
         */
        static Log& getInstance(){
            static Log instance;
            return instance;
        }

        /**
         * @brief Bind the printLog function to the actual publisher.
         * 
         * @param fun Function pointer to the actual publishing function.
         */
        void bindPrintLog(std::function<void(Severity, std::string, std::string, std::string, uint32_t)> fun){
            this->publish = fun;
        }

        /**
         * @brief Function called by the macro that will call the publish function to print log messages.
         * 
         * @param level Severity level of the log message.
         * @param msg Content of the log message.
         * @param file File name where the log message originated.
         * @param func Function name where the log message originated.
         * @param line Line number where the log message originated.
         */
        void printLog(Severity level, std::string msg, std::string file, std::string func, uint32_t line){
            if (publish == nullptr){
                std::cout << to_fg_red("WARNING! LOG DISABLED! you need to bind the printLog with your publisher") << std::endl;
                return;
            }
            m_mutex.lock();
            publish(level,msg,file,func,line);
            m_mutex.unlock();
        }
    };

}