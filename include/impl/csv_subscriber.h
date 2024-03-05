#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <list>
#include <thread>

#include <string>
#include <fstream>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <sys/stat.h>
#include <chrono>

#include "core/data_subscriber.h"

/**
 * @brief A subscriber class for reading log data from a CSV file.
 * 
 * This class extends the functionality of DataSubscriber to read log data from a CSV file.
 */
class CsvSubscriber : public DataSubscriber {
    
    bool m_isSpinThreadRunning; /**< Flag indicating whether the spin thread is running. */
    std::thread m_spinThread; /**< Thread for spinning and reading log data. */
    std::string m_path; /**< Path to the CSV log file. */
    bool m_isValid; /**< Flag indicating whether the file path is valid. */
    bool m_playback; /**< Flag indicating whether to playback log data. */

public:
    /**
     * @brief Constructor for CsvSubscriber.
     * 
     * @param path The path to the CSV log file.
     * @param playback Flag indicating whether to playback log data.
     */
    CsvSubscriber(std::string path, bool playback) : DataSubscriber(false){
        m_isSpinThreadRunning = false;
        m_path = "";
        m_isValid = false;
        m_playback = playback; 
        readLogFile(path);   
    }

    /**
     * @brief Destructor for CsvSubscriber.
     * 
     * Stops the spin thread if running.
     */
    ~CsvSubscriber(){
        m_isSpinThreadRunning = false;
        spin();
    }

    /**
     * @brief Stops the spin thread if running.
     */
    void spin(){ 
        if (m_spinThread.joinable()){
            m_spinThread.join();
        }
    }

    /**
     * @brief Starts the spin thread to read log data from the CSV file.
     */
    void start(){
        if (m_isValid){
            m_spinThread = std::thread(&CsvSubscriber::threadFun,this);
        }
    }

private:
    /**
     * @brief Reads the CSV log file and validates the path.
     * 
     * @param path The path to the CSV log file.
     */
    void readLogFile(std::string path){
        struct stat sb;
        if (stat(path.c_str(), &sb) == 0 && !(sb.st_mode & S_IFDIR)){
            std::cout << "The path is valid!" << std::endl;
            m_path = path;
            m_isValid = true;
        }
        else {
            std::cout << "The Path is invalid!"<< std::endl; 
            m_isValid = false;
        }
    }

    /**
     * @brief Function executed by the spin thread to read log data from the CSV file.
     */
    void threadFun(){
        m_isSpinThreadRunning = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::ifstream myFile(m_path);

        std::string line;
        std::shared_ptr<LogMessage> message;
        uint64_t timestampPublishMessage = 0;

        if(myFile.good())
        {
            std::getline(myFile, line);
            while(std::getline(myFile, line) && m_isSpinThreadRunning)
            {
                std::shared_ptr<LogMessage>  message = std::make_shared<LogMessage>();
                std::istringstream ss(line);
                std::string val;
                const char delimiter  = 0x60;

                std::getline(ss, val, delimiter);
                message->stamp = std::stoull(val);
                std::getline(ss, val, delimiter);
                message->level = std::stoul(val);       
                std::getline(ss, val, delimiter);
                message->name = val;
                std::getline(ss, val, delimiter);
                message->msg  = val;
                std::getline(ss, val, delimiter);
                message->file = val;
                std::getline(ss, val, delimiter);
                message->function = val;
                std::getline(ss, val, delimiter);
                message->line = val;
                
                if (timestampPublishMessage == 0){
                    timestampPublishMessage = message->stamp;
                }

                while (m_playback && timestampPublishMessage < message->stamp){
                    timestampPublishMessage+= 10000000;
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                m_logPublisher.notify(message);
                updateNodeInfo(message->name, true);
            } 
        }
        myFile.close();
        m_isSpinThreadRunning = false;
        return;
    }
};
