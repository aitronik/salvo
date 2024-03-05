#pragma once

#include <memory> 
#include <unordered_map>
#include <thread>
#include <mutex>

#include "salvo/log.h"
#include "msgs/log_message.h"
#include "msgs/node_info.h"
#include "core/subject.h"

/**
 * @brief Class representing a data subscriber.
 * 
 * This class subscribes to data updates and provides methods
 * to add subscribers for different types of data.
 */
class DataSubscriber {
public:
    /**
     * @brief Adds an observer for log message updates.
     * 
     * @param obs Pointer to the observer for log messages.
     */
    void addLogMessageSubscriber(Observer<std::shared_ptr<LogMessage>>* obs){
        m_logPublisher.attach(obs);
    }

    /**
     * @brief Adds an observer for node information updates.
     * 
     * @param obs Pointer to the observer for node information.
     */
    void addNodeInfoSubscriber(Observer<NodeInfo>* obs){
        m_nodeInfoPublisher.attach(obs);
    }

    /**
     * @brief Constructor.
     * 
     * @param checkPing Flag indicating whether to run the ping thread.
     */
    DataSubscriber(bool checkPing = true){
        m_pingThreadRunning = checkPing ;
        if (checkPing){
            m_pingThread = std::thread(&DataSubscriber::threadFun,this);
        }
    }

    /**
     * @brief Destructor.
     * 
     * Cleans up resources and stops the ping thread if running.
     */
    ~DataSubscriber(){
        if (m_pingThreadRunning){
            m_pingThreadRunning = false;
            if (m_pingThread.joinable()) 
                m_pingThread.join();
        }
        m_logPublisher.detachAll();
        m_nodeInfoPublisher.detachAll();
    }

private:
    // Thread variables
    bool m_pingThreadRunning; /**< Flag indicating whether the ping thread is running. */
    std::thread m_pingThread; /**< Thread for monitoring node pings. */
    std::mutex m_nodeInfoMutex; /**< Mutex for thread synchronization. */
    std::unordered_map<std::string,uint64_t> m_nodePings; /**< Map to store node ping information. */

    /**
     * @brief Thread function for monitoring node pings.
     * 
     * This function runs in a separate thread and monitors the
     * ping status of nodes. It sends notifications for warning
     * and ping loss events.
     */
    void threadFun(){
        while (m_pingThreadRunning){
            uint64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            m_nodeInfoMutex.lock();

            NodeInfo infoSalvo;
            infoSalvo.name    = "salvo";
            infoSalvo.pingInfo = now;
            infoSalvo.state = NodeInfo::status::ACTIVE;
                            
            m_nodeInfoPublisher.notify(infoSalvo); 

            for (auto &it : m_nodePings){
                if (it.second != UINT64_MAX){
                    float timeElapsed = (now - it.second )/10e8;

                    if (timeElapsed >2.5 ){
                        std::shared_ptr<LogMessage>  message = std::make_shared<LogMessage>();
                        message->name     = "salvo";
                        message->msg      = it.first +  ": " + std::to_string(timeElapsed) + "s elapsed since last ping";
                        message->file     =__FILE__;
                        message->function = "PingCheck";
                        message->line     = __LINE__;
                        message->level    = salvo::Severity::WARN;
                        message->stamp    = now;

                        m_logPublisher.notify(message);
                      

                        NodeInfo info;
                        info.name    = it.first;
                        info.pingInfo = now;
                        info.state = NodeInfo::status::WARNING;

                        if (timeElapsed >= 10.0){
                            info.state = NodeInfo::status::PING_LOST;
                            it.second = UINT64_MAX;
                        }                            
                        m_nodeInfoPublisher.notify(info); 
                    }
                }
            }
            m_nodeInfoMutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

protected:
    Subject<std::shared_ptr<LogMessage>>  m_logPublisher; /**< Subject for log message updates. */
    Subject<NodeInfo>    m_nodeInfoPublisher; /**< Subject for node information updates. */

    /**
     * @brief Updates node information.
     * 
     * This function updates the ping status of a node and notifies
     * subscribers about the changes.
     * 
     * @param data The name of the node.
     * @param offlineLog Flag indicating whether it's an offline log.
     */
    void updateNodeInfo(std::string data, bool offlineLog = false){
        NodeInfo nodeInfoMsg;
        nodeInfoMsg.name  = data;
        uint64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        nodeInfoMsg.pingInfo = now;
        if (offlineLog){
            nodeInfoMsg.state = NodeInfo::LOG;
        }
        else {
            nodeInfoMsg.state = NodeInfo::ACTIVE;
        }
    
        m_nodeInfoMutex.lock();
        m_nodePings[nodeInfoMsg.name] = nodeInfoMsg.pingInfo;
        m_nodeInfoPublisher.notify(nodeInfoMsg);
        m_nodeInfoMutex.unlock();
    }
};