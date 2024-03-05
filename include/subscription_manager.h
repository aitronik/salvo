#pragma once
 
#include <memory> 
#include <unordered_map>
#include <thread>
#include <mutex>

#include "core/observer.h"
#include "msgs/subscription_choice.h"
#include "impl/csv_subscriber.h"
#include "impl/ros2_subscriber.h"

class SubscriptionManager : public Observer<SubscriptionChoice> {
public:
    // void addLogMessageSubscriber(Observer<std::shared_ptr<LogMessage>>* obs){

    // }
    // void addNodeInfoSubscriber(Observer<NodeInfo>* obs){

    // }
    SubscriptionManager(int argc, char * argv[]){
        m_argc = argc;
        m_argv = argv;
        m_initialized = false;
        data0 = nullptr;
        data2 = nullptr;
    }
    ~SubscriptionManager(){
        if (data0 != nullptr){
            delete data0;
        }
        if (data2 != nullptr){
            delete data2;
        }

    }

    void notifyLogMessageTo(Observer<std::shared_ptr<LogMessage>>* obs) {
        m_LogMessageObs = obs;
    }
    void notifyNodeInfoTo(Observer<NodeInfo>* obs){
        m_nodeInfoObs = obs;
    }

    

private:
    Observer<std::shared_ptr<LogMessage>>* m_LogMessageObs;
    Observer<NodeInfo>* m_nodeInfoObs;

    bool m_initialized;
    int   m_argc;
    char** m_argv;
    
    CsvSubscriber* data0;
    Ros2Subscriber* data2;

    virtual void update(SubscriptionChoice newMsg) {

        std::cout << " callback SubscriptionChoice" << std::endl;

        // if (m_initialized){
        //     return;
        // }

        switch (newMsg.type)
        {
        case SubscriptionChoice::FILE:
        {   
            if (data0 != nullptr){
                delete data0;
                data0 = nullptr;
            }
            if (data2 != nullptr){
                data2->stop();
                delete data2;
                data2 = nullptr;
            }

            std::cout << " creating CSV" << std::endl;
            data0 = new CsvSubscriber(newMsg.data,false);
            data0->addLogMessageSubscriber(m_LogMessageObs);
            data0->addNodeInfoSubscriber(m_nodeInfoObs);
            data0->start();
            m_initialized = true;
            /* code */
            break;
        }
        case SubscriptionChoice::PLAYBACK:
        {   
            if (data0 != nullptr){
                delete data0;
                data0 = nullptr;
            }
            if (data2 != nullptr){
                data2->stop();
                delete data2;
                data2 = nullptr;
            }

            std::cout << " creating CSV" << std::endl;
            data0 = new CsvSubscriber(newMsg.data,true);
            data0->addLogMessageSubscriber(m_LogMessageObs);
            data0->addNodeInfoSubscriber(m_nodeInfoObs);
            data0->start();
            m_initialized = true;
            /* code */
            break;
        }
        case SubscriptionChoice::ROS1:
        {    /* code */
            break;
        }
        case SubscriptionChoice::ROS2:
        {
            if (data0 != nullptr){
                delete data0;
                data0 = nullptr;
            }
            if (data2 != nullptr){
                data2->stop();
                delete data2;
                data2 = nullptr;
            }


            std::cout << " creating ROS2" << std::endl;
            data2 = new Ros2Subscriber(m_argc,m_argv);
            data2->addLogMessageSubscriber(m_LogMessageObs);
            data2->addNodeInfoSubscriber(m_nodeInfoObs);
            m_initialized = true;
            /* code */
            break;
        }
        case SubscriptionChoice::ZMQ:
        {    /* code */
            break;
        }
        default:
            break;
        }
    }
};
 