#pragma once

#include <iostream>
#include <memory>
#include <list>
#include <unordered_map>
#include <vector>

#include "NodeItem.h"
#include "LogFilter.h"
#include "LogList.h"
#include "Observer.h" 
#include "Subject.h"

class LogManager : public Observer<std::shared_ptr<LogMessage>> {
    
    std::list<std::shared_ptr<LogItem>> m_completeLog;

    LogFilter  m_filter;
   
 
public: 

    // void listenTo(LogSubject &sub){
    //     m_subject = &sub;
    //     m_subject->attach(this);
    // }

    virtual void update(std::shared_ptr<LogMessage> msg){
   //     std::cout << "[LogManager] got a new LogMessage, save a ListItem!" << std::endl;
        std::shared_ptr<LogItem> newMsg(new LogItem(msg));

        m_completeLog.push_back(newMsg);
        std::list<std::shared_ptr<LogItem>>::iterator it = m_completeLog.end();
        --it;
        
        m_filter.check(it);

        // m_activeNodes.setState(newMsg);

        // if (newMsg.isSelected()){
        //     m_logList.setState(newMsg);
        // }


        // New message received. update the activeNode the node
   //     
    }

    void notifyLogListTo(Observer<std::shared_ptr<LogItem>>* obs) {
        m_filter.notifyLogListTo(obs);
    }
    void notifyNodeInfoTo(Observer<std::shared_ptr<NodeInfo>>* obs) {
        m_filter.attach(obs);
    }
 
};