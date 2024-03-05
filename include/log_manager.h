#pragma once

#include <iostream>
#include <memory>
#include <list>

#include <vector>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "node_item.h"
#include "log_item.h"

#include "msgs/node_info.h"
#include "msgs/filter_event.h"
#include "msgs/filter_node.h"

#include "core/subject.h"
#include "core/observer.h" 
#include "core/data_subscriber.h"

class LogManager : public Observer<std::shared_ptr<LogMessage>> ,
                   public Observer<FilterNode>,
                   public Observer<FilterEvent> {
    
    Subject<std::shared_ptr<LogMessage>>              m_logPublisher;
    Subject<std::list<std::shared_ptr<LogMessage>>*>  m_listPublisher;
   
    std::unordered_map<std::string, NodeItem>         m_activeNodes;
    std::list<std::shared_ptr<LogMessage>>            m_filteredLog;
    std::list<std::shared_ptr<LogItem>>               m_completeLog;
    std::mutex m_logMutex;

    void check(std::shared_ptr<LogMessage>& msg){
         
        //get name
        const std::string name = msg->name;
        // get the current level of the message
        uint32_t msgLevel  = msg->level;

        // Do I have already this node? if not, add it to the active nodes 
        if (m_activeNodes.find(name) == m_activeNodes.end()){
            m_activeNodes[name] = NodeItem();
        }

        bool show = m_activeNodes[name].getLevelVisibility(msgLevel) && m_activeNodes[name].getVisibility();
          // store the message
        std::shared_ptr<LogItem> newMsg(new LogItem(msg));
        newMsg->show = show;
    
        m_activeNodes[name].update(newMsg);
        m_logMutex.lock();
        m_completeLog.push_back(newMsg);
        // if the new message is allowed to be visualized, then notify to the gui
        if (show){
            m_filteredLog.push_back(msg);
            // set the list
            m_logPublisher.notify(msg);          
        }
        m_logMutex.unlock();
    }
    
    void resetFilteredLog(){
        m_logMutex.lock();
        m_filteredLog.clear();
        for (auto& logItem : m_completeLog){
            if (logItem->show){
                m_filteredLog.push_back(logItem->msg);
            }
        }
        m_listPublisher.notify(&m_filteredLog);
        m_logMutex.unlock();
    }

public: 
  
    void listenTo(DataSubscriber &data){
        data.addLogMessageSubscriber(this);
        }
    
    virtual void update(std::shared_ptr<LogMessage> msg){
    
        check(msg);
 
    }
    void update(FilterEvent f){
        //std::cout << "got a click from the gui" << f.active << ", " << f.filterEvent << std::endl;
        for (auto& node : m_activeNodes){
            node.second.setLevelVisibility(f.filterEvent, f.active);
        }

        resetFilteredLog();
    }
    void update(FilterNode f){
        //std::cout << "got a click from the gui" << f.active << ", " << f.node << std::endl;
        if (f.selectAll){
            for (auto &n : m_activeNodes) {
                n.second.setVisibility(f.active);
            }
        }
        else{
            m_activeNodes[f.node].setVisibility(f.active);
        }

        resetFilteredLog();
    }

    void notifyLogListTo(Observer<std::list<std::shared_ptr<LogMessage>>*>* obs) {
        m_listPublisher.attach(obs);
    }

    void notifyLogMessageTo(Observer<std::shared_ptr<LogMessage>>* obs) {
        m_logPublisher.attach(obs);
    }
    void listenToGui(Subject<FilterEvent> *sub){
        sub->attach(this);
    }
 
};