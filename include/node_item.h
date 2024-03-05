#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <list>

#include "log_item.h"
#include "core/subject.h"
#include "msgs/node_info.h"

typedef std::list<std::shared_ptr<LogItem>> listOfMessages;

class NodeItem  {
private:
    std::unordered_map<uint32_t,listOfMessages> m_logs;
    std::unordered_map<uint32_t, bool>  m_activeLevels;

    bool active;
 public:
    NodeItem(){
        active = true;
     }
    //void setPing(uint64_t lastPing){}
    void update(std::shared_ptr<LogItem> newMsg){

        uint32_t msgLevel = newMsg->msg->level;

        if (m_logs.find(msgLevel) == m_logs.end()){
           m_logs[msgLevel]         = std::list<std::shared_ptr<LogItem>>();
           m_activeLevels[msgLevel] = true;
        }
        
        m_logs[msgLevel].push_back(newMsg);
               
    }
 
    void setVisibility(bool action){
        if (active != action){
            active = action;
            for (auto& listOfIterators : m_logs){
                for (auto msg : listOfIterators.second){
                    msg->show = action && getLevelVisibility(listOfIterators.first);
                }
            }
        }   
    }
    bool getVisibility(){
        return active;
    }

    void setLevelVisibility(uint32_t level, bool action){

        if (m_activeLevels[level] != action){
            m_activeLevels[level]= action;
            
            auto listOfIterators = m_logs[level];
            for (auto msg : listOfIterators){
                msg->show = active && action;
            }
        }
    }
    bool getLevelVisibility(uint32_t level){
        return m_activeLevels[level];
    }
    
    bool isActive() {return active; }
      

};