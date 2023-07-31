#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <list>

#include "LogItem.h"
#include "LogLevel.h"
#include "Subject.h"
#include "NodeInfo.h"

class NodeItem  {
    
private:
    std::unordered_map<unsigned char,std::list<std::list<std::shared_ptr<LogItem>>::iterator>> m_logs;
    uint64_t m_lastPingTimestamp;
    std::shared_ptr<LogItem> m_lastReceivedLog;

    bool show;
    bool highlight;
public:
    NodeItem(){
        show = true;
        highlight = true;
    }
    void setPing(uint64_t lastPing){}
    void update(std::list<std::shared_ptr<LogItem>>::iterator newMsg){

        unsigned char msgLevel = (*newMsg)->msg->level ;

        // if (m_logs.find(msgLevel) == m_logs.end()){
        //    m_logs[msgLevel] = std::list<std::list<std::shared_ptr<LogItem>>();
        // }

        
        m_logs[msgLevel].push_back(newMsg);
        m_lastReceivedLog = *newMsg;
        
               
    }
    void getInfo(std::shared_ptr<NodeInfo>& info){
        info->name     = m_lastReceivedLog->msg->name;
        info->lastMsg  = m_lastReceivedLog->msg->msg;
        info->pingInfo = m_lastReceivedLog->msg->stamp;
    }

    bool hasLostPing(uint64_t timeNow){}
    bool hasTimedOut(uint64_t timeNow){}
    bool getShowFlag() {return show; }
    bool getHighlightFlag() {return highlight; }
     

};