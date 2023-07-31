#pragma once

#include <list>
#include <memory>
#include <vector>
#include <unordered_map>

#include "NodeInfo.h"
#include "NodeItem.h"
//#include "Observer.h"
#include "Subject.h"
#include "LogLevel.h"
#include "LogList.h"
#include "LogItem.h"


class LogFilter : public Subject<std::shared_ptr<NodeInfo>> {
    
    std::unordered_map<std::string, NodeItem>  m_activeNodes;
    bool m_activeLevels[LogLevel::numLevels];

    LogList m_filteredLog;

    public:

    LogFilter(){
        for (int i = 0 ; i < LogLevel::numLevels; i++){
            m_activeLevels[i] = true;
        }
    }

    void check(std::list<std::shared_ptr<LogItem>>::iterator& newMsg){
        
        if (m_activeNodes.find((*newMsg)->msg->name) == m_activeNodes.end()){
           m_activeNodes[(*newMsg)->msg->name] = NodeItem();
        }

        bool show      = m_activeNodes[(*newMsg)->msg->name].getShowFlag();
        bool highlight = m_activeNodes[(*newMsg)->msg->name].getHighlightFlag();
        bool msgLevel  = (*newMsg)->msg->level ;
        
        (*newMsg)->show      = show;
        (*newMsg)->highlight = highlight;

        m_activeNodes[(*newMsg)->msg->name].update(newMsg);

        std::shared_ptr<NodeInfo> info = std::make_shared<NodeInfo>();
        m_activeNodes[(*newMsg)->msg->name].getInfo(info);

        notify(info); 

        if (m_activeLevels[msgLevel] ){
            if(show){
                m_filteredLog.addToList((*newMsg));
            }
        }
    }
    void notifyLogListTo(Observer<std::shared_ptr<LogItem>>* obs) {
        m_filteredLog.attach(obs);
    }
};
