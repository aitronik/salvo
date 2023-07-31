#pragma once

#include "Subject.h"
#include "LogItem.h"


class LogList : public Subject<std::shared_ptr<LogItem>> {
    
    public:
    std::list<std::shared_ptr<LogItem>> m_log;

    void addToList(std::shared_ptr<LogItem> log){
        m_log.push_back(log);
        // set the list
        notify(log);
    }
};