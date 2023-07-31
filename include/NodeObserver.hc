#pragma once

#include "Observer.h"
#include "ActiveNodes.h"

class NodesObserver : public Observer{


    ActiveNodes* m_subject;
  public:
    NodesObserver(ActiveNodes &a){
        m_subject = &a;
        m_subject->attach(this);
    }
    void update(){
        std::shared_ptr<std::vector<NodeInfo>> result = m_subject->getState();
    }
};