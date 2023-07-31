#pragma once

#include <list>
#include "Observer.h"

template <class T>
class Subject {
    
    std::list<Observer<T>*> m_observers;

public:
    void attach(Observer<T>* obs){
        if (obs != nullptr){
            m_observers.push_back(obs);
        }
    }

protected:
    void notify(T msg) {
        for(auto it : m_observers){
            it->update(msg);
        }
    }
};
