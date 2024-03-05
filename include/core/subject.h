#pragma once

#include <list>
#include "core/observer.h"

/**
 * @brief Subject class template.
 * 
 * This class template represents a subject that observers can attach to
 * in order to receive updates about changes in its state.
 * 
 * @tparam T The type of message that observers will receive updates for.
 */
template <class T>
class Subject {
    
    std::list<Observer<T>*> m_observers; /**< List of observers attached to the subject. */

public:
    /**
     * @brief Attaches an observer to the subject.
     * 
     * @param obs Pointer to the observer to be attached.
     */
    void attach(Observer<T>* obs){
        if (obs != nullptr){
            m_observers.push_back(obs);
        }
    }

    /**
     * @brief Detaches an observer from the subject.
     * 
     * @param obs Pointer to the observer to be detached.
     */
    void detach(Observer<T>* obs){
        for(auto it = m_observers.begin(); it != m_observers.end(); ++it){
            if (*it == obs){
                m_observers.erase(it);
                break;
            }
        }  
    }

    /**
     * @brief Detaches all observers from the subject.
     */
    void detachAll(){
        m_observers.clear();  
    }

    /**
     * @brief Notifies all attached observers about an update.
     * 
     * This function notifies all attached observers about a change in
     * the subject's state by calling the update function of each observer.
     * 
     * @param msg The message being sent to the observers.
     */
    void notify(T msg) {
        for(auto it : m_observers){
            it->update(msg);
        }
    }
};