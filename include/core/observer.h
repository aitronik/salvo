#pragma once

/**
 * @brief Observer base class template.
 * 
 * This class template defines an interface for observers
 * that need to receive updates of a certain type of message.
 * 
 * @tparam T The type of message that observers will receive updates for.
 */
template <class T>
class Observer {
  public:
    /**
     * @brief Virtual function to be implemented by concrete observers.
     * 
     * This function is called by the subject to notify the observer
     * about an update with a new message of type T.
     * 
     * @param msg The message being sent to the observer.
     */
    virtual void update(T msg) = 0;
};