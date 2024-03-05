#pragma once

#include <unordered_map>
#include <memory>

#include "core/subject.h"
#include "msgs/filter_event.h"
#include "msgs/filter_node.h"
#include "msgs/log_message.h"
#include "msgs/node_info.h"
#include "msgs/subscription_choice.h"

/**
 * @brief Abstract base class for GUI implementations.
 * 
 * This class defines the interface for GUI implementations
 * and provides methods for managing subscriptions and publishing events.
 */
class AbstractGui : public Observer<std::shared_ptr<LogMessage>>,
                    public Observer<std::list<std::shared_ptr<LogMessage>>*>, 
                    public Observer<NodeInfo> {

public:
    /**
     * @brief Constructor.
     * 
     * Initializes data structures and settings.
     */
    AbstractGui(){
        m_stringLevels.push_back("DEBUG");
        m_stringLevels.push_back("INFO");
        m_stringLevels.push_back("WARN");
        m_stringLevels.push_back("ERROR");
        m_stringLevels.push_back("FATAL");

        m_levelById.push_back(10);
        m_levelById.push_back(20);
        m_levelById.push_back(30);
        m_levelById.push_back(40);
        m_levelById.push_back(50);
    }
    
    /**
     * @brief Adds an observer for filter event updates.
     * 
     * @param obs Pointer to the observer for filter events.
     */
    void addFilterEventSubscriber(Observer<FilterEvent>* obs){
        m_filterEventPublisher.attach(obs);
    }

    /**
     * @brief Adds an observer for filter node updates.
     * 
     * @param obs Pointer to the observer for filter nodes.
     */
    void addFilterNodeSubscriber(Observer<FilterNode>* obs){
        m_filterNodePublisher.attach(obs);
    }

    /**
     * @brief Adds an observer for subscription choice updates.
     * 
     * @param obs Pointer to the observer for subscription choices.
     */
    void addSubscriptionChoiceSubscriber(Observer<SubscriptionChoice>* obs){
        m_subscriptionChoicePublisher.attach(obs);
    }

    /**
     * @brief Publishes a filter event.
     * 
     * @param eventId The ID of the event.
     * @param active Flag indicating whether the event is active.
     */
    void publishFilterEvent(int eventId, bool active){
        FilterEvent msg;
        msg.active = active;
        msg.filterEvent = m_levelById[eventId];

        m_filterEventPublisher.notify(msg);
    }

    /**
     * @brief Publishes a filter node.
     * 
     * @param nodeId The ID of the node.
     * @param active Flag indicating whether the node is active.
     */
    void publishFilterNode(int nodeId, bool active){
        FilterNode msg;
        msg.active = active;
        msg.node = m_names[nodeId];
        msg.selectAll = false;
        m_filterNodePublisher.notify(msg);
    }

    /**
     * @brief Publishes selection of all nodes.
     * 
     * @param active Flag indicating whether all nodes are selected.
     */
    void publishSelectAllNodes(bool active){
        FilterNode msg;
        msg.active = active;
        msg.node = "all";
        msg.selectAll = true;
        m_filterNodePublisher.notify(msg);
    }

    /**
     * @brief Publishes a subscription choice.
     * 
     * @param i The choice index.
     * @param data The data associated with the choice.
     */
    void publishSubscriptionChoice(int i,std::string data){
        SubscriptionChoice msg;
        msg.data = data;
        msg.type = (SubscriptionChoice::Choice)i;
        m_subscriptionChoicePublisher.notify(msg);
    }

    /**
     * @brief Updates the GUI with a list of log messages.
     * 
     * @param list Pointer to the list of log messages.
     */
    virtual void update(std::list<std::shared_ptr<LogMessage>>* list) override {
        std::list<std::string> listOfMessages;
        for (auto msg : *list){
            if (m_firstMessage) {
                m_t0 = msg->stamp;
                m_firstMessage = false;
            }
            std::string message = std::to_string((msg->stamp - m_t0) / 10e8) + "[" + msg->name + ", " + msg->function + "] : " + msg->msg;
            listOfMessages.push_back(message);
        }
        resetList(listOfMessages);
    }

    /**
     * @brief Updates the GUI with a single log message.
     * 
     * @param newMsg Pointer to the new log message.
     */
    virtual void update(std::shared_ptr<LogMessage> newMsg) override {
        if (m_firstMessage) {
            m_t0 = newMsg->stamp;
            m_firstMessage = false;
        }
        std::string message = std::to_string((newMsg->stamp - m_t0) / 10e8) + "[" + newMsg->name + ", " + newMsg->function + "] : " + newMsg->msg;
        updateLog(message);
    }

    /**
     * @brief Updates the GUI with node information.
     * 
     * @param msg The node information.
     */
    virtual void update(NodeInfo msg) override {
        bool newNode = false;
        auto it = m_nodes.find(msg.name);
        if (it == m_nodes.end()) {
            m_ids[msg.name] = m_nodes.size();
            m_names[m_nodes.size()] = msg.name;
            newNode = true;     
        }
        m_nodes[msg.name] = msg;

        std::string message;
        int r,g,b;
        if (msg.state == NodeInfo::status::ACTIVE){
            message = "[ACTIVE]: ";
            r = 0;
            g = 0;
            b = 0;
        }
        else if (msg.state == NodeInfo::status::WARNING) {
            message = "[WARNING]: ";
            r = 255;
            g = 165;
            b = 0;
        }
        else if (msg.state == NodeInfo::status::PING_LOST){
            message = "[PING_LOST]: ";
            r = 255;
            g = 0;
            b = 0;
        }
        else if (msg.state == NodeInfo::status::LOG){
            message = "[LOG]: ";
            r = 0;
            g = 0;
            b = 0;
        }
        message += msg.name;

        updateNodes(message,r,g,b,newNode,m_ids[msg.name] );
    }

    /**
     * @brief Retrieves the event string corresponding to the given ID.
     * 
     * @param i The event ID.
     * @return The event string.
     */
    std::string getEventStringbyId(int i) {
        return m_stringLevels[i];
    }
    
protected:
    Subject<FilterEvent> m_filterEventPublisher; /**< Publisher for filter event updates. */
    Subject<FilterNode>  m_filterNodePublisher; /**< Publisher for filter node updates. */
    Subject<SubscriptionChoice>  m_subscriptionChoicePublisher; /**< Publisher for subscription choice updates. */

    std::unordered_map<std::string, NodeInfo> m_nodes; /**< Map of node information. */
    std::unordered_map<std::string, int> m_ids; /**< Map of node IDs. */
    std::unordered_map<int, std::string> m_names; /**< Map of node names. */
    
    uint64_t m_t0; /**< Initial timestamp. */
    bool m_firstMessage = true; /**< Flag indicating the first message. */
    std::list<std::string> m_list; /**< List of log messages. */

    /**
     * @brief Pure virtual function to reset the list of log messages.
     * 
     * @param listOfMessages The updated list of log messages.
     */
    virtual void resetList(std::list<std::string>& listOfMessages) = 0;

    /**
     * @brief Pure virtual function to update the log.
     * 
     * @param message The log message to update.
     */
    virtual void updateLog(std::string message) = 0;

    /**
     * @brief Pure virtual function to update the nodes.
     * 
     * @param message The node message to update.
     * @param r The red component of the color.
     * @param g The green component of the color.
     * @param b The blue component of the color.
     * @param newNode Flag indicating whether it's a new node.
     * @param index The index of the node.
     */
    virtual void updateNodes(std::string message, int r, int g, int b, bool newNode, int index) = 0;

    std::vector<std::string> m_stringLevels; /**< Vector of log levels as strings. */
    std::vector<uint32_t>    m_levelById; /**< Vector of log levels as IDs. */
};