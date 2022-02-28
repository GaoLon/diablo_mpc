/**
 * @file OSDK_CallbackManager.hpp
 * @brief Callback_Manager class, manage the issue of one callback thread
 */
#pragma once

#include <iostream>
#include <vector>
#include <deque>
#include <pthread.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "OSDK_Listener.hpp"
#include "OSDK_Broadcaster.hpp"

namespace DIABLO
{
namespace OSDK{

class Callback_Manager
{
public:
    Callback_Manager();
    Callback_Manager(std::string event_t);
    Callback_Manager(Broadcaster* broadcaster);
    Callback_Manager(Broadcaster* broadcaster, uint8_t stacklength);
    ~Callback_Manager();

    /**
     * @brief Set the broadcaster message queue length
     * @param[in] length length of th queue
     */
    void SetDequeLength(uint8_t length);

    /**
     * @brief Set the Broadcaster of the event
     * @param[in] broadcaster pointer to the broadcaster
     * @return true add braodcaster \n
     *         false error occur, see terminal output  
     */
    bool SetBroadcaster(Broadcaster* broadcaster);

    /**
     * @brief Add listener to the event
     * @param[in] listener pointer to the listener
     * @return true add listener \n 
     *         false error occur, see terminal output
     */
    bool AddListener(Listener* listener);
    
    /**
     * @brief remove boradcaster from the event
     */
    void RemoveBroadcaster();

    /**
     * @brief remove listener from the event
     * @param[in] listener pointer to the listener
     */
    void RemoveListener(Listener* listener);

    /**
     * @brief broadcaster send a new message
     * @param[in] msg pointer to the message
     */
    void NewMsgArrive(void* msg);

    /**
     * @brief send one message to the listener
     */
    void SendMsg();

    /**
     * @brief close the event thread
     */
    void CloseManager();

    /**
     * @brief Get the number of listener
     * @return number of the listener
     */
    uint32_t GetNumListener();

    /**
     * @brief start the callback thread
     */
    void Activate();

    /**
     * @brief start sending messages
     * @note NON-API FUNCTION
     */
    void StartManager(void);
private:
    Broadcaster* broadcaster;
    std::vector<Listener*> listenerlist;
    std::deque<void*> msg_deque;
    std::string event;
    uint8_t max_store_msg;
    bool working;

    boost::mutex msg_deque_mutex;
    boost::thread* callback_thd;
};

}
}