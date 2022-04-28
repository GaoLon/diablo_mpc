/**
 * @file OSDK_Callback.hpp
 * @brief Singlrton CallBack_Handler class, manaage all callback threads
 */
#pragma once

#include <map>
#include <string>
#include <vector>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

#include "OSDK_Broadcaster.hpp"
#include "OSDK_Listener.hpp"
#include "OSDK_CallbackManager.hpp"

// typedef enum{
//     OSDK_DEFAULT_CALLBACK,
// }OSDK_Callback_t;

namespace DIABLO{
namespace OSDK
{
typedef void(*Callback_function)(void*);
    
class CallBack_Handler
{   
public:
    static CallBack_Handler& GetInstance()
    {
        static CallBack_Handler handler_instance;
        return handler_instance;
    }

    /**
     * @brief Remove the broadcaster from a certain event
     * @param[in] event_t event name
     * @return true: remove broadcaster \n 
     *         false: error occur, see terminal output \n
     */
    bool RemoveBroadcaster(std::string event_t);
    /**
     * @brief Remove a certain listener from a certain event
     * @param[in] event_t event name
     * @param[in] listener pointer to the listener to remove 
     * @return true: remove listener \n 
     *         false: error occur, see terminal output \n
     */
    bool RemoveListener(std::string event_t, Listener* listener);

    /**
     * @brief Add broadcaster to a certain event
     * @note If the event does not exist, it will create one
     * @tparam M datatype of the message sending to listener
     * @param[in] event_t event name
     * @param[in] deque_length max number of message that could be stored in broadcaster side
     * @return Broadcaster* pointer to the created broadcaster \n 
     *         NULL if error occur
     */
    template <typename M>
    Broadcaster* NewBroadcaster(std::string event_t, int deque_length);

    /**
     * @brief Add listener to a certain event
     * @note If the event does not exist, it will create one
     * @param[in] event_t event name
     * @param[in] length max number of message that could be stored in listener side
     * @param[in] somefunc pointer to user defined callback function
     * @return Listener* pointer to the created listener \n 
     *         NULL error occur
     */
    Listener* NewListener(std::string event_t, uint8_t length, Callback_function somefunc);

    /**
     * @brief Get the thread manager of a certain event
     * @param[in] event_t event name
     * @return Callback_Manager* pointer to the mananger \n
     *         NULL event does not exist
     */
    Callback_Manager* GetManager(std::string event_t);

    /**
     * @brief close a certain event
     * @param[in] event_t event name
     */
    void CloseEvent(std::string event_t);

    /**
     * @brief Get the number of listener of a certain event
     * @param[in] event_t event name
     * @return listener number
     */
    uint32_t GetNumListener(std::string event_t);
private:
    CallBack_Handler(void) {}
    ~CallBack_Handler();

private:
    std::map<std::string, Broadcaster*> Broadcaster_table;
    std::map<std::string, Callback_Manager*> Callbackmanager_table;
};

template <class M>
Broadcaster* CallBack_Handler::NewBroadcaster(std::string event_t, int deque_length)
{
    Callback_Manager* manager;
    if(Callbackmanager_table.find(event_t) == Callbackmanager_table.end())
    {
        manager = new Callback_Manager(event_t);
        Callbackmanager_table.insert(std::pair<std::string, Callback_Manager*>(event_t,manager));
    }
    else
    {
        manager = Callbackmanager_table.find(event_t)->second;
    }
    Broadcaster* newbroadcaster = new Broadcaster();
    newbroadcaster->template InitBroadcaster<M>(event_t);
    newbroadcaster->setManager(manager);
    if(!manager->SetBroadcaster(newbroadcaster))
    {
        newbroadcaster->~Broadcaster();
        printf("Fail to add broadcaster.\n");
        throw "Fail to add broadcaster.\n";
        //return NULL;
    }
    manager->SetDequeLength(deque_length);
    Broadcaster_table.insert(std::pair<std::string, Broadcaster*>(event_t, newbroadcaster));
    return newbroadcaster;
} 

}
}