#include <algorithm>
#include "OSDK_Callback.hpp"

using namespace DIABLO::OSDK;

CallBack_Handler::~CallBack_Handler()
{
    Broadcaster_table.clear();
    Callbackmanager_table.clear();
}

bool CallBack_Handler::RemoveBroadcaster(std::string event_t)
{
    if(Callbackmanager_table.find(event_t) == Callbackmanager_table.end())
    {
        printf("ERROR: Event does not exist 1.\n");
        return false;
    }
    if(Broadcaster_table.find(event_t) == Broadcaster_table.end())
    {
        printf("ERROR: Event does not have a broadcaster.\n");
        return false;
    }
    Callbackmanager_table.find(event_t)->second->RemoveBroadcaster();
    Broadcaster_table.erase(Broadcaster_table.find(event_t));
    return true;
}

bool CallBack_Handler::RemoveListener(std::string event_t, Listener* listener)
{
    if(Callbackmanager_table.find(event_t) == Callbackmanager_table.end())
    {
        printf("ERROR: Event does not exist 2.\n");
        return false;
    }
    Callbackmanager_table.find(event_t)->second->RemoveListener(listener);
    listener->~Listener();
    return true;
}


Listener* CallBack_Handler::NewListener(std::string event_t, uint8_t length, Callback_function somefunc)
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
    Listener* newlistener = new Listener();
    newlistener->InitListener(length, event_t, somefunc);
    if(!manager->AddListener(newlistener))
    {
        printf("Fail to Add Listener.\n");
        throw "Fail to Add Listener.\n";
    }
    return newlistener;
}

void CallBack_Handler::CloseEvent(std::string event_t)
{
    std::map<std::string, Callback_Manager*>::iterator iter = Callbackmanager_table.find(event_t);
    if(iter == Callbackmanager_table.end())
    {
        return;
    }
    iter->second->CloseManager();
    Callbackmanager_table.erase(iter);
}

Callback_Manager* CallBack_Handler::GetManager(std::string event_t)
{
    return Callbackmanager_table.find(event_t)->second;
}    

uint32_t CallBack_Handler::GetNumListener(std::string event_t)
{
    std::map<std::string, Callback_Manager*>::iterator iter = Callbackmanager_table.find(event_t);
    if(iter == Callbackmanager_table.end())
        return 0;
    else if(iter->second == NULL)
        return 0;
    else
        return iter->second->GetNumListener();
}