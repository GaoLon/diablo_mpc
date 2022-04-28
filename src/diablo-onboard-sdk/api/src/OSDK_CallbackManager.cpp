#include <algorithm>
#include "OSDK_CallbackManager.hpp"
#include "OSDK_Broadcaster.hpp"

using namespace DIABLO::OSDK;

Callback_Manager::Callback_Manager(): max_store_msg((uint8_t)1), working(false), callback_thd(NULL), broadcaster(NULL)
{}

Callback_Manager::Callback_Manager(std::string event_t): max_store_msg((uint8_t)1), working(false), callback_thd(NULL), broadcaster(NULL)
{
    event = event_t;
}

Callback_Manager::Callback_Manager(Broadcaster* broadcaster): max_store_msg((uint8_t)1), working(false), 
                                                            callback_thd(NULL)
{this->broadcaster = broadcaster;}

Callback_Manager::Callback_Manager(Broadcaster* broadcaster, uint8_t stacklength): working(false), callback_thd(NULL) 
{this->broadcaster = broadcaster; max_store_msg = stacklength;}

Callback_Manager::~Callback_Manager()
{
    if(callback_thd)
    {
        pthread_cancel(callback_thd->native_handle());
        delete callback_thd;
        callback_thd = NULL;
    }
    listenerlist.clear();
}

void Callback_Manager::Activate()
{
    working = true;
    for(int i=0; i<listenerlist.size(); i++)
    {
        listenerlist[i]->Activate();
    }
    callback_thd = new boost::thread(boost::bind(&Callback_Manager::StartManager, this));
}

bool Callback_Manager::SetBroadcaster(Broadcaster* broadcaster)
{
    if(this->broadcaster == NULL)
    {
        this->broadcaster = broadcaster;
        return true;
    }
    else
    {
        printf("Error: Event already has a broadcaster.\n");
        return false;
    }
}

bool Callback_Manager::AddListener(Listener* listener)
{
    if(listener->getEvent().compare(event) != 0)
    {
        printf("Error: Add Listener to a wrong event.\n");
        return false;
    }
    if(find(listenerlist.begin(),listenerlist.end(),listener) != listenerlist.end())
    {
        printf("Error: Listener had already been registered.\n");
        return false;
    }
    listenerlist.push_back(listener);
    return true;
}

void Callback_Manager::RemoveBroadcaster()
{
    max_store_msg = 1;
    msg_deque.clear();
    broadcaster->~Broadcaster();
    broadcaster = NULL;
}

void Callback_Manager::RemoveListener(Listener* listener)
{
    if(listener->getEvent().compare(event) != 0)
    {
        printf("Error: Remove Listener from a wrong event.\n");
        return;
    }
    std::vector<Listener*>::iterator iter = find(listenerlist.begin(),listenerlist.end(),listener);
    if(iter == listenerlist.end())
    {
        printf("Error: Listener had already been removed.\n");
        return;
    }
    listenerlist.erase(iter);
}

void Callback_Manager::NewMsgArrive(void* msg)
{
    if(!working)
        return;
    if(msg_deque.size() >= max_store_msg)
    {
        //printf("message stack is full. Drop the very first message\n");
        boost::unique_lock<boost::mutex> lock(msg_deque_mutex);
        msg_deque.pop_front();
        msg_deque.push_back(msg);
        return;
    }
    else
    {
        boost::unique_lock<boost::mutex> lock(msg_deque_mutex);
        msg_deque.push_back(msg);
        return;
    }
}

void Callback_Manager::SendMsg()
{
    if(!working)
        return;
    if(listenerlist.size()==0)
    {
        return;
    }
    while(!msg_deque.empty())
    {
        //boost::unique_lock<boost::mutex> lock(msg_deque_mutex);
        msg_deque_mutex.lock();
        void* msg = msg_deque.back();
        msg_deque.pop_back();
        msg_deque_mutex.unlock();
        for(int i=0; i<listenerlist.size();i++)
        {
            listenerlist[i]->Listen(msg);
        }
    }
}

void Callback_Manager::CloseManager()
{
    working = false;
    if(broadcaster != NULL)
    {
        broadcaster->~Broadcaster();
        broadcaster = NULL;
    }
    for(int i=0; i<listenerlist.size(); i++)
    {
        listenerlist[i]->~Listener();
    }
    listenerlist.clear();
    msg_deque.clear();
    this->~Callback_Manager();
}

void Callback_Manager::SetDequeLength(uint8_t length)
{
    max_store_msg = length;
}

uint32_t Callback_Manager::GetNumListener()
{
    return listenerlist.size();
}

void Callback_Manager::StartManager(void)
{
    while(true)
    {
        SendMsg();
    }
}