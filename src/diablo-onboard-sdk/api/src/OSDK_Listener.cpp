#include "OSDK_Listener.hpp"

using namespace DIABLO::OSDK;

void Listener::InitListener(uint8_t length, std::string event_t, Callback_function somefunc)
{
    max_store_msg = length;
    event = event_t;
    callback_function = somefunc;
}

void Listener::Activate()
{
    listener_thd = new boost::thread(boost::bind(&Listener::CallFunc, this));
}

std::string Listener::getEvent()
{
    return event;
}

void Listener::Listen(void* in_msg)
{
    if(msg_deque.size() >= max_store_msg)
    {
        //printf("message stack is full. Drop the very first message\n");
        boost::unique_lock<boost::mutex> lock(msg_mutex);
        msg_deque.pop_front();
        msg_deque.push_back(in_msg);
    }
    else
    {
        boost::unique_lock<boost::mutex> lock(msg_mutex);
        msg_deque.push_back(in_msg);
    }
}

void Listener::CallFunc()
{
    while(true)
    {
        if(!msg_deque.empty())
        {
            //boost::unique_lock<boost::mutex> lock(msg_mutex);
            msg_mutex.lock();
            void* msg = msg_deque.back();
            msg_deque.pop_back();
            msg_mutex.unlock();
            callback_function(msg);
        }
    }
}