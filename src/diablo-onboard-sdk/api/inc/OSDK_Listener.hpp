#pragma once

#include <iostream>
#include <string>
#include <deque>
#include <pthread.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace DIABLO{
namespace OSDK{

class Listener;

typedef void(*Callback_function)(void*);

class Listener
{
public:
    Listener(): valid(true), listener_thd(NULL), max_store_msg((uint8_t)1) {};
    ~Listener() 
    {
        if(listener_thd)
        {
            pthread_cancel(listener_thd->native_handle());
            delete listener_thd;
            listener_thd = NULL;
        }
    }

    void InitListener(uint8_t length, std::string event_t, Callback_function somefunc);
    void Activate();

    std::string getEvent();
    void Listen(void* in_msg);
    void CallFunc();
private:
    std::string event;
    bool valid;
    std::deque<void*> msg_deque;
    uint8_t max_store_msg;

    Callback_function callback_function;
    
    boost::mutex msg_mutex;
    boost::thread* listener_thd;
};

}
}