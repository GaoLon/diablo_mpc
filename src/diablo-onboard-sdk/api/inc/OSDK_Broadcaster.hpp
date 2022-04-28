#pragma once

#include <iostream>
#include <string>
#include <typeinfo>

// #include "OSDK_CallbackManager.hpp"

namespace DIABLO
{
namespace OSDK
{

class Callback_Manager;

class Broadcaster
{
public:
    Broadcaster(): valid(true) {}
    ~Broadcaster() {}

    template<class M>
    void InitBroadcaster(std::string event_t)
    {
        this->event = event_t;
        type_name = typeid(M).name();
    }

    void Broadcast(void* msg);

    void setManager(Callback_Manager* manager);
    std::string getEvent();
    uint32_t getNumListerner();
    void Close();

    bool isValid() {return valid;}
private:
    Callback_Manager* manager;
    std::string event;
    const char* type_name;
    bool valid;
};

}
}