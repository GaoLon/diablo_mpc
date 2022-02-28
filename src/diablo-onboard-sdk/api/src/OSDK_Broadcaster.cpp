#include "OSDK_Broadcaster.hpp"
#include "OSDK_CallbackManager.hpp"

using namespace DIABLO::OSDK;

void Broadcaster::setManager(Callback_Manager* manager)
{
    this->manager = manager;
}

void Broadcaster::Broadcast(void* msg)
{
    manager->NewMsgArrive(msg);
}

std::string Broadcaster::getEvent()
{
    return event;
}

uint32_t Broadcaster::getNumListerner()
{
    return manager->GetNumListener();
}