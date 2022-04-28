#include "OSDK_Vehicle.hpp"

using namespace DIABLO::OSDK;

/**
 * @brief   Initialize SDK port to vehicle
 */
uint8_t Vehicle::init(void)
{
    if(intra_bridge)
    {
        #if defined (_WITH_ROS)
        bridge = new ROS_Bridge(this);
        #else
        bridge = new Dummy_Bridge(this);
        #endif
        if(!bridge)
        {
            std::cerr<<"Failed to allocate memory for Bridge!\n"<<std::endl;
            return 1;
        }
    }

    movement_ctrl = new Movement_Ctrl(this);
    if(!movement_ctrl)
    {
        std::cerr<<"Failed to allocate memory for Movement_Ctrl!\n"<<std::endl;
        return 1;
    }

    virtual_rc = new Virtual_RC(this);
    if(!virtual_rc)
    {
        std::cerr<<"Failed to allocate memory for Virtual RC!\n"<<std::endl;
        return 1;
    }

    telemetry = new Telemetry(this);
    if(!telemetry)
    {
        std::cerr<<"Failed to allocate memory for Telemetry!\n"<<std::endl;
        return 1;
    }

    return 0;
}