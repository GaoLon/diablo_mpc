/**
 * @file OSDK_Vehicle.hpp
 * @brief main interface to interact with robot
 */
#pragma once

#include "OSDK_Vehicle.hpp"
#include "OSDK_HAL.hpp"

#include "OSDK_Virtual_RC.hpp"
#include "OSDK_Movement.hpp"
#include "OSDK_Telemetry.hpp"
#include "OSDK_Bridge.hpp"

namespace DIABLO{
namespace OSDK{

class Vehicle
{
public: 
    Vehicle(HAL* hal): hal(hal), bridge(NULL), intra_bridge(true),
    movement_ctrl(NULL), 
    virtual_rc(NULL),
    telemetry(NULL)
    {}

    Vehicle(HAL* hal, Bridge* bridge): hal(hal), bridge(bridge), intra_bridge(false),
    movement_ctrl(NULL), 
    virtual_rc(NULL),
    telemetry(NULL)
    {}

    ~Vehicle()
    {
        if(movement_ctrl) delete movement_ctrl;
        if(virtual_rc)    delete virtual_rc;
        if(telemetry)     delete telemetry;
        if(bridge && intra_bridge)  delete bridge;
    }

    /**
     * @brief   Initialize SDK port to vehicle
     */
    uint8_t init(void);

public:
    HAL*               hal;
    Bridge*         bridge;
    
public:
    Movement_Ctrl*  movement_ctrl;
    Virtual_RC*     virtual_rc;
    Telemetry*      telemetry;

private:
    bool           intra_bridge;
};

}
}
