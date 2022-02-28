#pragma once

#include <stdio.h>
#include <iostream>

#ifdef _WITH_ROS    
    #include <ros/ros.h>
    //#include "DIABLO_OSDK_msgs/Motors.h"
#endif

#include "Onboard_SDK_Uart_Protocol.h"

class DDJ_M15
{
public:
    DDJ_M15():rev(0), enc_pos(0),
        pos(0), vel(0), iq(0){} 

    double pos;
    double vel;
    double iq;

    int    rev;
    double enc_pos;

    /**
     * @brief update serial 
     * @note  NON-API FUNCTION
     */
    void update_serial(OSDK_Push_Data_M15_t& data);
};

class Leg_Motors{
public:
    DDJ_M15 left_hip;
    DDJ_M15 left_knee;
    DDJ_M15 left_wheel;
    DDJ_M15 right_hip;
    DDJ_M15 right_knee;
    DDJ_M15 right_wheel;
};