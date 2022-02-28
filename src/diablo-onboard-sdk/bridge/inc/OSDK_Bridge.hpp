/**
 * @file OSDK_Bridge.hpp
 * @brief Bridge class, send received packet to third party library
 */
#pragma once

#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <string.h>

#include "Onboard_SDK_Uart_Protocol.h"
#include "OSDK_Telemetry.hpp"

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#if defined (_WITH_ROS)
#include <ros/ros.h>
#include "diablo_sdk/OSDK_ACCL.h"
#include "diablo_sdk/OSDK_GYRO.h"
#include "diablo_sdk/OSDK_LEGMOTORS.h"
#include "diablo_sdk/OSDK_POWER.h"
#include "diablo_sdk/OSDK_QUATERNION.h"
#include "diablo_sdk/OSDK_RC.h"
#include "diablo_sdk/OSDK_STATUS.h"
#endif


namespace DIABLO{
namespace OSDK{

// Forward Declaration
class Vehicle;

/**
 * @brief Abstract bridge class
 */
class Bridge
{
    //friend class Telemetry;

public:
    Bridge(Vehicle* vehicle) : vehicle(vehicle), bridge_thd(NULL) {}
    ~Bridge() 
    {
        if(bridge_thd)
        {
            pthread_cancel(bridge_thd->native_handle());
            delete bridge_thd;
            bridge_thd = NULL;
        }
    }

    /**
     * @brief send data to third party library
     */
    virtual void BridgeSend(void) = 0;

protected:
    Vehicle* vehicle;

    boost::thread*  bridge_thd;
    boost::mutex    bridge_mtx;
};

#if defined (_WITH_ROS)
/**
 * @brief ROS Bridge, create when ros is used in the project
 */
class ROS_Bridge : public Bridge
{
public:
    ROS_Bridge(Vehicle* vehicle) : Bridge(vehicle) { }
    ~ROS_Bridge() {}

    /**
     * @brief Set the Publisher object
     * 
     * @param[in] nh ros::NodeHandle, to generate ros publishers
     */
    void setPublisher(ros::NodeHandle& nh);

    /**
     * @brief Publishers publish received data 
     */
    void BridgeSend(void) override;

    /**
     * @brief Start Bridge thread 
     */
    void ActivateBridge();

private:
    ros::Publisher ACCLPublisher;
    ros::Publisher GYROPublisher;
    ros::Publisher LEGMOTORSPublisher;
    ros::Publisher POWERPublisher;
    ros::Publisher QUATERNIONPublisher;
    ros::Publisher RCPublisher;
    ros::Publisher STATUSPublisher;
};
#endif

/**
 * @brief Dummy bridge, create when no need to link third party library
 */
class Dummy_Bridge : public Bridge
{
public:
    Dummy_Bridge(Vehicle* vehicle) : Bridge(vehicle) {}
    ~Dummy_Bridge() {}

    /**
     * @brief send data to third party library
     */
    void BridgeSend(void) override;
};

}
}