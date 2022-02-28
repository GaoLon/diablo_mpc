#include <ros/ros.h>
#include <std_msgs/String.h>

#include "OSDK_Vehicle.hpp"
#include "diablo_sdk/Diablo_Ctrl.h"
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>

#define head_control_mode 1
#define height_control_mode 1
#define pitch_control_mode 1

DIABLO::OSDK::Movement_Ctrl* pMovementCtrl;

void teleop_ctrl(const diablo_sdk::Diablo_CtrlConstPtr& msg)
{
    if(!pMovementCtrl->in_control())
    {
        printf("to get movement ctrl.\n");
        uint8_t result = pMovementCtrl->obtain_control();
        return;
    }

    
    if (fabs(msg->speed) > 1.5)
    {
        pMovementCtrl->ctrl_data.forward = msg->speed>0?1.5:-1.5;
    }
    else
    {
        pMovementCtrl->ctrl_data.forward = msg->speed;
    }

    if (fabs(msg->omega) > 2.4)
    {
        pMovementCtrl->ctrl_data.left = msg->omega>0?2.4:-2.4;
    }
    else
    {
        pMovementCtrl->ctrl_data.left = msg->omega;
    }

    if (fabs(msg->roll) > 0.15)
    {
        pMovementCtrl->ctrl_data.roll = msg->roll>0?0.15:-0.15;
    }
    else
    {
        pMovementCtrl->ctrl_data.roll = msg->roll;
    }

    if (pitch_control_mode == 1)
    {
        if (fabs(msg->pitch) > 0.9424778)
        {
            pMovementCtrl->ctrl_data.pitch = msg->pitch>0?0.9424778:-0.9424778;
        }
        else
        {
            pMovementCtrl->ctrl_data.pitch = msg->pitch;
        }
    }
    else
    {
        if (fabs(msg->pitch_vel) > 1.5)
        {
            pMovementCtrl->ctrl_data.pitch = msg->pitch_vel>0?1.5:-1.5;
        }
        else
        {
            pMovementCtrl->ctrl_data.pitch = msg->pitch_vel;
        }
    }

    if (height_control_mode == 1)
    {
        if (msg->height > 1.0)
        {
            pMovementCtrl->ctrl_data.up = 1.0;
        }
        else if (msg->height < 0.0)
        {
            pMovementCtrl->ctrl_data.up = 0.0;
        }
        else
        {
            pMovementCtrl->ctrl_data.up = msg->height;
        }
    }
    else
    {
        if (msg->height_vel > 0.2)
        {
            pMovementCtrl->ctrl_data.up = msg->height_vel>0?0.2:-0.2;
        }
        else
        {
            pMovementCtrl->ctrl_data.up = msg->height_vel;
        }
    }

    pMovementCtrl->SendMovementCtrlCmd();

    return;
}

void init_control_mode()
{
    if(!pMovementCtrl->in_control())
    {
        printf("to get movement ctrl.\n");
        uint8_t result = pMovementCtrl->obtain_control();
        return;
    }

    pMovementCtrl->ctrl_mode_cmd = true;
    // head control mode: 0: auto move to keep balance, 1: keep angle with ground
    pMovementCtrl->ctrl_mode_data.head_controller_mode = head_control_mode;
    // height control mode: 0: speed command, 1: position command
    pMovementCtrl->ctrl_mode_data.height_ctrl_mode = height_control_mode;
    // pitch control mode: 0: speed command, 1: position command
    pMovementCtrl->ctrl_mode_data.pitch_ctrl_mode = pitch_control_mode;

    pMovementCtrl->SendMovementModeCtrlCmd();
    
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diablo_ctrl");
    ros::NodeHandle nh("~");

    DIABLO::OSDK::HAL_Serial Hal;                                                //Initialize HAL driver
    Hal.initSerial("/dev/ttyUSB0", 115200);                               //Initialize serial port

    DIABLO::OSDK::Vehicle vehicle(&Hal);                                  //Initialize Onboard SDK
    if(vehicle.init()) return -1;

    printf("%d\n",sizeof(OSDK_Uart_Header_t));

    vehicle.telemetry->activate();
    // vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_1Hz);
    //vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_100Hz);
    // vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_100Hz);
    // vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_100Hz);
    // vehicle.telemetry->configUpdate(); 
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_MOTOR);
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_QUATERNION);
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_ACCL);
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_GYRO);

    pMovementCtrl = vehicle.movement_ctrl;
    init_control_mode();
    ros::Subscriber sub = nh.subscribe("/diablo_cmd", 1, teleop_ctrl); //subscribe to ROS topic

    ros::spin();

    return 0;
}