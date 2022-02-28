#include <ros/ros.h>
#include <std_msgs/String.h>

#include "OSDK_Vehicle.hpp"
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>

int dj_mode = 0;
int height = 100;

DIABLO::OSDK::Virtual_RC* pVirtualRC;

/**
 * @brief A ROS callback function that listen to keyboard key command topic and send 
 *        SDK virtual RC command accordingly
 * @note  This is an example of how to initialize and form an SDK virtual RC command  
 */
void teleop_callBack(const std_msgs::String::ConstPtr& cmd)
{   
    if(!pVirtualRC->in_control()) //check control status
    {
        printf("to get ctr.\n");
        uint8_t result = pVirtualRC->obtain_control();
        return;
    }

    pVirtualRC->reset_data();
    int forward_cmd = 0, left_cmd = 0, up_cmd = 0, tilt_cmd = 0;
    for(const char& c : cmd->data)
    {
        switch(c)
        {
            case 'w':
                forward_cmd += 1; break;
            case 's':
                forward_cmd -= 1; break;
            case 'a':
                left_cmd += 1;    break;
            case 'd':
                left_cmd -= 1;    break;
            case 'f':
                up_cmd += 1;      break;
            case 'g':
                up_cmd -= 1;      break;
            case 'e':
                tilt_cmd += 1;    break;
            case 'q':
                tilt_cmd -= 1;    break;
            case 'r':
                pVirtualRC->data.ch14 = 1;      //jump
                break;
            case 'c':
                if(dj_mode == 0)
                {
                    dj_mode = 1;                // switch to sport mode, higher speed and enable jump
                }
                break;
            case 'v':
                if(dj_mode == 1) 
                {
                    dj_mode = 0;                // switch to demo mode, lower speed and disable jump
                }
                break;
            case 'z':
                if(height < 500)
                    height = 600;               // transform up
                break;
            case 'x':
                if(height > 500)
                    height = 100;               // transform down
                break;
        }
    }

    pVirtualRC->data.ch1  = -left_cmd * 400 + 1000;     //need to check the sign of command, similar to SBUS protocol
    pVirtualRC->data.ch2  = -forward_cmd * 400 + 1000;
    pVirtualRC->data.ch4  =  tilt_cmd * 400 + 1000;
    pVirtualRC->data.ch5  =  up_cmd * 800 + 1000;
    pVirtualRC->data.ch15 =  dj_mode;                         //set DIABLO to be in sport mode, enable jumping
    pVirtualRC->data.ch9  =  height;


    uint8_t result = pVirtualRC->SendVirtualRCCmd();                     //send Virtual RC command over serial port
    //printf("send result: %u\n",result);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_rc_example");
    ros::NodeHandle nh("~");

#if defined(_WITH_SERIAL)
    DIABLO::OSDK::HAL_Serial Hal;                                                //Initialize HAL driver
    Hal.initSerial("/dev/ttyUSB0", 460800);                               //Initialize serial port
#elif defined(_WITH_PI)
    DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init()) return -1;
#endif

    DIABLO::OSDK::Vehicle vehicle(&Hal);                                  //Initialize Onboard SDK
    if(vehicle.init()) return -1;

    printf("%d\n",sizeof(OSDK_Uart_Header_t));

    vehicle.telemetry->activate();
    //vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_POWER, OSDK_PUSH_DATA_10Hz);
    //vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_100Hz);
    //vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_100Hz);
    //vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_100Hz);
    vehicle.telemetry->configUpdate(); 
    vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_STATUS);
    // vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_QUATERNION);
    // vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_ACCL);
    // vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_GYRO);
    

    pVirtualRC = vehicle.virtual_rc;
    ros::Subscriber sub = nh.subscribe("/DJ_teleop", 1, teleop_callBack); //subscribe to ROS topic

    ros::spin();
    
    return 0;
}