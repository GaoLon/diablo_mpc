#include <ros/ros.h>
#include <std_msgs/String.h>
#include "unistd.h"

#include "OSDK_Vehicle.hpp"
#include <pthread.h>
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>
#include <chrono>

#include "OSDK_Callback.hpp"

void testcallback1(void* ptr)
{
    printf("callback1 %lld\n", *((int64_t*)ptr));
    sleep(10);
}
void testcallback2(void* ptr)
{
    printf("callback2 %lld\n", *((int64_t*)ptr));
    sleep(10);
}
void testcallback3(void* ptr)
{
    printf("callback3 %lld\n", *((int64_t*)ptr));
    sleep(10);
}
void testcallback4(void* ptr)
{
    printf("callback4 %lld\n", *((int64_t*)ptr));
    sleep(10);
}

void receivedata(DIABLO::OSDK::Broadcaster* bd)
{
    while(true)
    {
        int64_t t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        //printf("ts: %lld\n", t);
        bd->Broadcast((void*)&t);
        usleep(10000);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "callback_test");
    ros::NodeHandle nh();

    #if defined(_WITH_SERIAL)
    DIABLO::OSDK::HAL_Serial Hal;                                                //Initialize HAL driver
    Hal.initSerial("/dev/ttyUSB0", 460800);                               //Initialize serial port
    #elif defined(_WITH_PI)
    DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init()) return -1;
    #endif

    DIABLO::OSDK::Vehicle vehicle(&Hal);                                  //Initialize Onboard SDK
    if(vehicle.init()) return -1;

    uint32_t bandwidth = (sizeof(OSDK_Uart_Header_t)+sizeof(OSDK_Push_Data_Flag_t)+sizeof(OSDK_Push_Data_Status_t)+4)*10*10
        + (sizeof(OSDK_Uart_Header_t)+sizeof(OSDK_Push_Data_Flag_t)+sizeof(OSDK_Push_Data_Quaternion_t)+4)*10*500
        + (sizeof(OSDK_Uart_Header_t)+sizeof(OSDK_Push_Data_Flag_t)+sizeof(OSDK_Push_Data_XYZ_t)+4)*10*600
        + + (sizeof(OSDK_Uart_Header_t)+sizeof(OSDK_Push_Data_Flag_t)+sizeof(OSDK_Push_Data_Power_t)+4)*10;
    printf("%u\n",bandwidth);

    vehicle.telemetry->activate();
    //vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_POWER, OSDK_PUSH_DATA_10Hz);
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_100Hz);
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_100Hz);
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_100Hz);
    vehicle.telemetry->configUpdate(); 
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_POWER);

    DIABLO::OSDK::Broadcaster* broadcaster = DIABLO::OSDK::CallBack_Handler::GetInstance().NewBroadcaster<int64_t*>("dataget", 10);
    DIABLO::OSDK::CallBack_Handler::GetInstance().NewListener("dataget", 10,testcallback1);
    DIABLO::OSDK::CallBack_Handler::GetInstance().NewListener("dataget", 10,testcallback2);
    DIABLO::OSDK::CallBack_Handler::GetInstance().NewListener("dataget", 10,testcallback3);
    DIABLO::OSDK::CallBack_Handler::GetInstance().NewListener("dataget", 10,testcallback4);
    DIABLO::OSDK::Callback_Manager* manager = DIABLO::OSDK::CallBack_Handler::GetInstance().GetManager("dataget");
    manager->Activate();
    printf("%d\n",manager->GetNumListener());
    boost::thread* re_thd;
    re_thd = new boost::thread(boost::bind(&receivedata, broadcaster));

    ros::spin();
}