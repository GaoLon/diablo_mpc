#include <ros/ros.h>
#include <std_msgs/String.h>

#include "OSDK_Vehicle.hpp"
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>

DIABLO::OSDK::Bridge* ros_bridge;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_bridge_example");
    ros::NodeHandle nh("~");

#if defined (_WITH_SERIAL)
    DIABLO::OSDK::HAL_Serial Hal;
    Hal.initSerial("/dev/ttyUSB0", 460800);
#elif defined (_WITH_PI)
    DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init())    return -1;
#endif

    DIABLO::OSDK::Vehicle vehicle(&Hal);

    if(vehicle.init())  return -1;

    vehicle.telemetry->activate();
    //vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_POWER, OSDK_PUSH_DATA_10Hz);
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_100Hz);
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_100Hz);
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_100Hz);
    vehicle.telemetry->configUpdate(); 
    vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_POWER);
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_QUATERNION);
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_ACCL);
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_GYRO);

    ros_bridge = vehicle.bridge;
    ((DIABLO::OSDK::ROS_Bridge*)ros_bridge)->setPublisher(nh);
    ((DIABLO::OSDK::ROS_Bridge*)ros_bridge)->ActivateBridge();

    ros::spin();

    return 0;
}