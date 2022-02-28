#include <ros/ros.h>
#include <std_msgs/String.h>

#include "diablo_sdk/OSDK_ACCL.h"
#include "diablo_sdk/OSDK_GYRO.h"
#include "diablo_sdk/OSDK_LEGMOTORS.h"
#include "diablo_sdk/OSDK_POWER.h"
#include "diablo_sdk/OSDK_QUATERNION.h"
#include "diablo_sdk/OSDK_RC.h"
#include "diablo_sdk/OSDK_STATUS.h"

void ACCL_sub(diablo_sdk::OSDK_ACCL msg)
{
    printf("ACCL_X:\t%f\nACCL_Y:\t%f\nACCL_Z:\t%f\n", msg.x, msg.y, msg.z);
}

void GYRO_sub(diablo_sdk::OSDK_GYRO msg)
{
    printf("GYRO_X:\t%f\nGYRO_Y:\t%f\nGYRO_Z:\t%f\n", msg.x, msg.y, msg.z);
}

void POWER_sub(diablo_sdk::OSDK_POWER msg)
{
    printf("Power:\nVoltage:\t%f\nCurrent:\t%f\nCap_EN:\t%f\nPercent:\t%u\n", msg.battery_voltage, msg.battery_current, msg.battery_current, msg.battery_power_percent);
}

void QUATERNION_sub(diablo_sdk::OSDK_QUATERNION msg)
{
    printf("Quaternion_w:\t%f\nQuaternion_x:\t%f\nQuaternion_y:\t%f\nQuaternion_z:\t%f\n", msg.w, msg.x, msg.y, msg.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bridge_receiver");
    ros::NodeHandle nh;

    ros::Subscriber sub_accl = nh.subscribe("/ros_bridge_example/diablo_ros_ACCL_b", 10, ACCL_sub);
    ros::Subscriber sub_gyro = nh.subscribe("/ros_bridge_example/diablo_ros_GYRO_b", 10, GYRO_sub);
    ros::Subscriber sub_power = nh.subscribe("/ros_bridge_example/diablo_ros_POWER_b", 10, POWER_sub);
    ros::Subscriber sub_quaternion = nh.subscribe("/ros_bridge_example/diablo_ros_QUATERNION_b", 10, QUATERNION_sub);
    printf("sub1\n");

    ros::spin();

    return 0;
}