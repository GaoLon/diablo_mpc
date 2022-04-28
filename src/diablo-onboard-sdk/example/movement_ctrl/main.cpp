#include <ros/ros.h>
#include <std_msgs/String.h>

#include "OSDK_Vehicle.hpp"
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>

DIABLO::OSDK::Movement_Ctrl* pMovementCtrl;

void teleop_ctrl(const std_msgs::String::ConstPtr& msg)
{
    if(!pMovementCtrl->in_control())
    {
        printf("to get movement ctrl.\n");
        uint8_t result = pMovementCtrl->obtain_control();
        return;
    }
    if(pMovementCtrl->ctrl_mode_data.height_ctrl_mode == 0)
        pMovementCtrl->ctrl_data.up=0.0f;
    pMovementCtrl->ctrl_data.forward = 0.0f;
    pMovementCtrl->ctrl_data.left = 0.0f;
    for(const char& c : msg->data)
    {
        switch(c)
        {
            case 'w':
                pMovementCtrl->ctrl_data.forward = 1.0f;
                break;
            case 'a':
                pMovementCtrl->ctrl_data.left = 1.0f;
                break;
            case 's':
                pMovementCtrl->ctrl_data.forward = -1.0f;
                break;
            case 'd':
                pMovementCtrl->ctrl_data.left = 1.0f;
                break;
            case 'q':
                pMovementCtrl->ctrl_data.roll = -0.1f;
                break;
            case 'e':
                pMovementCtrl->ctrl_data.roll = 0.1f;
                break;
            case 'r':
                pMovementCtrl->ctrl_data.roll = 0.0f;
                break;
            case 'z':
                pMovementCtrl->SendTransformDownCmd();
                return;
                break;
            case 'v':
                pMovementCtrl->SendTransformUpCmd();
                printf("up\n");
                return;
                break;
            case 'n':
                pMovementCtrl->ctrl_mode_data.height_ctrl_mode = 0;      // vel ctrl mode
                pMovementCtrl->ctrl_mode_cmd = true;
                break;
            case 'm':
                pMovementCtrl->ctrl_mode_data.height_ctrl_mode = 1;      // pos ctrl mode
                pMovementCtrl->ctrl_mode_cmd = true;
                break;
            case 'f':
                pMovementCtrl->ctrl_data.up = 0.0f;     //pos & angular_vel ctrl
                break;
            case 'g':
                pMovementCtrl->ctrl_data.up = 0.5f;     //pos ctrl
                break;
            case 'h':
                pMovementCtrl->ctrl_data.up = 1.0f;     //pos ctrl
                break;
            case 'x':
                pMovementCtrl->ctrl_data.up = -0.1f;    // vel ctrl
                break;
            case 'c':
                pMovementCtrl->ctrl_data.up = 0.1f;     // vel ctrl
                break;
            case 'y':
                pMovementCtrl->ctrl_mode_data.pitch_ctrl_mode = 0;      // angular_vel ctrl mode
                pMovementCtrl->ctrl_mode_cmd = true;
                break;
            case 'u':
                pMovementCtrl->ctrl_mode_data.pitch_ctrl_mode = 1;      // pos ctrl mode
                pMovementCtrl->ctrl_mode_cmd = true;
                break;
            case 'i':
                pMovementCtrl->ctrl_data.pitch = -0.5f; // pos & angular_vel ctrl
                break;
            case 'o':
                pMovementCtrl->ctrl_data.pitch = 0.0f;  // pos & angular_vel ctrl
                break;
            case 'p':
                pMovementCtrl->ctrl_data.pitch = 0.5f;  // pos & angular_vel ctrl
                break;
            case 'j':
                pMovementCtrl->ctrl_data.leg_split = -0.3f;
                break;
            case 'k':
                pMovementCtrl->ctrl_data.leg_split = 0.0f;
                break;
            case 'l':
                pMovementCtrl->ctrl_data.leg_split = 0.3f;
                break;
            case 'b':
                pMovementCtrl->EmergencyBrake();
                return;
            default:
                break;
        }
    }

    if(pMovementCtrl->ctrl_mode_cmd)
    {uint8_t result = pMovementCtrl->SendMovementModeCtrlCmd();}
    else
    {uint8_t result = pMovementCtrl->SendMovementCtrlCmd();}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "movement_ctrl_example");
    ros::NodeHandle nh("~");

#if defined(_WITH_SERIAL)
    DIABLO::OSDK::HAL_Serial Hal;                                                //Initialize HAL driver
    Hal.initSerial("/dev/ttyUSB0", 115200);                               //Initialize serial port
#elif defined(_WITH_PI)
    DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init()) return -1;
#endif

    DIABLO::OSDK::Vehicle vehicle(&Hal);                                  //Initialize Onboard SDK
    if(vehicle.init()) return -1;

    printf("%d\n",sizeof(OSDK_Uart_Header_t));

    vehicle.telemetry->activate();
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_1Hz);
    //vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_100Hz);
    // vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_100Hz);
    // vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_100Hz);
    vehicle.telemetry->configUpdate(); 
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_MOTOR);
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_QUATERNION);
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_ACCL);
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_GYRO);

    pMovementCtrl = vehicle.movement_ctrl;
    ros::Subscriber sub = nh.subscribe("/DJ_teleop", 1, teleop_ctrl); //subscribe to ROS topic

    ros::spin();

    return 0;
}