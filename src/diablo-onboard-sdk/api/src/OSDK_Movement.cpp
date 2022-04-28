#include "OSDK_Movement.hpp"
#include "OSDK_Vehicle.hpp"
#include "OSDK_Header.hpp"

using namespace DIABLO::OSDK;

uint8_t Movement_Ctrl::obtain_control(uint16_t timeout_ms)
{
    if(this->in_control()) return 5;

    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Movement_Ctrl_Request_t) + OSDK_MISC_SIZE;
    header.data.SESSION = 2;
    header.data.ACK     = 1;
    header.data.SEQ     = vehicle->hal->serial_getSeq();
    header.append_crc();

    OSDK_Movement_Ctrl_Request_t req;
    req.request     = OSDK_ENABLE;
    req.timeout_act = 0;
    req.timeout_ms  = timeout_ms;

    uint16_t ack = -1;
    while(ack != 0x0002)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack, 
            OSDK_CONTROL_SET, OSDK_CTRL_AUTHORIZE_ID,
            &req, sizeof(OSDK_Movement_Ctrl_Request_t));
        
        if(result) return result;
        if(ack == 0x0000)
        {
            printf("ERROR: SDK control disable on manual movement_ctrl, check your robot.\n");
            return 3;
        }
        if(ack == 0x000A)
        {
            printf("ERROR: Cannot switch to SDK control, check yout robot status.\n");
            return 3;
        }
        usleep(5000);
    }
    
    this->ctrl_status = CTRL_OBTAINED;
    printf("SDK Handle Movement control\n");
    //abort();
    return 0;
}

uint8_t Movement_Ctrl::release_control()
{
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Movement_Ctrl_Request_t) + OSDK_MISC_SIZE;
    header.data.SESSION = 2;
    header.data.ACK     = 1;
    header.data.SEQ     = vehicle->hal->serial_getSeq();
    header.append_crc();

    OSDK_Movement_Ctrl_Request_t req;
    req.request     = OSDK_DISABLE;
    req.timeout_act = 0;
    req.timeout_ms  = 0;

    vehicle->hal->serialSend(header.data,
        OSDK_CONTROL_SET, OSDK_CTRL_AUTHORIZE_ID,
        &req, sizeof(OSDK_Movement_Ctrl_Request_t));
    
    this->ctrl_status = CTRL_RELEASED;
    return 0;
}

uint8_t Movement_Ctrl::EmergencyBrake(void)
{
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Brake_Cmd_t) + OSDK_MISC_SIZE;
    header.data.SESSION = 2;
    header.data.ACK     = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    OSDK_Brake_Cmd_t req = OSDK_ENABLE;

    uint16_t ack = -1;
    while(ack != OSDK_ENABLE)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack,
            OSDK_CONTROL_SET, OSDK_EMERGENCY_BRAKE_ID, 
            (uint8_t*)&req, sizeof(OSDK_Brake_Cmd_t));

        if(result) return result;
    }

    std::cout<<"==EMERGENCY BRAKE ACTIVATED=="<<std::endl;
    return 0;
}

uint8_t Movement_Ctrl::EmergencyBrakeRelease(void)
{
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Brake_Cmd_t) + OSDK_MISC_SIZE;
    header.data.SESSION = 2;
    header.data.ACK     = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    OSDK_Brake_Cmd_t req = OSDK_DISABLE;

    uint16_t ack = -1;
    while(ack != OSDK_DISABLE)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack,
            OSDK_CONTROL_SET, OSDK_EMERGENCY_BRAKE_ID, 
            (uint8_t*)&req, sizeof(OSDK_Brake_Cmd_t));

        if(result) return result;
    }

    std::cout<<"==EMERGENCY BRAKE RELEASED=="<<std::endl;
    return 0;
}   

uint8_t Movement_Ctrl::SendMovementModeCtrlCmd()
{
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) +
        sizeof(OSDK_Movement_Ctrl_Mode_t) + OSDK_MISC_SIZE;
    header.data.SESSION = 2;
    header.data.ACK = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    uint16_t ack =-1;
    while(ack != 0x0000)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack,
            OSDK_CONTROL_SET, OSDK_MOVEMENT_CTRL_MODE_ID,
            &ctrl_mode_data, sizeof(OSDK_Movement_Ctrl_Mode_t));

        if(result) return result;
    }

    printf("==MOVEMENT CONTROL MODE SET==\n");
    ctrl_mode_cmd = false;
    return 0;
}

uint8_t Movement_Ctrl::SendMovementCtrlCmd()
{
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Movement_Ctrl_t) + OSDK_MISC_SIZE;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    return vehicle->hal->serialSend(header.data,
        OSDK_CONTROL_SET, OSDK_MOVEMENT_CTRL_ID,
        &ctrl_data, sizeof(OSDK_Movement_Ctrl_t));
}

uint8_t Movement_Ctrl::SendTransformUpCmd()
{
    transform_data.transform_down = 0;
    transform_data.transform_up = 1;
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Transform_Cmd_t) + OSDK_MISC_SIZE;
    header.data.SESSION  = 2;
    header.data.ACK = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    uint16_t ack = -1;
    while(ack != 1)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack,
            OSDK_CONTROL_SET, OSDK_TRANSFORM_ID,
            &transform_data, sizeof(OSDK_Transform_Cmd_t));
        
        if(result) return result;

        if(ack == 0)
        {
            printf("==TRANSFORM UP FAIL==\n");
            return 0xFF;
        }
        usleep(10000);
    }
}

uint8_t Movement_Ctrl::SendTransformDownCmd()
{
    transform_data.transform_down = 1;
    transform_data.transform_up = 0;
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Transform_Cmd_t) + OSDK_MISC_SIZE;
    header.data.SESSION  = 2;
    header.data.ACK = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    uint16_t ack = -1;
    while(ack != 0)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack,
            OSDK_CONTROL_SET, OSDK_TRANSFORM_ID,
            &transform_data, sizeof(OSDK_Transform_Cmd_t));
        
        if(result) return result;

        if(ack == 1)
        {
            printf("==TRANSFORM DOWN FAIL==\n");
            return 0xFF;
        }
        usleep(10000);
    }
}