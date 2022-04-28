#include "OSDK_Header.hpp"
#include "OSDK_Vehicle.hpp"
#include "OSDK_Virtual_RC.hpp"

using namespace DIABLO::OSDK;

uint8_t Virtual_RC::obtain_control(uint16_t timeout_ms)
{
    if(this->in_control()) return 5;
    
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Virtual_RC_Request_t) + OSDK_MISC_SIZE;
    header.data.SESSION = 2;
    header.data.ACK     = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    OSDK_Virtual_RC_Request_t req;
    req.request     = 1;
    req.timeout_act = 0;
    req.timeout_ms  = timeout_ms;

    uint16_t ack = -1;
    while(ack != 0x0002)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack,
        OSDK_VIRTUAL_RC_SET, OSDK_VIRTUAL_RC_AUTHORIZE_ID, 
        &req, sizeof(OSDK_Virtual_RC_Request_t));

        if(result) return result;
        if(ack == 0x0000)
        {
            std::cerr<<"SDK control disabled on manual RC controller, check your RC setting"<<std::endl;
            return 3;
        }
        if(ack == 0x000A)
        {
            std::cerr<<"Cannot switch to SDK control, check robot status"<<std::endl;
            return 3;
        }
        usleep(5000);
    }

    this->ctrl_status = CTRL_OBTAINED;
    std::cout<<"SDK Virtual RC In Control"<<std::endl;
    //abort();
    return 0;
}

uint8_t Virtual_RC::release_control()
{
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Virtual_RC_Request_t) + OSDK_MISC_SIZE;
    header.data.SESSION = 2;
    header.data.ACK     = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    OSDK_Virtual_RC_Request_t req;
    req.request     = 0;
    req.timeout_act = req.timeout_ms = 0;

    vehicle->hal->serialSend(header.data, 
        OSDK_VIRTUAL_RC_SET, OSDK_VIRTUAL_RC_AUTHORIZE_ID, 
        &req, sizeof(OSDK_Virtual_RC_Request_t));

    this->ctrl_status = CTRL_RELEASED;
    return 0;
}

uint8_t Virtual_RC::SendVirtualRCCmd()
{
    //printf("send VRC Cmd\n");
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Virtual_RC_t) + OSDK_MISC_SIZE;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    return vehicle->hal->serialSend(header.data, 
        OSDK_VIRTUAL_RC_SET, OSDK_VIRTUAL_RC_DATA_ID, 
        &data, sizeof(OSDK_Virtual_RC_t));
}
