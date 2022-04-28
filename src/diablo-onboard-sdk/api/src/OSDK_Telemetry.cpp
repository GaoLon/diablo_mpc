#include "OSDK_Telemetry.hpp"
#include "OSDK_Header.hpp"
#include "OSDK_Vehicle.hpp"
#include "OSDK_Virtual_RC.hpp"
#include <boost/bind.hpp>
#include <math.h>

using namespace DIABLO::OSDK;
using namespace boost;

Telemetry::Telemetry(Vehicle* vehicle): vehicle(vehicle)
{
    log_flag = 0;
    newcome = 0x00;
    memset(bit_r, 0, 7);
    bit_r[0] = 1;
    bit_r[5] = 1;
    memset(frequency_flag, OSDK_PUSH_DATA_NO_CHANGE, 7);
    thd = new boost::thread(bind(&Telemetry::SerialHandle, this));
}

uint8_t Telemetry::activate(void)
{
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 4 + OSDK_MISC_SIZE;
    header.data.SESSION = 2;
    header.data.ACK     = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    uint32_t key = 0x12345678;

    vehicle->hal->serialSend(header.data, OSDK_INIT_SET, OSDK_ACTIVATION_ID, &key, 4);
}

uint8_t Telemetry::Calibration(OSDK_Offline_Calibration_Cmd_t calibration_cmd)
{
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 2 + OSDK_MISC_SIZE;
    header.data.SESSION = 2;
    header.data.ACK = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();
    uint16_t ack = -1;
    while(ack != OSDK_ENABLE)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack, 
        OSDK_INIT_SET, OSDK_OFFLINE_CALIBRATION_ID, &calibration_cmd, 2);
        if(result) return result;
        if(ack == 0x0001)
        {
            printf("Error: Low Battery, Calibration denied\n");
            return 1;
        }
        if(ack == 0x0002)
        {
            printf("Error: Robot has to stand before calibration. Check your RC setting\n");
            return 2;
        }
        sleep(5);
    }
    return 0;
}

uint8_t Telemetry::configTopic(const OSDK_Topic_t topic, const OSDK_Push_Data_Freq_Select_t freq)
{
    switch(topic)
    {
        case TOPIC_STATUS:
            if(freq < OSDK_PUSH_DATA_10Hz)
            {
                std::cerr<<"Cannot set telemetry topic frequency \"status\" below 10Hz" <<std::endl;
                return 1;
            }
            frequency_flag[0] = freq;
            break;
        case TOPIC_QUATERNION:
            frequency_flag[1] = freq;
            break;
        case TOPIC_ACCL:
            frequency_flag[2] = freq;
            break;
        case TOPIC_GYRO:
            frequency_flag[3] = freq;
            break;
        case TOPIC_RC:
            frequency_flag[4] = freq;
            break;
        case TOPIC_POWER:
            frequency_flag[5] = freq;
            break;
        case TOPIC_MOTOR:
            frequency_flag[6] = freq;
            break;
        default:
            std::cerr<<"Invalid selection of telemetry topic"<<std::endl;
            return 2;
            break;
    }

    return 0;
}

uint8_t Telemetry::configUpdate(const bool save)
{
    OSDK_Set_Push_Data_Freq_t msg;
    msg.save_config = save ? 1 : 0;
    msg.status      = frequency_flag[0];
    msg.quaternion  = frequency_flag[1];
    msg.accl        = frequency_flag[2];
    msg.gyro        = frequency_flag[3];
    msg.RC          = frequency_flag[4];
    msg.power       = frequency_flag[5];
    msg.motor       = frequency_flag[6];

    for(int i=0; i<7; i++)
    {
        bit_r[i] <<= 16;
        switch(frequency_flag[i]){
            case OSDK_PUSH_DATA_OFF:
                bit_r[i] |= 0;
                break;
            case OSDK_PUSH_DATA_1Hz:
                bit_r[i] |= 1;
                break;
            case OSDK_PUSH_DATA_10Hz:
                bit_r[i] |= 10;
                break;
            case OSDK_PUSH_DATA_50Hz:
                bit_r[i] |= 50;
                break;
            case OSDK_PUSH_DATA_100Hz:
                bit_r[i] |= 100;
                break;
            case OSDK_PUSH_DATA_500Hz:
                bit_r[i] |= 500;
                break;
            case OSDK_PUSH_DATA_1000Hz:
                bit_r[i] |= 1000;
                break;
            case OSDK_PUSH_DATA_NO_CHANGE:
                break;
            default:
                printf("Invalid frequency selection!\n");
        }
    }
    
    if(calculatebitrate() > (uint32_t)(this->vehicle->hal->getSerialBr()*3/4))
    {
        printf("Insufficient Bandwidth. No frequency change will be made.\n");
        for(int i=0; i<7; i++)
        {
            bit_r[i] >>= 16;
            frequency_flag[i] = OSDK_PUSH_DATA_NO_CHANGE;
        }
        return 1;
    }
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Set_Push_Data_Freq_t) + OSDK_MISC_SIZE;
    header.data.SESSION = 1;
    header.data.ACK     = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    vehicle->hal->serialSend(header.data, 
        OSDK_INIT_SET, OSDK_SET_PUSH_DATA_FREQ_ID, 
        (uint8_t*)&msg, sizeof(OSDK_Set_Push_Data_Freq_t));

    memset(frequency_flag, OSDK_PUSH_DATA_NO_CHANGE, 7);

    printf("config update\n");

    return 0;
}

uint32_t Telemetry::calculatebitrate()
{
    return (sizeof(OSDK_Uart_Header_t)+sizeof(OSDK_Push_Data_Flag_t)+sizeof(OSDK_Push_Data_Status_t)+4)*10*(bit_r[0] & 0x0000FFFF)
        + (sizeof(OSDK_Uart_Header_t)+sizeof(OSDK_Push_Data_Flag_t)+sizeof(OSDK_Push_Data_Quaternion_t)+4)*10*(bit_r[1] & 0x0000FFFF)
        + (sizeof(OSDK_Uart_Header_t)+sizeof(OSDK_Push_Data_Flag_t)+sizeof(OSDK_Push_Data_XYZ_t)+4)*10*(bit_r[2] & 0x0000FFFF)
        + (sizeof(OSDK_Uart_Header_t)+sizeof(OSDK_Push_Data_Flag_t)+sizeof(OSDK_Push_Data_XYZ_t)+4)*10*(bit_r[3] & 0x0000FFFF)
        + (sizeof(OSDK_Uart_Header_t)+sizeof(OSDK_Push_Data_Flag_t)+sizeof(OSDK_Push_Data_RC_t)+4)*10*(bit_r[4] & 0x0000FFFF)
        + (sizeof(OSDK_Uart_Header_t)+sizeof(OSDK_Push_Data_Flag_t)+sizeof(OSDK_Push_Data_Power_t)+4)*10*(bit_r[5] & 0x0000FFFF)
        + (sizeof(OSDK_Uart_Header_t)+sizeof(OSDK_Push_Data_Flag_t)+sizeof(OSDK_Push_Data_Motor_t)+4)*10*(bit_r[6] & 0x0000FFFF);
}

void Telemetry::SerialDisconnectHandle(void)
{
    memset(&this->status, 0, sizeof(OSDK_Push_Data_Status_t));
    vehicle->virtual_rc->SerialDisconnectHandle();
    vehicle->movement_ctrl->SerialDisconnectHandle();
    std::cerr<<"OSDK Serial Receive Timeout Occured!"<<std::endl;
}

void Telemetry::SerialHandle(void)
{
    while(true)
    {
        //printf("%u\n",newcome);
        boost::unique_lock<boost::mutex> lock(vehicle->hal->serial_rx_mtx);
        OSDK_Push_Data_Flag_t* flag = (OSDK_Push_Data_Flag_t*)(vehicle->hal->serialWaitRXDataS(lock, OSDK_DATA_SET, OSDK_PUSH_DATA_ID));

        if(flag == NULL)
        {
            this->status.robot_mode = OSDK_ROBOT_STATE_DISCONNECT;
            this->SerialDisconnectHandle();
            continue;
        }

        uint8_t* pData = (uint8_t*)flag + sizeof(OSDK_Push_Data_Flag_t);
        
        //process timestamp  
        this->timestamp = *((OSDK_Push_Data_Timestamp_t*)pData)/10000.;
        pData += sizeof(OSDK_Push_Data_Timestamp_t);
        log_start = false; //will only record timestamp once per log

        if(flag->status)
        {
            memcpy(&this->status, pData, sizeof(OSDK_Push_Data_Status_t));
            vehicle->movement_ctrl->CtrlStatusMonitorHandle(status.ctrl_mode);
            vehicle->virtual_rc->CtrlStatusMonitorHandle(status.ctrl_mode);
            pData += sizeof(OSDK_Push_Data_Status_t);
            setNewcomeFlag(0x40);
            
            if(log_flag & 1 << TOPIC_STATUS)    statusLog(status);
        }
        if(flag->quaternion)
        {
            memcpy(&this->quaternion, pData, sizeof(OSDK_Push_Data_Quaternion_t));
            pData += sizeof(OSDK_Push_Data_Quaternion_t);
            setNewcomeFlag(0x20);

            //printf("q");

            if(log_flag & 1 << TOPIC_QUATERNION)    quaternionLog(quaternion);
        }
        if(flag->accl)
        {
            memcpy(&this->accl, pData, sizeof(OSDK_Push_Data_XYZ_t));
            pData += sizeof(OSDK_Push_Data_XYZ_t);
            setNewcomeFlag(0x10);

            //printf("a");

            if(log_flag & 1 << TOPIC_ACCL)  acclLog(accl);
        }
        if(flag->gyro)
        {
            memcpy(&this->gyro, pData, sizeof(OSDK_Push_Data_XYZ_t));
            pData += sizeof(OSDK_Push_Data_XYZ_t);
            setNewcomeFlag(0x08);

            //printf("g");

            if(log_flag & 1 << TOPIC_GYRO)  gyroLog(gyro);
        }
        if(flag->RC)
        {
            memcpy(&this->rc, pData, sizeof(OSDK_Push_Data_RC_t));
            pData += sizeof(OSDK_Push_Data_RC_t);
            setNewcomeFlag(0x04);

            //printf("rc\n");

            if(log_flag & 1 << TOPIC_RC)    rcLog(rc);
        }
        if(flag->power)
        {
            memcpy(&this->power, pData, sizeof(OSDK_Push_Data_Power_t));
            pData += sizeof(OSDK_Push_Data_Power_t);
            setNewcomeFlag(0x02);

            if(log_flag & 1 << TOPIC_POWER)    powerLog(power);
        }
        if(flag->motor)
        {
            motors.left_hip.rev = ((OSDK_Push_Data_Motor_t*)pData)->left_hip.enc_rev;
            motors.left_hip.enc_pos = ((OSDK_Push_Data_Motor_t*)pData)->left_hip.enc_pos;
            motors.left_hip.pos = motors.left_hip.enc_pos * 2 * M_PI / 32768;
            motors.left_hip.vel = ((OSDK_Push_Data_Motor_t*)pData)->left_hip.enc_vel * CAN_M15_RPM_PSC * CAN_RPM_2_RADIAN_SEC_PSC;
            motors.left_hip.iq = ((OSDK_Push_Data_Motor_t*)pData)->left_hip.iq * CAN_M15_IQ_PSC;

            motors.left_knee.rev = ((OSDK_Push_Data_Motor_t*)pData)->left_knee.enc_rev;
            motors.left_knee.enc_pos = ((OSDK_Push_Data_Motor_t*)pData)->left_knee.enc_pos;
            motors.left_knee.pos = motors.left_knee.enc_pos * 2 * M_PI / 32768;
            motors.left_knee.vel = ((OSDK_Push_Data_Motor_t*)pData)->left_knee.enc_vel * CAN_M15_RPM_PSC * CAN_RPM_2_RADIAN_SEC_PSC;
            motors.left_knee.iq = ((OSDK_Push_Data_Motor_t*)pData)->left_knee.iq * CAN_M15_IQ_PSC;

            motors.left_wheel.rev = ((OSDK_Push_Data_Motor_t*)pData)->left_wheel.enc_rev;
            motors.left_wheel.enc_pos = ((OSDK_Push_Data_Motor_t*)pData)->left_wheel.enc_pos;
            motors.left_wheel.pos = motors.left_wheel.enc_pos * 2 * M_PI / 32768;
            motors.left_wheel.vel = ((OSDK_Push_Data_Motor_t*)pData)->left_wheel.enc_vel * CAN_M15_RPM_PSC * CAN_RPM_2_RADIAN_SEC_PSC;
            motors.left_wheel.iq = ((OSDK_Push_Data_Motor_t*)pData)->left_wheel.iq * CAN_M15_IQ_PSC;

            motors.right_hip.rev = ((OSDK_Push_Data_Motor_t*)pData)->right_hip.enc_rev;
            motors.right_hip.enc_pos = ((OSDK_Push_Data_Motor_t*)pData)->right_hip.enc_pos;
            motors.right_hip.pos = motors.right_hip.enc_pos * 2 * M_PI / 32768;
            motors.right_hip.vel = ((OSDK_Push_Data_Motor_t*)pData)->right_hip.enc_vel * CAN_M15_RPM_PSC * CAN_RPM_2_RADIAN_SEC_PSC;
            motors.right_hip.iq = ((OSDK_Push_Data_Motor_t*)pData)->right_hip.iq * CAN_M15_IQ_PSC;

            motors.right_knee.rev = ((OSDK_Push_Data_Motor_t*)pData)->right_knee.enc_rev;
            motors.right_knee.enc_pos = ((OSDK_Push_Data_Motor_t*)pData)->right_knee.enc_pos;
            motors.right_knee.pos = motors.right_knee.enc_pos * 2 * M_PI / 32768;
            motors.right_knee.vel = ((OSDK_Push_Data_Motor_t*)pData)->right_knee.enc_vel * CAN_M15_RPM_PSC * CAN_RPM_2_RADIAN_SEC_PSC;
            motors.right_knee.iq = ((OSDK_Push_Data_Motor_t*)pData)->right_knee.iq * CAN_M15_IQ_PSC;

            motors.right_wheel.rev = ((OSDK_Push_Data_Motor_t*)pData)->right_wheel.enc_rev;
            motors.right_wheel.enc_pos = ((OSDK_Push_Data_Motor_t*)pData)->right_wheel.enc_pos;
            motors.right_wheel.pos = motors.right_wheel.enc_pos * 2 * M_PI / 32768;
            motors.right_wheel.vel = ((OSDK_Push_Data_Motor_t*)pData)->right_wheel.enc_vel * CAN_M15_RPM_PSC * CAN_RPM_2_RADIAN_SEC_PSC;
            motors.right_wheel.iq = ((OSDK_Push_Data_Motor_t*)pData)->right_wheel.iq * CAN_M15_IQ_PSC;
            pData += sizeof(OSDK_Push_Data_M15_t)*6;
            setNewcomeFlag(0x01);

            if(log_flag & 1 << TOPIC_MOTOR)    motorLog(motors);
        }

        memset(flag, 0, sizeof(OSDK_Push_Data_Flag_t));//invalidate flag variable
    }
}
