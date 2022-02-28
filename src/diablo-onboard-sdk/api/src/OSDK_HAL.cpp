#include <stdio.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include "OSDK_HAL.hpp"
#include "OSDK_Header.hpp"

#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/bind.hpp>
#include "OSDK_CRC.hpp"

using namespace DIABLO::OSDK;
using namespace boost;
using namespace boost::chrono;

void HAL::serialPackData(const OSDK_Uart_Header_t& header, 
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len)
{
    memcpy(serial_txbuf, &header, sizeof(OSDK_Uart_Header_t));
    serial_txbuf[sizeof(OSDK_Uart_Header_t)]     = cmd_set;
    serial_txbuf[sizeof(OSDK_Uart_Header_t) + 1] = cmd_id;
    memcpy(serial_txbuf + OSDK_DATA_POS_OFFSET, data, data_len);
    uint16_t CRC16 = DIABLO::Utility::update_crc16(serial_txbuf, OSDK_DATA_POS_OFFSET + data_len);
    memcpy(serial_txbuf + OSDK_DATA_POS_OFFSET + data_len, &CRC16, 2);
}

void* HAL::serialWaitRXDataS(boost::unique_lock<boost::mutex>& lock, const uint8_t cmd_set, const uint8_t cmd_id)
{
    static const chrono::duration<int, milli> timeout(1000);
    if(!serial_rx_data_cond.wait_for(lock, timeout,
        boost::bind(&DIABLO::OSDK::HAL::verifyRXType, this, cmd_set, cmd_id))) return NULL;

    *((uint8_t*)rx_data - 2) = *((uint8_t*)rx_data - 1) = -1; //mark rx cmd set and id to be invalid
    return getRXData();
}

uint8_t HAL::serialSend_ack(const OSDK_Uart_Header_t& header, uint16_t& ack,
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len)
{
    if(serialSend(header, cmd_set, cmd_id, data, data_len)) 
    {
        printf("no ack\n");
        return 1; //transmit timeout
    }

    static const chrono::duration<int, milli> timeout(100);
    {
        //printf("ack\n");
        unique_lock<mutex> lock(serial_rx_mtx);
        if(!serial_rx_ack_cond.wait_for(lock, timeout, 
            boost::bind(&DIABLO::OSDK::HAL::verifyRXType, this, cmd_set, cmd_id))) 
        {
            printf("wait ack timeout\n");
            return 2; //wait ack timeout
        }
        ack = this->getACK();
    }


    return 0;
}

void HAL::TXMonitorProcess(void)
{
    static const chrono::duration<int, milli> timeout(100);
    while(true)
    {
        {
            unique_lock<mutex> lock(serial_tx_mtx);
            if(serial_tx_idle && 
              !serial_tx_cond.wait_for(lock, timeout, [this](void){return !serial_tx_idle;}))
            {
                serial_tx_idle = true;
                continue;
            }
        }
        this_thread::sleep_for(boost::chrono::duration<double>(serial_tx_duration));
        {
            unique_lock<mutex> lock(serial_tx_mtx);
            serial_tx_idle = true;
            serial_tx_cond.notify_all();
        }
    }
}

void HAL::storeBrokenPacket(uint8_t pos, uint8_t len)
{
    //printf("broken packet\n");
    unique_lock<mutex> lock(serial_prerx_mtx);
    prebytes = len-pos;
    memcpy(serial_prerxbuf, (serial_rxbuf+pos), prebytes);
    
}

#if defined (_WITH_SERIAL)
uint8_t HAL_Serial::initSerial(std::string port, int baudrate, serial::Timeout timeout)
{
    this->serial = new serial::Serial(port, baudrate, timeout);
    if(!this->serial)
    {
        std::cout<<"Failed to allocate memory for serial port instance!"<<std::endl;
        return 1;
    }

    if(this->serial->isOpen())
    {
        this->serial->flush();
        serial_br = baudrate;
        serial_tx_duration = 0;
        serial_tx_idle = true;
        serial_tx_thd = new boost::thread(boost::bind(&HAL_Serial::TXMonitorProcess, this));
        serial_rx_thd = new boost::thread(boost::bind(&HAL_Serial::RXMonitorProcess, this)); //TODO CPU load too heavy, modify this

        usleep(10000); //ensure stability
        std::cout<<"Serial port \""<<port<<"\" connected"<<std::endl;
        rx_data = (void*)(serial_rxbuf + 2 + sizeof(OSDK_Uart_Header_t));   
        
        return 0;
    }
    else
    {
        std::cerr<<"Cannot connect to serial port \""<<port<<'\"'<<std::endl;
        return 2;
    }
}


uint8_t HAL_Serial::serialSend(const OSDK_Uart_Header_t& header, 
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len)
{
    static const chrono::duration<int, milli> timeout(100);
    unique_lock<mutex> lock(serial_tx_mtx);
    
    if(!serial_tx_idle && 
       !serial_tx_cond.wait_for(lock, timeout, [this](void){return serial_tx_idle;}))
        return 1;
    
    serialPackData(header, cmd_set, cmd_id, data, data_len);

    size_t size = OSDK_DATA_POS_OFFSET+data_len+2;

    // if(cmd_set == OSDK_VIRTUAL_RC_SET && cmd_id == OSDK_VIRTUAL_RC_DATA_ID)
    // {
    //     // for(int i=0;i<(int)size;i++)
    //     // {printf("%u ",serial_txbuf[i]);}
    //     VRC_Data_packet_num += 1;
    //     printf("VRC Data: %u\n",VRC_Data_packet_num);
    //     //printf("\n");
    // }

    // if(cmd_set == OSDK_VIRTUAL_RC_SET && cmd_id == OSDK_VIRTUAL_RC_AUTHORIZE_ID)
    // {
    //     // for(int i=0;i<(int)size;i++)
    //     // {printf("%u ",serial_txbuf[i]);}
    //     VRC_REQ_packet_num += 1;
    //     printf("VRC REQ: %u\n",VRC_REQ_packet_num);
    //     //printf("\n");
    // }


    serial_tx_duration = size*10./serial_br + 2e-4;
    serial_seq++;
    serial_tx_idle = false;
    serial_tx_cond.notify_all();

    this->serial->write(serial_txbuf, size);

    return 0;
}

void HAL_Serial::RXMonitorProcess(void)
{
    while(true)
    {
        uint8_t rx_bytes;
        uint8_t p = 0;

        //first packet header relocation
        if(!first_rx_flag)
        {
            printf("first packet issue\n");
            rx_bytes = serial->read(serial_rxbuf,256);
            first_rx_flag = rx_bytes > 0;
            while(serial_rxbuf[p] != OSDK_HEADER)
            {
                p += 1;
            }
        }
        else
        {
            if(prebytes)    // Handle broken packet
            {
                //printf("splice packet\n");
                unique_lock<mutex> lock0(serial_prerx_mtx);
                unique_lock<mutex> lock1(serial_rx_mtx);
                memcpy(serial_rxbuf, & serial_prerxbuf, prebytes);
                rx_bytes = serial->read(serial_rxbuf+prebytes, 256-prebytes);
                rx_bytes += prebytes;
                prebytes = 0;
            }
            else
            {
                rx_bytes = serial->read(serial_rxbuf, 256); //process serial receive of one or several packets
            }
        }
        printf("Bytes: %u\n",rx_bytes-p);
        for(int i=p;i<rx_bytes;i++)
        {
            printf("%u ",serial_rxbuf[i]);
        }
        printf("\n");
        while(p < rx_bytes && p < 256-sizeof(OSDK_Uart_Header_t))
        {
            Header* header = (Header*)(serial_rxbuf + p);
            // if(header->data.LEN > 255)
            // {printf("Invalid length\n");break;}

            if(rx_bytes-p < sizeof(OSDK_Uart_Header_t))  // packet head broken
            {
                //printf("broken packet 1! position: %u\n", p);
                storeBrokenPacket(p, rx_bytes);
                break;
            }
            else if(
                !header->verify() || //bad header
                header->data.LEN > 255
            ) 
            {
                //p += header->data.LEN;
                printf("invalid header\n");
                printf("%u\t%u\n",p,rx_bytes);
                prebytes = 0;
                break;
                // printf("Bytes: %u\n",rx_bytes-p);
                // for(int i=p;i<rx_bytes;i++)
                // {
                //     printf("%u ",serial_rxbuf[i]);
                // }
                // printf("\n");
                //abort();
                //p += std::min((uint16_t)header->data.LEN,(uint16_t)(rx_bytes-p));
            } //drop this frame
            else if((rx_bytes-p)<header->data.LEN)  // packet data broken
            {
                //printf("broken packet 2! position: %u\n", p);
                storeBrokenPacket(p, rx_bytes);
                break;
            }
            else if(!DIABLO::Utility::verify_crc16(serial_rxbuf+p, header->data.LEN))   //bad data
            {
                //printf("disturbed packet\n");
                p += header->data.LEN;
            }
            else
            {
                unique_lock<mutex> lock(serial_rx_mtx);
                rx_data = (void*)(serial_rxbuf + p + 2 + sizeof(OSDK_Uart_Header_t));
                if(header->data.ACK)
                {
                    // for(int i=p;i<p+header->data.LEN;i++)
                    // {
                    //     printf("%u ",serial_rxbuf[i]);
                    // }
                    // printf("\n");
                    //printf("ack receive\n");
                    serial_rx_ack_cond.notify_all();
                }
                else
                    serial_rx_data_cond.notify_all();
                p += header->data.LEN;
            }

            //printf("data_LEN:%u\n",header->data.LEN);
            header->data.SOF = 0xFF; //mark the start byte to be invalid
        }
    }
}
#endif

#if defined (_WITH_PI)
uint8_t HAL_Pi::serialSend(const OSDK_Uart_Header_t& header, 
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len)
{
    static const chrono::duration<int, milli> timeout(100);
    unique_lock<mutex> lock(serial_tx_mtx);

    if(!serial_tx_idle && 
       !serial_tx_cond.wait_for(lock, timeout, [this](void){return serial_tx_idle;}))
        return 1;
    
    serialPackData(header, cmd_set, cmd_id, data, data_len);

    size_t size = OSDK_DATA_POS_OFFSET+data_len+2;

    serial_tx_duration = size*10./serial_br + 2e-4;
    serial_seq++;
    serial_tx_idle = false;
    serial_tx_cond.notify_all();

    for(uint8_t i = 0; i < size; i++)   
        serialPutchar(fd, serial_txbuf[i]);

    return 0;
}

void HAL_Pi::RXMonitorProcess(void)
{
    Header header;
    uint8_t* p_header = (uint8_t*)(&header);
    uint32_t byte_micro = 1000000 * 10/serial_br;
    while(true)
    {
        int start = -1;
        while(start != OSDK_HEADER)
        {
            start = serialGetchar(this->fd);
            if(start == -1) //no data received at all
                std::cerr<<"Serial receive timeout occured 1!"<<std::endl;
        }
        
        //wait for receiving a complete header
        usleep(byte_micro * sizeof(OSDK_Uart_Header_t));
        while(serialDataAvail(fd) < sizeof(OSDK_Uart_Header_t) - 1) usleep(byte_micro);
        
        header.data.SOF = start;
        uint16_t len, data_len;
        for(uint16_t i = 1; i < sizeof(OSDK_Uart_Header_t); i++)
        {
            int result = serialGetchar(this->fd);
            if(result == -1)
            {
                std::cerr<<"Serial receive timeout occured 2!"<<std::endl;
                goto DROP_FRAME;
            }
            p_header[i] = result;
        }

        len = header.data.LEN;
        if(
            !header.verify() || //bad header
             len > 255 //invalid length
          ) goto DROP_FRAME;
        data_len = len - sizeof(OSDK_Uart_Header_t);

        //wait for receiving a complete frame
        usleep(byte_micro * data_len);
        while(serialDataAvail(fd) < data_len) usleep(byte_micro);

        serial_rx_mtx.lock();
        for(uint16_t i = sizeof(OSDK_Uart_Header_t); i < len; i++)
        {
            int result = serialGetchar(this->fd);
            
            if(result == -1)
            {
                std::cerr<<"Serial receive timeout occured 3!"<<std::endl;
                goto DROP_FRAME;
            }
            serial_rxbuf[i] = result;
        }

        memcpy(serial_rxbuf, &header, sizeof(OSDK_Uart_Header_t));
        if(!DIABLO::Utility::verify_crc16(serial_rxbuf, len)) goto DROP_FRAME;

        if(header.data.ACK)    serial_rx_ack_cond.notify_all();
        else                   serial_rx_data_cond.notify_all();

DROP_FRAME:
        header.data.SOF = 0xFF; //mark the start byte to be invalid
        serial_rx_mtx.unlock();
    }
}

void HAL_Pi::HeartBeatProcess(void)
{
    pinMode(heartbeat_pin, OUTPUT);
    digitalWrite(heartbeat_pin, HIGH);

    this->start_tp = system_clock::now();
    system_clock::time_point tp = start_tp + milliseconds(100);
    while(true)
    {
        this_thread::sleep_until(tp);
        digitalWrite(heartbeat_pin, LOW);
        tp += milliseconds(100);
        this_thread::sleep_until(tp);
        digitalWrite(heartbeat_pin, HIGH);
        tp += milliseconds(100);
    }
}

#endif