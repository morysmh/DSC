#include "define.h"
#include "mcp2515.h"
#ifndef __CAN__Class__
#define __CAN__Class__

class ServerCAN
{
public:
    ServerCAN(uint8_t iIn){};
    ~ServerCAN(){}
    void run();
    bool read_motLocation(int32_t *location,
                            bool *pBottomSensor,
                            bool *pTopSensor,
                            bool *pMotorMoving,
                            uint8_t *motNO);
    void send_data(uint8_t oMotNO,uint8_t iOpcode,uint8_t *data);
    void ptr8_to_int32(uint8_t *ptrdata,int32_t *iout);
    void int32_to_ptrint8(int32_t iVal,uint8_t *ptrdata);
private:
    bool _bv(uint32_t iVal,uint8_t iBV);
    typedef struct{
        uint8_t iOpCode;
        uint8_t imotNO;
        uint8_t idata[7];
    }storage;
    void copy_uint8(uint8_t *dest,uint8_t *source,uint8_t count);
    void handle_message();
    int8_t ringbuff_adder(int8_t RbIndex, int8_t iVal);
    int8_t p_send_head = 0,p_send_tail = 0;
    int8_t p_rx_head = 0,p_rx_tail = 0;
    const int8_t c_buff_size = 45;
    storage p_data[50] = {};
    storage p_rx[50] = {};
    tCAN p_can = {};
    int64_t p_mili = 0;
    const int64_t c_interval = 14510LL;
};

#endif