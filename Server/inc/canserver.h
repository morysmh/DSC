#include "define.h"
#include "mcp2515.h"
#ifndef __CAN__Class__
#define __CAN__Class__

class CanCotroll
{
public:
    CanCotroll(int32_t i_add);
    ~CanCotroll(){}
    void run();
    bool read_new_msg(int32_t *i_para,int32_t *i_val,int8_t *motNO);
    void send_data(int32_t i_para,int32_t i_val,int8_t i_motNO);
private:
    typedef struct{
        int8_t i_para;
        int8_t motNO;
        int32_t i_val;
    }storage;
    void handle_message();
    bool handle_rx_confirm();
    int8_t ringbuff_adder(int8_t RbIndex, int8_t iVal);
    int16_t p_address = 100;
    int8_t p_send_head = 0,p_send_tail = 0;
    int8_t p_rx_head = 0,p_rx_tail = 0;
    const int8_t c_buff_size = 95;
    storage p_data[100] = {};
    storage p_rx[100] = {};
    tCAN p_can;
    int64_t p_mili = 0;
    const int64_t c_interval = 14510LL;
    uint8_t p_sendtwice = 0;
    int8_t p_retransmit_count = 0;
    const int8_t C_count_retransmit = 6;
};

#endif