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
    int32_t p_address = 100;
    uint8_t p_send_head = 0,p_send_tail = 0;
    uint8_t p_rx_head = 0,p_rx_tail = 0;
    const uint8_t c_buff_size = 75;
    storage p_data[150] = {};
    storage p_rx[150] = {};
    tCAN p_can;
    int64_t p_mili = 0;
    const int64_t c_interval = 3128LL;
    uint8_t p_sendtwice = 0;
};

#endif