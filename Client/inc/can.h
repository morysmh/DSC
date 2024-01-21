#include "define.h"
#include "mcp2515.h"

class CanCotroll
{
public:
    CanCotroll(int32_t i_add);
    ~CanCotroll(){}
    void send_data(int32_t i_para,int32_t i_val);
    void run();
    bool read_new_msg(int32_t *i_para,int32_t *i_val);
private:
    typedef struct{
        int8_t i_para;
        int32_t i_val;
    }storage;
    void handle_message();
    int32_t p_address = 100;
    uint8_t p_send_head = 0,p_send_tail = 0;
    uint8_t p_rx_head = 0,p_rx_tail = 0;
    const uint8_t c_buff_size = 35;
    storage p_data[40] = {};
    storage p_rx[40] = {};
    tCAN p_can;
};