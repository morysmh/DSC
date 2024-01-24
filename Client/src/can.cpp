#include "can.h"
bool CanCotroll::read_new_msg(int32_t *i_para,int32_t *i_val)
{
    if(p_rx_head == p_rx_tail)
        return false;
    *i_para = p_rx[p_rx_tail].i_para;
    *i_val = p_rx[p_rx_tail].i_val;
    p_rx_tail++;
    if(p_rx_tail >= c_buff_size)
        p_rx_tail = 0;
    return true;
}
void CanCotroll::send_data(int32_t i_para,int32_t i_val)
{
    p_data[p_send_head].i_para = i_para;
    p_data[p_send_head].i_val = i_val;
    p_send_head++;
    if(p_send_head >= c_buff_size)
        p_send_head = 0;
}
void CanCotroll::handle_message()
{
    int32_t i_val = 0;
    i_val |= (((uint32_t)p_can.data[0])<<0ULL);
    i_val |= (((uint32_t)p_can.data[1])<<8ULL);
    i_val |= (((uint32_t)p_can.data[2])<<16ULL);
    i_val |= (((uint32_t)p_can.data[3])<<24ULL);
    p_rx[p_rx_head].i_para = (uint32_t)p_can.data[4];
    p_rx[p_rx_head].i_val = i_val;
    p_rx_head++;
    if(p_rx_head >= c_buff_size)
        p_rx_head = 0;
}
void CanCotroll::run()
{
    if(mcp2515_check_message())
    {
        mcp2515_get_message(&p_can);
        if((p_can.id == p_address) || (p_can.id == C_DSC_BRODCAST_ADDRESS_CAN))
            handle_message();
    }
    if(p_send_head == p_send_tail)
        return;
    if(mcp2515_check_free_buffer() == false)
        return;
    p_can.header.length = 8;
    p_can.header.rtr = 0;
    p_can.id = C_DSC_Server_ADDRESS_CAN;
    p_can.data[0] =((p_data[p_send_tail].i_val>>0ULL) & 0xFFULL);
    p_can.data[1] =((p_data[p_send_tail].i_val>>8ULL) & 0xFFULL);
    p_can.data[2] =((p_data[p_send_tail].i_val>>16ULL) & 0xFFULL);
    p_can.data[3] =((p_data[p_send_tail].i_val>>24ULL) & 0xFFULL);
    p_can.data[4] =((p_data[p_send_tail].i_para) & 0xFFULL);
    p_can.data[7] =(p_address & 0xFFULL);
    if(mcp2515_send_message(&p_can) == false)
        return;
    p_send_tail++;
    if(p_send_tail >= c_buff_size)
        p_send_tail = 0;
    
}
CanCotroll::CanCotroll(int32_t i_add)
{
    if(i_add == 0)
        i_add = 100;
    p_address = i_add;
}
void CanCotroll::software_message(int32_t i_para,int32_t i_val)
{
    p_rx[p_rx_head].i_para = i_para;
    p_rx[p_rx_head].i_val = i_val;
    p_rx_head++;
    if(p_rx_head >= c_buff_size)
        p_rx_head = 0;
}