#include "canserver.h"
bool CanCotroll::read_new_msg(int32_t *i_para,int32_t *i_val,int8_t *motNO)
{
    if(p_rx_head == p_rx_tail)
        return false;
    *i_para = p_rx[p_rx_tail].i_para;
    *i_val = p_rx[p_rx_tail].i_val;
    *motNO = p_rx[p_rx_tail].motNO;
    p_rx_tail = ringbuff_adder(p_rx_tail,1);
    return true;
}
void CanCotroll::send_data(int32_t i_para,int32_t i_val,int8_t i_motNO)
{
    p_data[p_send_head].i_para = i_para;
    p_data[p_send_head].i_val = i_val;
    p_data[p_send_head].motNO = i_motNO;
    p_send_head = ringbuff_adder(p_send_head,1);
}
bool CanCotroll::handle_rx_confirm()
{
    return false;
    int32_t iVal = 0;
    int8_t iPara = 0;
    iVal |= (((uint32_t)p_can.data[0])<<0ULL);
    iVal |= (((uint32_t)p_can.data[1])<<8ULL);
    iVal |= (((uint32_t)p_can.data[2])<<16ULL);
    iVal |= (((uint32_t)p_can.data[3])<<24ULL);
    if(p_retransmit_count >= C_count_retransmit)
        return false;
    p_retransmit_count++;
    p_send_tail = ringbuff_adder(p_send_tail,-1);
    if(iVal != p_data[p_send_tail].i_val)
        return false;
    if(p_can.data[4] != p_data[p_send_tail].i_para)
        return false;
    if(p_can.data[7] != p_data[p_send_tail].motNO)
        return false;
    p_send_tail = ringbuff_adder(p_send_tail,1);
    p_retransmit_count = 0;
    return true;
}
void CanCotroll::handle_message()
{
    int32_t i_val = 0;
    int8_t i_motNO;
    if(p_can.id != C_Server_ADDRESS_CAN)
        return;
    i_val |= (((uint32_t)p_can.data[0])<<0ULL);
    i_val |= (((uint32_t)p_can.data[1])<<8ULL);
    i_val |= (((uint32_t)p_can.data[2])<<16ULL);
    i_val |= (((uint32_t)p_can.data[3])<<24ULL);
    p_rx[p_rx_head].i_para = p_can.data[4];
    p_rx[p_rx_head].i_val = i_val;
    p_rx[p_rx_head].motNO= p_can.data[7];
    p_rx_head = ringbuff_adder(p_rx_head,1);
}
void CanCotroll::run()
{
    if(mcp2515_check_message())
    {
        mcp2515_get_message(&p_can);
        handle_message();
    }
    if(p_mili > time_us_64())
        return;
    p_mili = time_us_64() + c_interval;
    if(p_send_head == p_send_tail)
        return;
    if(mcp2515_check_free_buffer() == false)
        return;
    p_can.header.length = 8;
    p_can.header.rtr = 0;
    p_can.id = ((p_data[p_send_tail].motNO) & 0xFFULL);
    p_can.data[0] =((p_data[p_send_tail].i_val>>0ULL) & 0xFFULL);
    p_can.data[1] =((p_data[p_send_tail].i_val>>8ULL) & 0xFFULL);
    p_can.data[2] =((p_data[p_send_tail].i_val>>16ULL) & 0xFFULL);
    p_can.data[3] =((p_data[p_send_tail].i_val>>24ULL) & 0xFFULL);
    p_can.data[4] =((p_data[p_send_tail].i_para) & 0xFFULL);
    if(mcp2515_send_message(&p_can) == false)
        return;
    p_send_tail = ringbuff_adder(p_send_tail,1);
}
CanCotroll::CanCotroll(int32_t i_add)
{
    p_address = 0xFF;
}
int8_t CanCotroll::ringbuff_adder(int8_t RbIndex, int8_t iVal)
{
    RbIndex += iVal;
    if(RbIndex < 0)
        RbIndex = c_buff_size;
    if(RbIndex > c_buff_size)
        RbIndex = 0;
    return RbIndex;
}