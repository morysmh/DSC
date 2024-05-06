#include "canserver.h"
bool ServerCAN::read_motLocation(int32_t *olocation,int16_t *statusData,uint8_t *motNO)
{
    if(p_rx_head == p_rx_tail)
        return false;
    if(p_rx[p_rx_tail].iOpCode != C_DSC_OPCODE_REG_STATUS)
    {
        p_rx_tail = ringbuff_adder(p_rx_tail,1);
        return false;
    }
    ptr8_to_int32(&p_rx[p_rx_tail].idata[C_DSC_ARRAY_STATUS_LOCATION_index],olocation);
    *statusData = p_rx[p_rx_tail].idata[C_DSC_ARRAY_STATUS_REPORT_1byte];
    *motNO = p_rx[p_rx_tail].imotNO;
    p_rx_tail = ringbuff_adder(p_rx_tail,1);
    return true;
}
void ServerCAN::send_data(uint8_t oMotNO,uint8_t iOpcode,uint8_t *data)
{
    p_data[p_send_head].iOpCode = iOpcode;
    p_data[p_send_head].imotNO = oMotNO;
    copy_uint8(p_data[p_send_head].idata,data,7);
    p_send_head = ringbuff_adder(p_send_head,1);
}
void ServerCAN::handle_ack()
{
    if(p_data[p_send_tail].imotNO != (p_can.id - C_Server_ADDRESS_CAN))
        return;
    if(p_data[p_send_tail].iOpCode != (p_can.data[C_DSC_ARRAY_OPCODE] - C_DSC_OPCODE_ADD_VALUE_ACK))
        return;
    for(uint i=0;i<7;i++)
    {
        if(p_data[p_send_tail].idata[i] != p_can.data[i + 1])
            return;
    }
    pRetry = 0;
    p_mili = time_us_64() + 1001LL;
    p_send_tail = ringbuff_adder(p_send_tail,1);
}
void ServerCAN::handle_message()
{
    if(p_can.id < C_Server_ADDRESS_CAN)
        return;
    if(p_can.data[C_DSC_ARRAY_OPCODE] >= C_DSC_OPCODE_ADD_VALUE_ACK)
    {
        handle_ack();
        return;
    }
    p_rx[p_rx_head].iOpCode = p_can.data[C_DSC_ARRAY_OPCODE];
    p_rx[p_rx_head].imotNO = p_can.id - C_Server_ADDRESS_CAN;
    copy_uint8(p_rx[p_rx_head].idata,&p_can.data[C_DSC_ARRAY_OPCODE + 1],7);
    p_rx_head = ringbuff_adder(p_rx_head,1);
}
void ServerCAN::run()
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
    if(pRetry > c_MAX_Retry)
    {
        p_send_tail = ringbuff_adder(p_send_tail,1);
        pRetry = 0;
    }
    p_mili = time_us_64() + (c_interval * 5);
    p_can.header.length = 8;
    p_can.header.rtr = 0;
    p_can.id = ((p_data[p_send_tail].imotNO) & 0xFFULL);
    p_can.data[C_DSC_ARRAY_OPCODE] = p_data[p_send_tail].iOpCode;
    if(p_can.id != C_DSC_BRODCAST_ADDRESS_CAN)
        p_can.data[C_DSC_ARRAY_OPCODE] += C_DSC_OPCODE_ADD_VALUE_ACK;
    copy_uint8(&p_can.data[1],p_data[p_send_tail].idata,7);
    if(mcp2515_send_message(&p_can) == false)
        return;
    pRetry++;
    if(p_can.data[C_DSC_ARRAY_OPCODE] < C_DSC_OPCODE_ADD_VALUE_ACK)
        p_send_tail = ringbuff_adder(p_send_tail,1);
}
int8_t ServerCAN::ringbuff_adder(int8_t RbIndex, int8_t iVal)
{
    RbIndex += iVal;
    if(RbIndex < 0)
        RbIndex = c_buff_size;
    if(RbIndex > c_buff_size)
        RbIndex = 0;
    return RbIndex;
}
void ServerCAN::ptr8_to_int32(uint8_t *ptrdata,int32_t *iout)
{
    *iout = 0;
	ptrdata += 3;
    for(uint i=0;i<4;i++)
    {
        *iout = (uint32_t)((*iout)<<8LL);
        *iout |= (uint8_t)(*ptrdata);
	    *ptrdata--;
    }

}
void ServerCAN::int32_to_ptrint8(int32_t iVal,uint8_t *ptrdata)
{
    for(uint i=0;i<4;i++)
    {
        *ptrdata = (uint8_t)(iVal & 0xFFLL);
        ptrdata++;
        iVal = (iVal>>8LL);
    }
}
bool ServerCAN::_bv(uint32_t iVal,uint8_t iBV)
{
    if(iVal & (1ULL<<iBV))
        return true;
    return false;
}
void ServerCAN::copy_uint8(uint8_t *dest,uint8_t *source,uint8_t count)
{
    for(uint i=0;i<count;i++)
    {
        *dest = *source;
        dest++;
        source++;
    }
}