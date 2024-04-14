#include "can.h"
void CanCotroll::handl_ACK()
{
    p_canSend.id = C_DSC_Server_ADDRESS_CAN + p_address;
    p_canSend.header.length = p_canRX.header.length;
    p_canSend.header.rtr = 0;
    for(uint8_t i=0;i<p_canRX.header.length;i++)
    {
        p_canSend.data[i] = p_canRX.data[i];
    }
}
void CanCotroll::handle_message()
{
    if(p_address == 0)
        return;
    if(p_canRX.data[C_DSC_ARRAY_OPCODE] >= C_DSC_OPCODE_ADD_VALUE_ACK)
    {
        CanBusyACK = true;
        handl_ACK();
        p_canRX.data[C_DSC_ARRAY_OPCODE] -= C_DSC_OPCODE_ADD_VALUE_ACK;
    }
    p_rx[p_rx_head].iPara = p_canRX.data[C_DSC_ARRAY_OPCODE];
    uint8_t *ptrRX,*ptrCAN;
    ptrRX = p_rx[p_rx_head].idata;
    ptrCAN = &p_canRX.data[C_DSC_ARRAY_OPCODE + 1];
    for(uint i=0;i<7;i++)
    {
        *ptrRX = *ptrCAN;
        ptrRX++;
        ptrCAN++;
    }
    p_rx_head = ringbuff_adder(p_rx_head,1);
}
void CanCotroll::handle_other_sensor()
{
    if(p_other_sensor_address == 0)
        return;
    if(p_canRX.data[C_DSC_ARRAY_OPCODE] != C_DSC_OPCODE_REG_STATUS)
        return;
    handle_Self_OtherSensor_Status(p_canRX.data[C_DSC_ARRAY_STATUS_REPORT_1byte + 1]);
}
void CanCotroll::handle_Self_OtherSensor_Status(uint8_t status)
{
    if(status == pOtherMotorStat)
        return;
    pOtherMotorStat = status;
    pOtherSensorStatChange = true;
}
bool CanCotroll::read_other_motor_stat(uint8_t *istat)
{
    if(pOtherSensorStatChange == false)
        return false;
    *istat = pOtherMotorStat;
    return true;
}
void CanCotroll::send_status(int32_t iLocation,bool isMoving, bool iSensorBottom,bool iSensorTOP,bool iSendNOW)
{
    uint8_t *ptrSend;
    if(iSendNOW == true)
        pt_interval = time_us_64();
    if(pt_interval > time_us_64())
        return;
    pt_interval = time_us_64() + c_interval_status;
    if(isMoving == false)
        pt_interval = time_us_64() + (c_interval_status * 5);
    rStatusAvailable = true;
    p_status.header.rtr = 0;
    p_status.header.length = 6;
    int32_to_ptrint8(iLocation,(uint8_t *)&p_status.data[C_DSC_ARRAY_STATUS_LOCATION_index + 1]);
    p_status.data[C_DSC_ARRAY_OPCODE] = C_DSC_OPCODE_REG_STATUS;
    ptrSend = &p_status.data[C_DSC_ARRAY_STATUS_REPORT_1byte + 1];
    *ptrSend = 0;
    if(iSensorBottom)
        *ptrSend |= set_bv(C_DSC_BIT_STATUS_SENSOR_BOTTOM_STATUS);
    if(iSensorTOP)
        *ptrSend |= set_bv(C_DSC_BIT_STATUS_SENSOR_TOP_STATUS);
    if(isMoving)
        *ptrSend |= set_bv(C_DSC_BIT_STATUS_MOTOR_MOVING);
    p_status.id = C_DSC_Server_ADDRESS_CAN + p_address;
}
uint32_t CanCotroll::set_bv(uint8_t iBv)
{
    if(iBv >= 32)
        return 0;
    return (1ULL<<iBv);
}
bool CanCotroll::read_new_message(uint8_t *ptrdata,uint8_t *motNO,uint8_t *iPara)
{
    if(p_rx_head == p_rx_tail)
        return false;
    uint8_t *ptrRX;
    ptrRX = p_rx[p_rx_tail].idata;
    for(uint i=0;i<7;i++)
    {
        *ptrdata = *ptrRX;
        ptrRX++;
        ptrdata++;
    }
    *motNO = p_rx[p_rx_tail].imotNO;
    *iPara = p_rx[p_rx_tail].iPara;
    p_rx_tail = ringbuff_adder(p_rx_tail,1);
    return true;
}
void CanCotroll::set_interval(uint8_t iVal)
{
    c_interval_status = (uint64_t)iVal * 4000ULL;
    c_interval_status += 15123ULL;
}
void CanCotroll::run()
{
    if(mcp2515_check_message())
    {
        mcp2515_get_message(&p_canRX);
        if(p_canRX.id == p_address)
            handle_message();
        if(p_canRX.id == p_other_sensor_address)
            handle_other_sensor();
    }
    if((CanBusyACK == false) && (rStatusAvailable == false))
        return;
    if(mcp2515_check_free_buffer() == false)
        return;
    if(rStatusAvailable)
    {
        rStatusAvailable = (mcp2515_send_message(&p_status)) ? false : rStatusAvailable;
        if(p_status.id == p_other_sensor_address)
            handle_Self_OtherSensor_Status(p_status.data[C_DSC_ARRAY_STATUS_REPORT_1byte + 1]);
        return;
    }
    if(CanBusyACK)
    {
        CanBusyACK = (mcp2515_send_message(&p_canSend)) ? false : CanBusyACK;
        return;
    }
}
CanCotroll::CanCotroll(uint8_t i_add)
{
    p_address = i_add;
}
void CanCotroll::set_other_sensor_code(uint8_t iOtherMotor)
{
    p_other_sensor_address = iOtherMotor + C_Server_ADDRESS_CAN;
}
uint8_t CanCotroll::ringbuff_adder(uint8_t rindx, int8_t iVal)
{
    int16_t RbIndex;
    RbIndex = rindx;
    RbIndex += iVal;
    if(RbIndex < 0)
        RbIndex = c_buff_size;
    if(RbIndex > c_buff_size)
        RbIndex = 0;
    return RbIndex;
}
void CanCotroll::ptr8_to_int32(uint8_t *ptrdata,int32_t *iout)
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
void CanCotroll::int32_to_ptrint8(int32_t iVal,uint8_t *ptrdata)
{
    for(uint i=0;i<4;i++)
    {
        *ptrdata = (uint8_t)(iVal & 0xFFLL);
        ptrdata++;
        iVal = (iVal>>8LL);
    }
}
