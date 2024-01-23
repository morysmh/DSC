#include "encoder.h"

Encoder::Encoder(uint8_t i_A,int32_t i_nm_per_pulse)
{
    static bool p_if_object_creat = false;
    if(i_A > 29)
        return;
    if(p_if_object_creat)
        return;
    p_if_object_creat = true;
    set_nm_pp(i_nm_per_pulse);
}

bool Encoder::is_change() 
{
    if (p_is_change)
    {
        p_is_change = false;
        return true;
    }
    
    return false;
}
void Encoder::set_software_inc(int8_t i_in)
{
    if(p_software_encoder == false)
        return;
    p_pulse += i_in;
    p_is_change = true;
}
void Encoder::set_hardware_inc(int8_t i_in)
{
    if(p_software_encoder == true)
        return;
    p_pulse += i_in;
    p_is_change = true;
}
int32_t Encoder::get_location() const
{
    if(p_software_encoder == true)
        return((p_pulse - p_zero_val) * p_nm_pp);
    if(p_direction)
        return(((p_pulse - p_zero_val) * p_nm_pp) * (-1));
    return((p_pulse - p_zero_val) * p_nm_pp);
}
void Encoder::set_nm_pp(int32_t i_nm_per_pulse)
{
    if(i_nm_per_pulse <= 10)
        i_nm_per_pulse = 10;
    p_nm_pp = i_nm_per_pulse / 10;
}