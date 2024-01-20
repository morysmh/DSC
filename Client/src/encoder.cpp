#include "encoder.h"

Encoder::Encoder(uint8_t i_A,int64_t i_nm_per_pulse)
{
    static bool p_if_object_creat = false;
    if(i_A > 29)
        return;
    if(p_if_object_creat)
        return;
    p_if_object_creat = true;
    p_nm_pp = i_nm_per_pulse;
}

int64_t Encoder::get_location() const
{
    if(p_direction)
        return(((p_pulse - p_zero_val) * p_nm_pp) * (-1));
    return((p_pulse - p_zero_val) * p_nm_pp);
}
void Encoder::set_nm_pp(int64_t i_nm_per_pulse)
{
    if(i_nm_per_pulse <= 10)
        i_nm_per_pulse = 10;
    p_nm_pp = i_nm_per_pulse ;
}