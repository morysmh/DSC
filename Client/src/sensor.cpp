#include "sensor.h"

Sensor::Sensor(uint8_t i_pin,Sensor::SensorType i_sen)
{
    if(i_pin < 29)
        p_pinNO = i_pin;
    p_sensorType = i_sen;
    
}
bool Sensor::get_sensor_stat() const
{
    if(!p_normal_stat)
        return !p_current_stat;
    return p_current_stat;
}
bool Sensor::is_triged() 
{
    if(!p_is_trig)
        return false;
    p_is_trig = false;
    return true;
}
void Sensor::run()
{
    if(p_pinNO <= 0)
        return;
    if (p_fast_run > time_us_64())
        return;
    p_fast_run = time_us_64() + c_fast_run_prevent;
    if(p_current_stat == gpio_get(p_pinNO))
    {
        p_change_mili = time_us_64() + c_hyst_us;
        return;
    }
    if(p_change_mili < time_us_64())
    {
        p_current_stat = gpio_get(p_pinNO);
        p_is_trig = true;
    }
}