#include "motor.h"

void StepMotor::change_direction()
{
    p_stat_clk = true;
    if(p_current_location - p_togo_Location)
        p_stat_dir = false;
    if(p_def_direction)
        gpio_put(p_pinDir,p_stat_dir);
    else
        gpio_put(p_pinDir,!p_stat_dir);
}
void StepMotor::run()
{
    if(p_is_config == false)
        return;
    if(p_mili > time_us_64())
        return;
    calc_Pid();
    p_mili = (int64_t)p_step_Current_us_delay + time_us_64();
    if(p_togo_Location == p_current_location)
        return;

    p_stat_clk = !p_stat_clk;
    change_direction();
    gpio_put(p_pinPulse, p_stat_clk);
}
void StepMotor::set_MAX_us(int32_t i_val)
{
    if(i_val > 200000ULL) i_val = 20000ULL;
    p_max_us_delay = i_val;
}
void StepMotor::set_low_us(int32_t i_val)
{
    if(i_val < 30) i_val = 30;
    p_low_us_delay = i_val;
}
void StepMotor::set_PID(int32_t i_kp,int32_t i_ki,int32_t i_kd)
{
    if(i_kp) kp = i_kp;
    if(i_ki) ki = i_ki;
    if(i_kd) kd = i_kd;
    init_pid();
}
StepMotor::StepMotor(uint8_t i_pinPulse,
                    uint8_t i_pinDir,
                    uint8_t i_pinDiv1,
                    uint8_t i_pinDiv2,
                    uint8_t i_pinDiv3)
{
    p_pinPulse = i_pinPulse;
    p_pinDir   = i_pinDir;
    p_pinDiv1  = i_pinDiv1;
    p_pinDiv2  = i_pinDiv2;
    p_pinDiv3  = i_pinDiv3;
    p_is_config = true;
    set_div(16);

}
void StepMotor::set_div(uint8_t i_div)
{
    switch (i_div)
    {
    case 1:
        gpio_put(p_pinDiv1, 0);
        gpio_put(p_pinDiv2, 0);
        gpio_put(p_pinDiv3, 0);
        break;
    case 2:
        gpio_put(p_pinDiv1, 1);
        gpio_put(p_pinDiv2, 0);
        gpio_put(p_pinDiv3, 0);
        break;
    case 4:
        gpio_put(p_pinDiv1, 0);
        gpio_put(p_pinDiv2, 1);
        gpio_put(p_pinDiv3, 0);
        break;
    case 8:
        gpio_put(p_pinDiv1, 1);
        gpio_put(p_pinDiv2, 1);
        gpio_put(p_pinDiv3, 0);
        break;
    case 16:
        gpio_put(p_pinDiv1, 1);
        gpio_put(p_pinDiv2, 1);
        gpio_put(p_pinDiv3, 1);
        break;
    default:
        gpio_put(p_pinDiv1, 1);
        gpio_put(p_pinDiv2, 1);
        gpio_put(p_pinDiv3, 1);
        break;
    }

}


void StepMotor::init_pid()
{
    k1 = kp + ki + kd;
    k2 = (kp * (-1)) - (2 * kd);
    k3 = kd;
    e0 = 0;
    e1 = 0;
    e2 = 0;
    res_pid = 0;
}

void StepMotor::calc_Pid()
{
    if(p_pid_en == false)
    {
        p_step_Current_us_delay = p_low_us_delay;
        return;
    }
    e2 = e1;
    e1 = e0;
    e0 = p_togo_Location - p_current_location;
    if(e0 < 0) e0 *= (-1);
    
    if(e0 > 512)
        e0 = 512;
    if(e0 < -512)
        e0 = -512;
    
    res_pid = res_pid +(k1 * e0)+(k2 * e1)+(k3 * e2);
    
    if(res_pid <= 0)
        res_pid = 0;
    if(res_pid > c_pid_Max)
        res_pid = c_pid_Max;
    p_step_Current_us_delay = p_max_us_delay - (((p_max_us_delay - p_low_us_delay) * res_pid) / c_pid_Max);
}
void StepMotor::set_togo_Location(int32_t i_togo)
{
    p_togo_Location = i_togo;
    p_mili = 0;
}