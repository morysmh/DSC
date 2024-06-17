#include "motor.h"

bool StepMotor::isMoving()
{
    int32_t rdiff = p_togo_Location - p_current_location;
    if(rdiff < 0)
        rdiff = rdiff * (-1);
    if(rdiff <= cMotorMovingRes)
        return false;
    return true;
}
bool StepMotor::is_pulse_available()
{
    if(p_sofware_pulse)
        return true;
    return false;
}
int8_t StepMotor::get_pulse()
{
    int32_t res = p_sofware_pulse;
    p_sofware_pulse = 0;
    return res;
}
void StepMotor::software_pulse(int8_t i_pulse)
{
    p_sofware_pulse += i_pulse;
}
void StepMotor::change_direction()
{
    p_stat_dir = true;
    if((p_current_location - p_togo_Location) > 0)
        p_stat_dir = false;
    if(p_def_direction)
    {
        gpio_put(p_pinDir,p_stat_dir);
        if((p_stat_clk) && (p_stat_dir))software_pulse(1);
        if((p_stat_clk) && (!p_stat_dir))software_pulse(-1);
        return;
    }
    gpio_put(p_pinDir,!p_stat_dir);
    if((p_stat_clk) && (p_stat_dir))software_pulse(1);
    if((p_stat_clk) && (!p_stat_dir))software_pulse(-1);

}
void StepMotor::run()
{
    if(p_motor_en == false)
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
    if(i_val > 200000ULL) i_val = 200000ULL;
    p_max_us_delay = i_val;
    p_max_minus_low  = p_max_us_delay - p_low_us_delay;
}
void StepMotor::set_low_us(int32_t i_val)
{
    if(i_val < 10) i_val = 10;
    p_low_us_delay = i_val;
    p_max_minus_low  = p_max_us_delay - p_low_us_delay;
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
                    uint8_t i_pinDiv3,
                    uint8_t i_en,
                    uint8_t i_sleep)
{
    p_pinPulse = i_pinPulse;
    p_pinDir   = i_pinDir;
    p_pinDiv1  = i_pinDiv1;
    p_pinDiv2  = i_pinDiv2;
    p_pinDiv3  = i_pinDiv3;
    p_pinEN  = i_en;
    p_pinSleep  = i_sleep;
    set_div(16);
    disable(); 
    set_MAX_us(10000L);
    set_low_us(8);

}
void StepMotor::set_div(uint8_t i_div)
{
    p_DriveRes = i_div;
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

void StepMotor::enable()
{
    gpio_put(p_pinEN,false);
    p_motor_en = true;

}
void StepMotor::disable()
{
    gpio_put(p_pinEN,true);
    p_motor_en = false;
}
void StepMotor::calc_Pid()
{
    int64_t r_tmp;
    if(p_pid_en == false)
    {
        p_step_Current_us_delay = p_low_us_delay;
        return;
    }
    e2 = e1;
    e1 = e0;
    e0 = p_togo_Location - p_current_location;
    e0 /= 50;
    if(e0 < 0)
        e0 *= -1;
    if(e0 > 100)
        e0 = 100;
    if(e0 < -50)
        e0 = -50;
    e0 -= 50;
    res_pid = res_pid +(k1 * e0)+(k2 * e1)+(k3 * e2);
    if(res_pid <= 0)
        res_pid = 0;
    if(res_pid > c_pid_Max)
        res_pid = c_pid_Max;
    r_tmp = (int64_t)p_max_minus_low * (int64_t)res_pid;
    r_tmp /= (int64_t)c_pid_Max;
    p_step_Current_us_delay = p_max_us_delay - r_tmp;
}
void StepMotor::set_togo_Location(int32_t i_togo)
{
    if(p_LockMotor == true)
        return;
    i_togo /= pResEncoder;
    i_togo *= pResEncoder;
    p_togo_Location = i_togo;
}
void StepMotor::set_move_command_togo(int32_t i_togo)
{
    p_wait_start_command_togo = i_togo;
}
void StepMotor::set_move_command_togo_releative(int32_t i_togo)
{
    p_wait_start_command_togo = (p_current_location + i_togo);
}
void StepMotor::stop()
{
    p_togo_Location = p_current_location;
    p_wait_start_command_togo = p_current_location;
}
void StepMotor::start_moving()
{
    set_togo_Location(p_wait_start_command_togo);
}
