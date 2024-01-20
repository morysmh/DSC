
#include "define.h"


class StepMotor 
{
public:
    StepMotor(  uint8_t i_pinPulse = PIN_TMC__CLK,
                uint8_t i_pinDir = PIN_TMC__DIR,
                uint8_t i_pinDiv1 = PIN_SPI_MOSI,
                uint8_t i_pinDiv2 = PIN_SPI__SCK,
                uint8_t i_pinDiv3 = PIN_SPI__Csn);
    ~StepMotor(){};
    void run();
    void set_div(uint8_t i_div);
    void set_PID(int32_t i_P,int32_t i_I,int32_t i_D);
    void set_togo_Location(int32_t i_togo);
    void set_current_location(int32_t i_CL){p_current_location = i_CL;}
    void set_default_direction(bool i_def){p_def_direction = i_def;}
    void set_low_us(int32_t i_val);
    void set_MAX_us(int32_t i_val);

private:
    int32_t p_current_location = 0;
    int32_t p_step_Current_us_delay = 100;
    void init_pid();
    void calc_Pid();
    void change_direction();
    int32_t p_togo_Location = 0;
    int64_t p_interval_PID_us = 2000;
    int32_t p_max_us_delay = 1000;
    int32_t p_low_us_delay = 100;
    uint8_t p_pinPulse;
    uint8_t p_pinDir;
    uint8_t p_pinDiv1;
    uint8_t p_pinDiv2;
    uint8_t p_pinDiv3;

    bool p_pid_en = false;
    int32_t e0,e1,e2;
    int32_t kp = 0,ki = 0,kd = 0;
    int32_t k1,k2,k3;
    int32_t res_pid;
    const int32_t c_pid_Max = 10000;
    int64_t p_mili = 0,p_mili_pid = 0;
    bool p_stat_clk = false;
    bool p_stat_dir = false;
    bool p_def_direction = true;
};