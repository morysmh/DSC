
#include "define.h"


class StepMotor 
{
public:
    StepMotor(  uint8_t i_pinPulse,
                uint8_t i_pinDir,
                uint8_t i_pinDiv1,
                uint8_t i_pinDiv2,
                uint8_t i_pinDiv3,
                uint8_t i_pinEN,
                uint8_t i_pinSleep);
    ~StepMotor(){};
    void run();
    void set_div(uint8_t i_div);
    void set_PID(int32_t i_P,int32_t i_I,int32_t i_D);
    void set_togo_Location(int32_t i_togo);
    void set_move_command_togo(int32_t i_togo);
    void set_move_command_togo_releative(int32_t i_togo);
    void set_current_location(int32_t i_CL){p_current_location = i_CL;}
    void set_default_direction(bool i_def){p_def_direction = i_def;}
    void set_low_us(int32_t i_val);
    void set_MAX_us(int32_t i_val);
    void pid_enable(){p_pid_en = true;}
    void pid_disable(){p_pid_en = false;}
    void enable();
    void disable();
    bool isMoving();
    bool is_pulse_available();
    int8_t get_pulse();
    void stop();
    void start_moving();
    void lock_motor(bool val){p_LockMotor = val;}
    void set_EncoderRes(uint16_t inm){pResEncoder = inm/10; cMotorMovingRes = inm;};
    int32_t get_MinDelay() const {return p_low_us_delay;}
    int32_t get_DriverRes() const {return p_DriveRes;}

private:
    int32_t p_current_location = 0;
    int32_t p_step_Current_us_delay = 100;
    void software_pulse(int8_t i_pulse);
    void init_pid();
    void calc_Pid();
    void change_direction();
    int32_t p_togo_Location = 0,p_wait_start_command_togo = 0;
    int64_t p_interval_PID_us = 2000;
    int32_t p_max_us_delay = 10000;
    int32_t p_low_us_delay = 15;
    uint8_t p_pinPulse;
    uint8_t p_pinDir;
    uint8_t p_pinDiv1;
    uint8_t p_pinDiv2;
    uint8_t p_pinDiv3;
    uint8_t p_pinEN;
    uint8_t p_pinSleep;
    int32_t p_DriveRes = 1;
    bool p_is_config = false;
    bool p_motor_en = false;
    bool p_LockMotor = false;

    bool p_pid_en = false;
    int32_t e0,e1,e2;
    int32_t kp = 0,ki = 0,kd = 0;
    int32_t k1,k2,k3;
    int32_t res_pid;
    const int32_t c_pid_Max = 10000LL,c_pid_interval_us = 10000LL;
    int32_t p_max_minus_low = 500;
    int64_t p_mili = 0,p_mili_pid = 0;
    bool p_stat_clk = false;
    bool p_stat_dir = false;
    bool p_def_direction = true;
    int32_t p_sofware_pulse = 0;
    uint16_t pResEncoder = 10;
    int32_t cMotorMovingRes = 0;
};