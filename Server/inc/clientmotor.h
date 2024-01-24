#include "define.h"
#include "canserver.h"
#ifndef __MOTOR__CLASS__
#define __MOTOR__CLASS__
class clientmotor
{
public:
    clientmotor(uint8_t add,CanCotroll *ptrcan);
    void set_sensor_bottom_normal_stat(bool i_val);
    void set_sensor_top_normal_stat(bool i_val);
    void set_div(int32_t i_div);
    void set_low_us(int32_t i_speed);
    void set_max_us(int32_t i_speed);
    void set_encoder_nmPP(int32_t i_nm);
    void set_encoder_dir(bool i_val);
    void enable_encoder();
    void disable_encoder();
    void stop();
    void enable();
    void disable();
    void pid_enable();
    void pid_disable();
    void start_moving();
    void set_motor_dir(bool i_dir);
    void set_pid_val(int32_t i_kp,int32_t i_ki,int32_t i_kd);
    void set_absolute_position(int32_t i_pos);
    void set_releative_position(int32_t i_pos);
    void get_can_message(int8_t i_para,int32_t i_val,uint8_t motNO);
    void set_stop_with_other_motor_sensor(uint8_t i_motNO);
    bool is_moving(){return (!(is_near()));}
    int32_t get_location() const;
    void default_low_us(int32_t i_speed);
    bool is_home() const{return (p_homeStat == home_position);}
    void home_motor();
    void run();
private:
    enum {
        normal_operation,
        sensor_stat_check,
        wait_for_sensor_reply,
        Home_Fast,
        waitTo_Home,
        wait_to_500um,
        Home_Slowly,
        read_sensor_again,
        wait_for_readBack,
        home_position
    }Homing_enum;
    void immidiate_absoulute_move(int32_t i_pos);
    void home_sequencer();
    void read_pos();
    bool is_near();
    int8_t p_homeStat = 0;
    int8_t p_otherMotSenor = 102;
    int32_t p_encoder_nm = 500;
    uint8_t p_address = 0;
    int16_t p_div = 16;
    int32_t p_def_low_us = 50;
    int32_t p_low = 50,p_high = 1000;
    int32_t kp = 0,ki = 0,kd = 0;
    int32_t p_togo = 0,p_current = 0;
    int64_t p_mili_homeSensor = 0;
    int64_t p_mili_home_sensor_refresh = 0;
    bool p_topSensorTrig = false;
    bool p_def_motor_directin = false;
    bool p_motor_en = false;
    bool p_pid_en = false;
    
    bool p_sensor_top_en = false;
    bool p_sensor_bottom_en = true;
    
    bool p_def_encoder_direction = true;
    bool p_encoder_en = false;
    CanCotroll *ptr_can;
    int64_t p_read_loc_mili = 0;
    int64_t c_interval_location_read_us =  1243106LL;
    bool p_homeSensor_Stat = false;
};

#endif