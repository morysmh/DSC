#include "clientmotor.h"

clientmotor::clientmotor(uint8_t add,CanCotroll *ptrcan)
{
    p_address = add;
    ptr_can = ptrcan;
    c_interval_location_read_us = 823241LL + (p_address * 25000LL);
    if(p_address == C_DSC_BRODCAST_ADDRESS_CAN)
        return;
    if(p_address <= 0)
        p_address = 0;
    if(p_address >= 8)
        p_address = 100;
}

int32_t clientmotor::get_location() const
{
    return p_current;
}

void clientmotor::set_div(int32_t i_div)
{
    if(i_div == p_div)
        return;
    switch (p_div)
    {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
        p_div = i_div;
        ptr_can->send_data(C_DSC_STEP_MOTOR_SET_DIV,p_div,p_address);
        break;
    default:
        break;
    }
    
}

void clientmotor::set_encoder_nmPP(int32_t i_nm)
{
    if(i_nm <= 0)
        return;
    if(i_nm == p_encoder_nm)
        return;
    p_encoder_nm = i_nm;
    ptr_can->send_data(C_DSC_ENCODER_RESOLATION,p_encoder_nm,p_address);
}
void clientmotor::set_encoder_dir(bool i_val)
{
    if(p_def_encoder_direction == i_val)
        return;
    p_def_encoder_direction = i_val;
    ptr_can->send_data(C_DSC_ENCODER_DIRECTION,p_def_encoder_direction,p_address);
}
void clientmotor::enable_encoder()
{
    ptr_can->send_data(C_DSC_ENCODER_HARDWARE_ENABLE,1,p_address);
}
void clientmotor::disable_encoder()
{
    ptr_can->send_data(C_DSC_ENCODER_HARDWARE_DISABLE,1,p_address);
}
void clientmotor::stop()
{
    p_homeStat = normal_operation;
    ptr_can->send_data(C_DSC_STEP_MOTOR_STOP,1,p_address);
}
void clientmotor::enable()
{
    ptr_can->send_data(C_DSC_STEP_MOTOR_ENABLE,1,p_address);

}
void clientmotor::disable()
{
    ptr_can->send_data(C_DSC_STEP_MOTOR_DISABLE,1,p_address);
}
void clientmotor::pid_enable()
{
    ptr_can->send_data(C_DSC_STEP_MOTOR_PID_ENABLE,1,p_address);
}
void clientmotor::pid_disable()
{
    ptr_can->send_data(C_DSC_STEP_MOTOR_PID_DISABLE,1,p_address);
}
void clientmotor::set_motor_dir(bool i_dir)
{
    if(p_def_motor_directin == i_dir)
        return;
    p_def_motor_directin = i_dir;
    ptr_can->send_data(C_DSC_STEP_MOTOR_SET_Default_Direction,p_def_motor_directin,p_address);
}
void clientmotor::set_pid_val(int32_t i_kp,int32_t i_ki,int32_t i_kd)
{
    if(i_kp != kp)
    {
        kp = i_kp;
        ptr_can->send_data(C_DSC_STEP_MOTOR_SET_KP,kp,p_address);
    }
    if(i_ki != ki)
    {
        ki = i_ki;
        ptr_can->send_data(C_DSC_STEP_MOTOR_SET_KI,ki,p_address);
    }
    if(i_kd != kd)
    {
        kd = i_kd;
        ptr_can->send_data(C_DSC_STEP_MOTOR_SET_KD,kd,p_address);
    }
}
void clientmotor::immidiate_absoulute_move(int32_t i_pos)
{
    p_togo = i_pos;
    p_togo /= p_encoder_nm;
    p_togo *= p_encoder_nm;
    ptr_can->send_data(C_DSC_STEP_MOTOR_SET_TOGO_Location,p_togo,p_address);
}
void clientmotor::set_absolute_position(int32_t i_pos)
{
    if(p_homeStat != normal_operation)
    {
        p_homeStat = normal_operation;
        set_low_us(p_def_low_us);
    }
    p_togo = i_pos;
    p_togo /= p_encoder_nm;
    p_togo *= p_encoder_nm;
    ptr_can->send_data(C_DSC_STEP_MOTOR_TOGO_ON_COMMAND,p_togo,p_address);
}
void clientmotor::start_moving()
{
    ptr_can->send_data(C_DSC_STEP_MOTOR_START_MOVING,1,p_address);
}
void clientmotor::set_releative_position(int32_t i_pos)
{
    p_togo += i_pos;
    set_absolute_position(p_togo);
}
void clientmotor::home_motor()
{
    if(p_homeStat == normal_operation)
        p_homeStat = sensor_stat_check;
}
void clientmotor::home_sequencer()
{
    if((p_mili_home_sensor_refresh < time_us_64()) && (p_homeStat != normal_operation))
    {
        p_mili_home_sensor_refresh = time_us_64() + 2012431LL;
        ptr_can->send_data(C_DSC_SENSOR_BOTTOM_READ_STATUS,1,p_address);
    }
    switch (p_homeStat)
    {
    case normal_operation:
        break;
    case sensor_stat_check:
        p_mili_homeSensor = time_us_64() + 500000LL;
        p_homeStat = wait_for_sensor_reply;
        ptr_can->send_data(C_DSC_SENSOR_BOTTOM_READ_STATUS,1,p_address);
        break;
    case wait_for_sensor_reply:
        if (p_mili_homeSensor > time_us_64())
            break;
        p_homeStat = Home_Fast;
        break;
    case Home_Fast:
        if(p_homeSensor_Stat != true)
            immidiate_absoulute_move(-400000000LL);
        p_homeStat = waitTo_Home;
        break;
    case waitTo_Home:
        if(p_homeSensor_Stat != true)
            break;
        p_homeStat = wait_to_500um;
        immidiate_absoulute_move(50000LL);
        break;
    case wait_to_500um:
        if(is_near() == false)
            break;
        p_homeStat = Home_Slowly;
        set_low_us(p_high - 20LL);
        immidiate_absoulute_move(-400000000LL);
        p_mili_homeSensor = time_us_64() + 500000LL;
        break;
    case Home_Slowly:
        if (p_mili_homeSensor > time_us_64())
            break;
        if(p_homeSensor_Stat != true)
            break;
        set_low_us(p_def_low_us);
        p_homeStat = read_sensor_again;
        break;
    case read_sensor_again:
        if(p_homeSensor_Stat != true)
            break;
        p_mili_homeSensor = time_us_64() + 500000LL;
        ptr_can->send_data(C_DSC_SENSOR_BOTTOM_READ_STATUS,1,p_address);
        p_homeStat = wait_for_readBack;
        break;
    case wait_for_readBack:
        if (p_mili_homeSensor > time_us_64())
            break;
        p_homeStat = home_position;
        break;
    case home_position:
        if(p_homeSensor_Stat == true)
            break;
        p_homeStat = normal_operation;
        break;
    default:
        break;
    }
}

bool clientmotor::is_near()
{
    int32_t tmp = p_current - p_togo;
    if(tmp < 0)
        tmp *= -1;
    if(tmp <= (2 * p_encoder_nm))
        return true;
    return false;
}
void clientmotor::run()
{
    read_pos();
    home_sequencer();
}
void clientmotor::get_can_message(int8_t i_para,int32_t i_val,uint8_t motNO)
{
    if(motNO == p_otherMotSenor)
    {
        if((i_para == C_DSC_SENSOR_TOP_READ_STATUS) && (i_val == 1))
        {
            p_read_loc_mili = 1;
            stop();
            read_pos();
            p_topSensorTrig = true;
        }
    }
    if(motNO != p_address)
        return;
    switch (i_para)
    {
    case C_DSC_SENSOR_BOTTOM_READ_STATUS:
        p_homeSensor_Stat = !!i_val;
        if(i_val == 0)
            break;
        p_togo = 0;
        p_current = 0;
        break;

    case C_DSC_ENCODER_LOCATION:
        p_current = i_val;
        if(p_topSensorTrig == true)
        {
            p_topSensorTrig = false;
            p_togo = i_val;
        }
        break;
    
    default:
        break;
    }
}

void clientmotor::set_stop_with_other_motor_sensor(uint8_t i_motNO)
{
    if(i_motNO <= 0)
        return;
    if(i_motNO > 8)
        return;
    p_otherMotSenor = i_motNO;
}
void clientmotor::read_pos()
{
    if (p_read_loc_mili > time_us_64())
        return;
    p_read_loc_mili = time_us_64() + c_interval_location_read_us;
    ptr_can->send_data(C_DSC_ENCODER_LOCATION,1,p_address);
}
void clientmotor::set_low_us(int32_t i_speed)
{
    p_low = i_speed;
    ptr_can->send_data(C_DSC_STEP_MOTOR_SET_low_us,p_low,p_address);
}
void clientmotor::set_max_us(int32_t i_speed)
{
    p_high = i_speed;
    ptr_can->send_data(C_DSC_STEP_MOTOR_SET_max_us,p_high,p_address);
}
void clientmotor::default_low_us(int32_t i_speed)
{
    if(i_speed <= 5)
        i_speed = 5;
    p_def_low_us = i_speed;
}
void clientmotor::set_sensor_bottom_normal_stat(bool i_val)
{
    ptr_can->send_data(C_DSC_SENSOR_BOTTOM_DEFAULT_VALUE,i_val,p_address);
}
void clientmotor::set_sensor_top_normal_stat(bool i_val)
{
    ptr_can->send_data(C_DSC_SENSOR_TOP_DEFAULT_VALUE,i_val,p_address);
}