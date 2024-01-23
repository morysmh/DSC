#include "client.h"

void Client::write_to_can_bus(Enum_DSC i_para,int32_t i_val)
{
    write_to_can((int32_t)i_para,i_val);
}
void Client::write_to_can(int32_t i_para,int32_t i_val)
{
    o_can->send_data(i_para,i_val);
}
Client::Client(
            uint8_t pin_clk,
            uint8_t pin_dir,
            uint8_t pin_Bottom_Sensor,
            uint8_t pin_TOP_Sensor,
            uint8_t pin_Encoder_A,
            uint8_t pin_DIV1,
            uint8_t pin_DIV2,
            uint8_t pin_DIV3,
            uint8_t pin_en,
            uint8_t pin_sleep,
            uint8_t i_add
            )
{
    o_can = new CanCotroll(i_add);
    o_encoder = new Encoder(pin_Encoder_A,5000);
    o_stepmotor = new StepMotor(pin_clk,pin_dir,pin_DIV1,pin_DIV2,pin_DIV3,pin_en,pin_sleep);
    o_top = new Sensor(pin_TOP_Sensor,Sensor::END_Sensor);
    o_stepmotor->set_PID(10,1,0);
    o_bottom = new Sensor(pin_Bottom_Sensor);
    o_bottom->set_normal_stat(true);
    o_top->set_normal_stat(true);
    o_bottom->enable();
    o_top->enable();
}
void Client::increse_sofware_encoder(int8_t i_val)
{
    o_encoder->set_hardware_inc(i_val);
}
Client::~Client()
{
    delete o_can;
    delete o_encoder;
    delete o_stepmotor;
    delete o_top;
    delete o_bottom;
}


void Client::run()
{
    check_message();
    if(o_stepmotor->is_pulse_available())
    {
        o_encoder->set_software_inc(o_stepmotor->get_pulse());
    }
    if(o_encoder->is_change())
    {
        o_stepmotor->set_current_location(o_encoder->get_location());
    }
    o_can->run();
    o_stepmotor->run();
    o_top->run();
    o_bottom->run();
    if(o_bottom->is_triged())
    {
        write_to_can_bus(C_DSC_SENSOR_BOTTOM_READ_STATUS,o_bottom->get_sensor_stat());
        if(o_bottom->get_sensor_stat())
        {
            o_encoder->set_zero();
            o_stepmotor->set_current_location(0ULL);
            o_stepmotor->set_togo_Location(0ULL);
        }
    }
    if(o_top->is_triged())
    {
        write_to_can_bus(C_DSC_SENSOR_TOP_READ_STATUS,o_top->get_sensor_stat());
        if(o_top->get_sensor_stat())
            write_to_can_bus(C_DSC_ENCODER_TOP_LOCATION, o_encoder->get_location());
    }
}
void Client::check_message()
{
    int32_t i_para,i_val;
    if(o_can->read_new_msg(&i_para,&i_val) == false)
        return;
    switch (i_para)
    {
    case C_DSC_STEP_MOTOR_SET_DIV:
        o_stepmotor->set_div(i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_low_us:
        o_stepmotor->set_low_us(i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_max_us:
        o_stepmotor->set_MAX_us(i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_KP:
        o_stepmotor->set_PID(i_val,0,0);
        break;
    case C_DSC_STEP_MOTOR_SET_KI:
        o_stepmotor->set_PID(0,i_val,0);
        break;
    case C_DSC_STEP_MOTOR_SET_KD:
        o_stepmotor->set_PID(0,0,i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_TOGO_Location:
        o_stepmotor->set_togo_Location(i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_Default_Direction:
        o_stepmotor->set_default_direction(!i_val);
        break;
    case C_DSC_STEP_MOTOR_ENABLE :
        o_stepmotor->enable();
        break;
    case C_DSC_STEP_MOTOR_DISABLE:
        o_stepmotor->disable();
        break;
    case C_DSC_STEP_MOTOR_PID_ENABLE :
        o_stepmotor->pid_enable();
        break;
    case C_DSC_STEP_MOTOR_PID_DISABLE:
        o_stepmotor->pid_disable();
        break;
    case C_DSC_STEP_MOTOR_STOP:
        o_stepmotor->stop();
        break;
    case C_DSC_STEP_MOTOR_START_MOVING:
        o_stepmotor->start_moving();
        break;
    case C_DSC_STEP_MOTOR_TOGO_ON_COMMAND:
        o_stepmotor->set_move_command_togo(i_val);
        break;
    case C_DSC_SENSOR_TOP_ENABLE:
        o_top->enable();
        break;
    case C_DSC_SENSOR_TOP_DISABLE:
        o_top->disable();
        break;
    case C_DSC_SENSOR_BOTTOM_ENABLE:
        o_bottom->enable();
        break;
    case C_DSC_SENSOR_BOTTOM_DISABLE:
        o_bottom->disable();
        break;
    case C_DSC_SENSOR_TOP_READ_STATUS:
        write_to_can_bus(C_DSC_SENSOR_TOP_READ_STATUS,o_top->get_sensor_stat());
        break;
    case C_DSC_SENSOR_BOTTOM_READ_STATUS:
        write_to_can_bus(C_DSC_SENSOR_BOTTOM_READ_STATUS,o_bottom->get_sensor_stat());
        break;
    
    case C_DSC_ENCODER_HARDWARE_ENABLE :
        o_encoder->hardware_enable();
        break;
    case C_DSC_ENCODER_HARDWARE_DISABLE:
        o_encoder->hardware_disable();
        break;
    case C_DSC_ENCODER_RESOLATION:
        o_encoder->set_nm_pp(i_val);
        break;
    case C_DSC_ENCODER_DIRECTION:
        o_encoder->direction(i_val);
        break;
    case C_DSC_ENCODER_LOCATION:
        write_to_can_bus(C_DSC_ENCODER_LOCATION,o_encoder->get_location());
        break;
    case C_DSC_ENCODER_TOP_LOCATION:
        //we are not store the top location so no action need here
        break;
    default:
        break;
    }

}
void Client::software_fake_message(int32_t i_para,int32_t i_val)
{
    o_can->software_message(i_para,i_val);
}