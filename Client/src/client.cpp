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
            uint8_t i_add
            )
{
    o_can = new CanCotroll(i_add);
    o_encoder = new Encoder(pin_Encoder_A,5000);
    o_stepmotor = new StepMotor(pin_clk,pin_dir,pin_DIV1,pin_DIV2,pin_DIV3);
    o_top = new Sensor(pin_TOP_Sensor,Sensor::END_Sensor);
    o_bottom = new Sensor(pin_Bottom_Sensor);
    o_bottom->set_normal_stat(true);
    o_top->set_normal_stat(true);
    o_bottom->enable();
    o_top->enable();
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
    o_bottom->run();
    if(o_bottom->is_triged())
    {
        write_to_can_bus(C_DSC_SENSOR_BOTTOM_READ_STATUS,o_bottom->get_sensor_stat());
        if(o_bottom->get_sensor_stat())
        {
            o_encoder->set_zero_sofware();
            o_stepmotor->set_current_location(0ULL);
            o_stepmotor->set_togo_Location(0ULL);
        }
    }
    o_top->run();
    if(o_top->is_triged())
    {
        write_to_can_bus(C_DSC_SENSOR_TOP_READ_STATUS,o_top->get_sensor_stat());
    }
}
void Client::check_message()
{
    int32_t i_para,i_val;
    if(o_can->read_new_msg(&i_para,&i_val) == false)
        return;
    switch (i_para)
    {
    case C_DSC_STEP_MOTOR_SET_Default_Direction:
        o_stepmotor->set_default_direction(!i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_DIV:
        o_stepmotor->set_div(i_val);
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
    case C_DSC_STEP_MOTOR_SET_low_us:
        o_stepmotor->set_low_us(i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_max_us:
        o_stepmotor->set_MAX_us(i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_TOGO_Location:
        o_stepmotor->set_togo_Location(i_val);
        break;
    
    case C_DSC_SENSOR_TOP_ENABLE:
        o_top->enable();
        break;
    case C_DSC_SENSOR_BOTTOM_ENABLE:
        o_bottom->enable();
        break;
    case C_DSC_SENSOR_TOP_READ_STATUS:
        write_to_can_bus(C_DSC_SENSOR_TOP_READ_STATUS,o_top->get_sensor_stat());
        break;
    case C_DSC_SENSOR_BOTTOM_READ_STATUS:
        write_to_can_bus(C_DSC_SENSOR_BOTTOM_READ_STATUS,o_bottom->get_sensor_stat());
        break;
    default:
        break;
    }

}