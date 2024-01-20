#include "define.h"
#include "mcp2515.h"
#include "motor.h"
#include "sensor.h"
#include "encoder.h"

class CanCotroll
{
public:
    CanCotroll(int32_t i_add,Sensor *i_bottom,Sensor *i_top,Encoder *i_encoder,StepMotor *i_step);
    ~CanCotroll(){}
    void send_data(Enum_DSC i_para,int32_t i_val);
    void run();
private:
    void write_buffer(int32_t i_para,int32_t i_val);
    void handle_message();
    int32_t p_address = 100;
    uint8_t p_head = 0,p_tail = 0;
    const uint8_t c_buff_size = 35;
    uint32_t p_data[40][2] = {};
    Sensor *p_sensor_bot,*p_sensor_top;
    Encoder *p_encoder;
    StepMotor *p_step;
    tCAN p_can;
};
void CanCotroll::write_buffer(int32_t i_para,int32_t i_val)
{
    p_data[p_head][0] = i_para;
    p_data[p_head][1] = i_val;
    p_head++;
    if(p_head > c_buff_size)
        p_head = c_buff_size;
}
void CanCotroll::handle_message()
{
    int32_t i_val = 0,i_para = 0;
    i_val |= (((uint32_t)p_can.data[0])<<0ULL);
    i_val |= (((uint32_t)p_can.data[1])<<8ULL);
    i_val |= (((uint32_t)p_can.data[2])<<16ULL);
    i_val |= (((uint32_t)p_can.data[3])<<24ULL);
    i_para |= (uint32_t)p_can.data[4];
    switch (i_para)
    {
    case C_DSC_STEP_MOTOR_SET_Default_Direction:
        p_step->set_default_direction(!i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_DIV:
        p_step->set_div(i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_KP:
        p_step->set_PID(i_val,0,0);
        break;
    case C_DSC_STEP_MOTOR_SET_KI:
        p_step->set_PID(0,i_val,0);
        break;
    case C_DSC_STEP_MOTOR_SET_KD:
        p_step->set_PID(0,0,i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_low_us:
        p_step.set_low_us(i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_max_us:
        p_step->set_MAX_us(i_val);
        break;
    case C_DSC_STEP_MOTOR_SET_TOGO_Location:
        p_step->set_togo_Location(i_val);
        break;
    
    case C_DSC_SENSOR_TOP_ENABLE:
        p_sensor_top->enable();
        break;
    case C_DSC_SENSOR_BOTTOM_ENABLE:
        p_sensor_bot->enable();
        break;
    case C_DSC_SENSOR_TOP_READ_STATUS:
        write_buffer(C_DSC_SENSOR_TOP_READ_STATUS,p_sensor_top->get_sensor_stat());
        break;
    case C_DSC_SENSOR_BOTTOM_READ_STATUS:
        write_buffer(C_DSC_SENSOR_BOTTOM_READ_STATUS,p_sensor_bot->get_sensor_stat());
        break;
    default:
        break;
    }

}
void CanCotroll::run()
{
    if(mcp2515_check_message())
    {
        mcp2515_get_message(&p_can);
        if(p_can.id == p_address)
            handle_message();
    }
    if(p_sensor_bot->is_triged())
    {
        write_buffer(C_DSC_SENSOR_BOTTOM_READ_STATUS,p_sensor_bot->get_sensor_stat())
        if(p_sensor_bot->get_sensor_stat())
        {
            p_encoder->set_zero_sofware();
            p_step->set_current_location(0ULL);
            p_step->set_togo_Location(0ULL);
        }
    }
    if(p_sensor_top->is_triged())
    {
        write_buffer(C_DSC_SENSOR_TOP_READ_STATUS,p_sensor_bot->get_sensor_stat())
    }
}
void CanCotroll::send_data(Enum_DSC i_para,int32_t i_val)
{
    p_step->run();
}
CanCotroll::CanCotroll(Sensor *i_bottom,Sensor *i_top,Encoder *i_encoder,StepMotor *i_step)
{
    p_sensor_bot = i_bottom;
    p_sensor_top = i_top;
    p_encoder = i_encoder;
    p_step = i_step;
}