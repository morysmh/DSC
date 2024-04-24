#include "client.h"

void Client::send_LocationData(bool iSNow)
{
    if(pReportingStaus == false)
        return;
    o_can->send_status(o_encoder->get_location(),
                        o_stepmotor->isMoving(),
                        o_bottom->get_sensor_stat(),
                        o_top->get_sensor_stat(),
                        iSNow,
                        bFailure);
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

void Client::CheckFailure()
{
    if(tFailCheck > time_us_64())
        return;
    tFailCheck = time_us_64() + 8000;
    if(bFailure)
    {
        return;
    }
    if(!o_stepmotor->isMoving())
        return;
    if(o_encoder->get_location() != rEncoderVal)
    {
        rFailureHyst = 0;
        rEncoderVal = o_encoder->get_location();
        return;
    }
    rFailureHyst++;
    if(rFailureHyst < cFailureHystCount)
        return;
    bFailure = true;
    send_LocationData(true);
    
}

void Client::run()
{
    check_message();
    send_LocationData(false);
    CheckFailure();
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
        send_LocationData(true);
        if(o_bottom->get_sensor_stat())
        {
            o_encoder->set_zero();
            o_stepmotor->set_current_location(0ULL);
            o_stepmotor->set_togo_Location(0ULL);
        }
    }
    if(o_top->is_triged())
    {
        send_LocationData(true);
    }
}
void Client::check_message()
{
    uint8_t iPara,motNO,oStat;
    uint8_t rbuff[10];
    if(o_can->read_other_motor_stat(&oStat))
    {
        status_register_handle(oStat);
    }
    if(o_can->read_new_message(rbuff,&motNO,&iPara) == false)
        return; 
    handle_message(rbuff,motNO,iPara);
}
void Client::handle_message(uint8_t *rbuff,uint8_t motNO,uint8_t iPara)
{
    switch (iPara)
    {
    case C_DSC_OPCODE_REG_CONFIG:
        config_register_handle(rbuff);
        bFailure = false;
        break;
    case C_DSC_OPCODE_REG_SPEED:
        speed_register_handle(rbuff);
        break;
    case C_DSC_OPCODE_REG_TOGO:
        togo_register_handle(rbuff);
        break;
    case C_DSC_OPCODE_REG_STATUS:
        status_register_handle(rbuff[C_DSC_ARRAY_STATUS_REPORT_1byte + 1]);
        break;
    case C_DSC_OPCODE_REG_START_STOP:
        Star_Stop_register_handle(rbuff);
        send_LocationData(true);
        break;
    case C_DSC_OPCODE_REG_PID_CONFIG:
        PID_register_handle(rbuff);
        break;
    case C_DSC_OPCODE_REG_Releative_TOGO:
        Releative_togo_register_handle(rbuff);
        break;
        
        default:
            break;
    }

}
void Client::speed_register_handle(uint8_t *data)
{
    uint16_t rLow = 0;
    uint16_t rHigh = 0,tmp = 0;

    rLow = data[C_DSC_ARRAY_SPEED_REG_LOW_us_2byte];
    rLow = (rLow<<8L);
    rLow |= data[C_DSC_ARRAY_SPEED_REG_LOW_us_1byte];

    rHigh = data[C_DSC_ARRAY_SPEED_REG_HIGH_us_2byte];
    rHigh = (rHigh<<8L);
    rHigh |= data[C_DSC_ARRAY_SPEED_REG_HIGH_us_1byte];

    if(rLow > rHigh)
    {
        tmp = rLow;
        rLow = rHigh;
        rHigh = tmp;
    }

    o_stepmotor->set_low_us(rLow);
    o_stepmotor->set_MAX_us(rHigh);
}
void Client::togo_register_handle(uint8_t *data)
{
    int32_t oToGo = 0;
    o_can->ptr8_to_int32(&data[C_DSC_ARRAY_TOGO_REG_index],&oToGo);
    o_stepmotor->set_move_command_togo(oToGo);
}
void Client::Releative_togo_register_handle(uint8_t *data)
{
    int32_t oToGo = 0;
    o_can->ptr8_to_int32(&data[C_DSC_ARRAY_Releative_TOGO_REG_index],&oToGo);
    o_stepmotor->set_move_command_togo_releative(oToGo);
}
void Client::status_register_handle(uint8_t iStat)
{
    if(pPrevReportStat == iStat)
        return;
    pPrevReportStat = iStat;
    if(_bv(iStat,C_DSC_BIT_STATUS_SENSOR_TOP_STATUS))
        o_stepmotor->stop();
}
void Client::Star_Stop_register_handle(uint8_t *data)
{
    uint8_t StartStop = 0;
    StartStop = data[C_DSC_ARRAY_StartStop];


    if(_bv(StartStop,C_DSC_BIT_START_MOVING))
        o_stepmotor->start_moving();
    if(_bv(StartStop,C_DSC_BIT_STOP_MOVING))
        o_stepmotor->stop();
    pReportingStaus = !(_bv(StartStop,C_DSC_BIT_STOP_REPORTING));
}
void Client::PID_register_handle(uint8_t *data)
{
    o_stepmotor->set_PID(data[C_DSC_ARRAY_PID_REG_KP],data[C_DSC_ARRAY_PID_REG_KI],data[C_DSC_ARRAY_PID_REG_KD]);
}
void Client::config_register_handle(uint8_t *data)
{
    uint16_t rConf = 0;
    uint16_t rencoderRes = 0;
    o_can->set_interval(data[C_DSC_ARRAY_CONFIG_REG_POSITION_INTERVAl]);
    o_can->set_other_sensor_code(data[C_DSC_ARRAY_CONFIG_REG_FROM_OTHER_MOTOR_STATUS_READ]);
    o_stepmotor->set_div(data[C_DSC_ARRAY_CONFIG_REG_DIV]);
    rencoderRes = data[C_DSC_ARRAY_CONFIG_REG_ENCODER_RES_2byte];
    rencoderRes = (rencoderRes<<8L);
    rencoderRes |= data[C_DSC_ARRAY_CONFIG_REG_ENCODER_RES_1byte];
    o_encoder->set_nm_pp(rencoderRes);
    o_stepmotor->set_EncoderRes(rencoderRes);

    rConf = data[C_DSC_ARRAY_CONFIG_REG_CONF_2byte];
    rConf = (rConf<<8L);
    rConf |= data[C_DSC_ARRAY_CONFIG_REG_CONF_1byte];
    if(_bv(rConf,C_DSC_BIT_CONFIG_ENCODER_HARDWARE) == true)
        o_encoder->hardware_enable();
    else
        o_encoder->hardware_disable();
    o_bottom->set_normal_stat(_bv(rConf,C_DSC_BIT_CONFIG_BOTTOM_SENSOR_DEFAULT));
    o_top->set_normal_stat(_bv(rConf,C_DSC_BIT_CONFIG_TOP_SENSOR_DEFAULT));
    if(_bv(rConf,C_DSC_BIT_CONFIG_SENSOR_ENABLE_BOTTOM))
        o_bottom->enable();
    else
        o_bottom->disable();
    if(_bv(rConf,C_DSC_BIT_CONFIG_SENSOR_ENABLE_TOP))
        o_top->enable();
    else
        o_top->disable();

    o_stepmotor->set_default_direction(_bv(rConf,C_DSC_BIT_CONFIG_MOTOR_DEFAULT_DIRECTION));
    o_encoder->direction(_bv(rConf,C_DSC_BIT_CONFIG_ENCODER_DEFAULT_DIRECTION));

    if(_bv(rConf,C_DSC_BIT_CONFIG_MOTOR_ENABLE))
        o_stepmotor->enable();
    else
        o_stepmotor->disable();
    if(_bv(rConf,C_DSC_BIT_CONFIG_PID_ENABLE))
        o_stepmotor->pid_enable();
    else
        o_stepmotor->pid_disable();
    o_stepmotor->lock_motor(_bv(rConf,C_DSC_BIT_CONFIG_LOCK_MOTOR));
}

bool Client::_bv(uint32_t iVal,uint8_t iBV)
{
    if(iVal & (1ULL<<iBV))
        return true;
    return false;
}
void Client::software_fake_message(int32_t iPara,uint8_t *rbuff)
{
    uint8_t motNO = 0;
    handle_message(rbuff,motNO,iPara);
}