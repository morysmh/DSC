#include "define.h"
#include "motor.h"
#include "sensor.h"
#include "encoder.h"
#include "can.h"



class Client
{
public:
    Client( 
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
            uint8_t can_address
            );
    ~Client();
    void run();
    void write_to_can(int32_t i_para,int32_t i_val);
    void write_to_can_bus(Enum_DSC i_para,int32_t i_val);
    void check_message();
    void increse_sofware_encoder(int8_t i_val);
    void software_fake_message(int32_t i_para,int32_t i_val);
private:
    Encoder *o_encoder;
    Sensor *o_top,*o_bottom;
    StepMotor *o_stepmotor;
    CanCotroll *o_can;
};
