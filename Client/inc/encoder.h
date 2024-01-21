#include "define.h"
#include "quadrature.pio.h"
#include "pio_rotary_encoder.pio.h"

class Encoder 
{
public:
    Encoder(uint8_t i_pin,int32_t i_nm_per_pulse = 5000);
    ~Encoder(){};
    void set_nm_pp(int32_t i_nm_per_pulse);
    void set_zero_sofware(){p_zero_val = p_pulse;};
    int32_t get_location() const;
    void set_software_inc(int8_t i_in);
    void set_hardware_inc(int8_t i_in);
    bool is_change();
    void hardware_enable(){p_software_encoder = false;}
    void hardware_disable(){p_software_encoder = true;}

private:
    int8_t p_pinA = -1;
    int32_t p_pulse = 0;
    int32_t p_zero_val = 0;
    int32_t p_nm_pp = 0;
    bool p_is_change = false;
    bool p_direction = false;
    bool p_software_encoder = true;
};
