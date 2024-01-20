#include "define.h"
#include "quadrature.pio.h"
#include "pio_rotary_encoder.pio.h"

class Encoder 
{
public:
    Encoder(uint8_t i_pin,int64_t i_nm_per_pulse = 5000);
    ~Encoder(){};
    void set_nm_pp(int64_t i_nm_per_pulse);
    void set_zero_sofware(){p_zero_val = p_pulse;};
    int64_t get_location() const;
    void set_software_inc(int8_t i_in){p_pulse += i_in;}

private:
    int8_t p_pinA = -1;
    int64_t p_pulse = 0;
    int64_t p_zero_val = 0;
    int64_t p_nm_pp = 0;
    bool p_direction = false;
};
