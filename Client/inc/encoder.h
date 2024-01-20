#include "define.h"
#include "quadrature.pio.h"


class Encoder 
{
public:
    Encoder(uint8_t i_pin,int64_t i_nm_per_pulse = 5000);
    ~Encoder(){};
    void set_nm_pp(int64_t i_nm_per_pulse);
    void set_zero_sofware(){p_zero_val = p_pulse};
    int64_t get_location() const;

private:
    static void pio_irq_handler();
    void Encoder_init();
    int8_t p_pinA = -1;
    static bool p_if_object_creat = false;
    PIO __encoder_pio = 0;
    uint __encoder_sm =0;
    int64_t p_pulse = 0;
    int64_t p_zero_val = 0;
    int64_t p_nm_pp = 0;
    bool p_direction = false;
};

Encoder::Encoder(uint8_t i_A,int64_t i_nm_per_pulse = 5000)
{
    if(i_A > 29)
        return;
    if(p_if_object_creat)
        return;
    p_if_object_creat = true;
    Encoder_init();
    p_nm_pp = i_nm_per_pulse;
}

int64_t Encoder::get_location() const
{
    if(p_direction)
        return(((p_pulse - p_zero_val) * p_nm_pp) * (-1));
    return((p_pulse - p_zero_val) * p_nm_pp);
}
void Encoder::set_nm_pp(int64_t i_nm_per_pulse)
{
    if(i_nm_per_pulse <= 10)
        i_nm_per_pulse = 10;
    p_nm_pp = i_nm_per_pulse ;
}
void Encoder::Encoder_init()
{
    uint __encoder_offset =0;
    __encoder_pio = pio0;
    //__encoder_offset = pio_add_program(__encoder_pio, &quadrature_encoder_program);
    __encoder_sm = 0;
    pio_gpio_init(__encoder_pio, p_pinA);
    gpio_set_pulls(p_pinA, false, false);
    pio_gpio_init(__encoder_pio, p_pinA+1);
    gpio_set_pulls(p_pinA+1, false, false);
    // load the pio program into the pio memory
    uint offset = pio_add_program(__encoder_pio, &pio_rotary_encoder_program);
    // make a sm config
    pio_sm_config c = pio_rotary_encoder_program_get_default_config(offset);
    // set the 'in' pins
    sm_config_set_in_pins(&c, p_pinA);
    // set shift to left: bits shifted by 'in' enter at the least
    // significant bit (LSB), no autopush
    sm_config_set_in_shift(&c, false, false, 0);
    // set the IRQ handler
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    // enable the IRQ
    irq_set_enabled(PIO0_IRQ_0, true);
    pio0_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_BITS;
    // init the sm.
    // Note: the program starts after the jump table -> initial_pc = 16
    pio_sm_init(__encoder_pio, __encoder_sm, 16, &c);
    // enable the sm
    pio_sm_set_enabled(__encoder_pio, __encoder_sm, true);
    //quadrature_encoder_program_init(__encoder_pio, __encoder_sm, __encoder_offset, PIN__A___Pin, 100);
}
static void Encoder::pio_irq_handler()
{
    if(pio0_hw->irq & 1)    
    {
        p_pulse--;
    }
    if(pio0_hw->irq & 2)
    {
        p_pulse++;
    }
    pio0_hw->irq = 3;
}
