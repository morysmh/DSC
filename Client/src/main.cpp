
#include "define.h"
#include "mcp2515.h"
#include "quadrature.pio.h"
#include "pio_rotary_encoder.pio.h"
#include "i2c-display-lib.h"
#include "client.h"

#ifdef __cplusplus
extern "C"
{
#endif
    void wifi_uart_rx()
    {
        if (uart_is_readable(uart1))
        {
            (void)uart_getc(uart1);
        }
    }
    void usbUartRxHandler()
    {
        if (uart_is_readable(uart0))
        {
            (void)uart_getc(uart1);
        }
    }
#ifdef __cplusplus
}
#endif
//******************************************
void pio_irq_handler();
void Encode_pio_init(uint p_pinA);
PIO __encoder_pio;
uint __encoder_sm;
Client *ptr_client;
//******************************************

void Board_pin_Config();
void chagneLED(LED_Interval *led);
int8_t Keys_Read_adderss();

void Usart_init_main();
void Usart_init_wifiTest();
void Enable_A4498();
void init_board_config();
void for_test_only();
int main()
{
    LED_Interval Pico_LED;
    volatile int8_t add = 0;

    Pico_LED.OFF = 340000;
    Pico_LED._ON = 200000;
    Pico_LED.lastCheck = 0;
    Pico_LED.stat = 1;
    Board_pin_Config();
    stdio_init_all();
    Enable_A4498();
    init_board_config();
    add = Keys_Read_adderss();
    mcp2515_init(14);
    volatile uint64_t t_first = time_us_64() + 25000ULL;
    while(t_first > time_us_64());
    Client thisclient(PIN_TMC_STEP,
                    PIN_TMC__DIR,
                    PIN_SW___Pin,
                    PIN__Z___Pin,
                    PIN__A___Pin,
                    PIN_SPI_MOSI,
                    PIN_SPI__SCK,
                    PIN_SPI__Csn,
                    PIN_TMC___EN,
                    PIN_SPI_MISO,
                    add);
    ptr_client = &thisclient;
    Encode_pio_init(PIN__A___Pin);
    while (1)
    {
        //for_test_only();
        thisclient.run();
        chagneLED(&Pico_LED);
        PICO_LED_Stat(Pico_LED.stat);
    }
}
void Usart_init_wifiTest()
{
    // Set up our UART with a basic baud rate.
    uart_init(uart1, 2400);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(5, GPIO_FUNC_UART);
    gpio_set_function(4, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(uart1, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(uart1, false, false);

    // Set our data format
    uart_set_format(uart1, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(uart1, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, wifi_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(uart1, true, false);

    // OK, all set up.
    // Lets send a basic string out, and then run a loop and wait for RX interrupts
    // The handler will count them, but also reflect the incoming data back with a slight change!
}
void Usart_init_main()
{
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, 2400);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART0_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, usbUartRxHandler);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, false, false);

    // OK, all set up.
    // Lets send a basic string out, and then run a loop and wait for RX interrupts
    // The handler will count them, but also reflect the incoming data back with a slight change!
}
void Board_pin_Config()
{
    adc_init();
    int8_t size_arr;
    uint8_t arr_out_pin[] = {
        PIN_SPI_MISO,
        PIN_CAN___CS,
        PIN_TMC_DIG0,
        PIN_TMC__Ain,
        PIN_TMC___EN,
        PIN_SPI_MOSI,
        PIN_SPI__SCK,
        PIN_SPI__Csn,
        PIN_TMC__CLK,
        PIN_TMC_STEP,
        PIN_TMC__DIR,
        PIN_LED__RED,
        PIN_LED_Gren
        };
    uint8_t arr_in_pin[] = {
        PIN_ADD_Pin1,
        PIN_ADD_Pin2,
        // PIN_SPI_MISO,
        PIN__A___Pin,
        PIN__B___Pin,
        PIN__Z___Pin,
        PIN_SW___Pin,
        PIN_CAN__INT,
        PIN_ADD_Pin3};

    size_arr = sizeof(arr_out_pin) / sizeof(uint8_t) - 1;
    for (; size_arr >= 0; size_arr--)
    {
        gpio_init(arr_out_pin[size_arr]);
        gpio_set_dir(arr_out_pin[size_arr], GPIO_OUT);
    }

    size_arr = sizeof(arr_in_pin) / sizeof(uint8_t) - 1;
    for (; size_arr >= 0; size_arr--)
    {
        gpio_init(arr_in_pin[size_arr]);
        gpio_set_dir(arr_in_pin[size_arr], GPIO_IN);
    }
    gpio_pull_up(PIN_ADD_Pin1);
    gpio_pull_up(PIN_ADD_Pin2);
    gpio_pull_up(PIN_ADD_Pin3);
    gpio_init(Led_Pin_PICO);
    gpio_set_dir(Led_Pin_PICO, GPIO_OUT);
}


void chagneLED(LED_Interval *led)
{
    if (led->lastCheck > time_us_64())
    {
        return;
    }
    if (led->stat)
    {
        led->lastCheck = time_us_64() + led->OFF;
        led->stat = 0;
    }
    else
    {
        led->lastCheck = time_us_64() + led->_ON;
        led->stat = 1;
    }
}

void Enable_A4498()
{
    gpio_put(PIN_TMC___EN, 0);
    gpio_put(PIN_SPI_MOSI, 1);
    gpio_put(PIN_SPI__SCK, 1);
    gpio_put(PIN_SPI__Csn, 1);
    gpio_put(PIN_SPI_MISO, 1);
    gpio_put(PIN_TMC__CLK, 1);
}
void init_board_config()
{
    CallBack_Parameter para;
}

int8_t Keys_Read_adderss()
{
    static volatile int8_t r_keys = 0;
    if (r_keys)
        return r_keys;
    r_keys |= ((gpio_get(PIN_ADD_Pin3)) << 0);
    r_keys |= ((gpio_get(PIN_ADD_Pin2)) << 1);
    r_keys |= ((gpio_get(PIN_ADD_Pin1)) << 2);
    r_keys = 0x7 - r_keys;
    return r_keys;
}

void Encode_pio_init(uint p_pinA)
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
void pio_irq_handler()
{
    if(pio0_hw->irq & 1)    
    {
        ptr_client->increse_sofware_encoder(-1);
    }
    if(pio0_hw->irq & 2)
    {
        ptr_client->increse_sofware_encoder(1);
    }
    pio0_hw->irq = 3;
}
void for_test_only()
{
    static volatile bool is_run = false;
    static volatile uint64_t t_mili = 500000ULL;
    static volatile int8_t r_i = 0;
    uint8_t buf[15] = {};
    uint16_t rtmp = 0;
    if(is_run == true)
        return;
    if(t_mili > time_us_64())
        return;
    switch (r_i)
    {
    case 0:
        rtmp = 0xFFFF;
        rtmp &= ~(1<<C_DSC_BIT_CONFIG_ENCODER_DEFAULT_DIRECTION);
        rtmp &= ~(1<<C_DSC_BIT_CONFIG_MOTOR_DEFAULT_DIRECTION);
        buf[C_DSC_ARRAY_CONFIG_REG_CONF_1byte] = rtmp & 0xFF;
        buf[C_DSC_ARRAY_CONFIG_REG_CONF_2byte] = (rtmp>>8);
        buf[C_DSC_ARRAY_CONFIG_REG_POSITION_INTERVAl] = 10;
        buf[C_DSC_ARRAY_CONFIG_REG_FROM_OTHER_MOTOR_STATUS_READ] = 2;
        buf[C_DSC_ARRAY_CONFIG_REG_DIV] = 16;
        buf[C_DSC_ARRAY_CONFIG_REG_ENCODER_RES_1byte] = 5000 & 0xFF;
        buf[C_DSC_ARRAY_CONFIG_REG_ENCODER_RES_2byte] = (5000>>8);
        ptr_client->software_fake_message(C_DSC_OPCODE_REG_CONFIG,buf);
        buf[C_DSC_ARRAY_PID_REG_KP] = 8;
        buf[C_DSC_ARRAY_PID_REG_KI] = 2;
        buf[C_DSC_ARRAY_PID_REG_KD] = 0;
        ptr_client->software_fake_message(C_DSC_OPCODE_REG_PID_CONFIG,buf);
        buf[C_DSC_ARRAY_SPEED_REG_HIGH_us_2byte] = (1500>>8);
        buf[C_DSC_ARRAY_SPEED_REG_HIGH_us_1byte] = (1500) & 0xFF;
        buf[C_DSC_ARRAY_SPEED_REG_LOW_us_2byte] = (50>>8);
        buf[C_DSC_ARRAY_SPEED_REG_LOW_us_1byte] = (50) & 0xFF;
        ptr_client->software_fake_message(C_DSC_OPCODE_REG_SPEED,buf);
        buf[C_DSC_ARRAY_TOGO_REG_index] = 240;
        buf[C_DSC_ARRAY_TOGO_REG_index + 1] = 126;
        buf[C_DSC_ARRAY_TOGO_REG_index + 2] = 14;
        buf[C_DSC_ARRAY_TOGO_REG_index + 3] = 0;
        ptr_client->software_fake_message(C_DSC_OPCODE_REG_TOGO,buf);

        t_mili = time_us_64() + 500000ULL;
        r_i++;
        break;
    case 1:
        buf[C_DSC_ARRAY_StartStop] = 0;
        buf[C_DSC_ARRAY_StartStop] |= (1<<C_DSC_BIT_START_MOVING);
        ptr_client->software_fake_message(C_DSC_OPCODE_REG_START_STOP,buf);
        t_mili = time_us_64() + 12000000ULL;
        r_i++;
        break;
    case 2:
        r_i++;
        buf[C_DSC_ARRAY_TOGO_REG_index] = 240;
        buf[C_DSC_ARRAY_TOGO_REG_index + 1] = 126;
        buf[C_DSC_ARRAY_TOGO_REG_index + 2] = 14;
        buf[C_DSC_ARRAY_TOGO_REG_index + 3] = 255;
        ptr_client->software_fake_message(C_DSC_OPCODE_REG_TOGO,buf);
        buf[C_DSC_ARRAY_StartStop] = 0;
        buf[C_DSC_ARRAY_StartStop] |= (1<<C_DSC_BIT_START_MOVING);
        ptr_client->software_fake_message(C_DSC_OPCODE_REG_START_STOP,buf);
        t_mili = time_us_64() + 12000000ULL;
        break;
    case 3:
        r_i++;
        buf[C_DSC_ARRAY_StartStop] = 0;
        buf[C_DSC_ARRAY_StartStop] |= (1<<C_DSC_BIT_STOP_MOVING);
        ptr_client->software_fake_message(C_DSC_OPCODE_REG_START_STOP,buf);
        t_mili = time_us_64() + 9000ULL;
        //ptr_client->software_fake_message(C_DSC_STEP_MOTOR_SET_TOGO_Location,-23000000LL);
        //ptr_client->software_fake_message(C_DSC_STEP_MOTOR_SET_TOGO_Location,5000ULL);
        break;
    
    default:
        r_i = 0;
        is_run = true;
        break;
    }
}