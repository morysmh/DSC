
#include "define.h"
#include "mcp2515.h"
#include "quadrature.pio.h"
#include "i2c-display-lib.h"

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

void Board_pin_Config();
void chagneLED(LED_Interval *led);
int8_t Keys_Read_adderss();

void Usart_init_main();
void Usart_init_wifiTest();
void Enable_A4498();
void init_board_config();
int main()
{
    uint8_t b_sendtoPC[20]={};
    LED_Interval Pico_LED;
    volatile uint64_t microdelay_500ms = 0, micro_picoled = 0, microdelay_in_state = 0;
    volatile int64_t t_nexpos = 0;
    volatile int64_t t_encoder = 0;
    volatile uint64_t t_send_can = 0;
    volatile int64_t stp = 1;
    volatile int8_t add = 0;
    volatile int32_t r_encoder = 0;

    volatile int32_t r_tmp_input = 0;
    volatile int8_t r_inputdata[32] = {};
    volatile uint8_t r_input_indx = 0;
    volatile int64_t t_inputdata = 0;
    volatile const int64_t c_delay_Between_Command = 3000;

    volatile uint64_t t_run_obj = 1000000;
    volatile uint8_t inx_mot = 0;
    char a_chrbuff[32] = {};

    CallBack_Parameter sendpara = {},para =  {},serverpara = {};

    Pico_LED.OFF = 340000;
    Pico_LED._ON = 200000;
    Pico_LED.lastCheck = 0;
    Pico_LED.stat = 1;
    Board_pin_Config();
    stdio_init_all();
    Enable_A4498();
    init_board_config();


    add = Keys_Read_adderss();
    mcp2515_init(20);
    tCAN msg_tst = {},msg_res = {};
    while (1)
    {
        if(mcp2515_check_message())
        {
            mcp2515_get_message(&msg_res);
            if(msg_res.id == C_Server_ADDRESS_CAN)
            {
                //TODO Handle Recive Message
            }
        }
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
        //PIN_TMC_STEP,
        //PIN_TMC__DIR,
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