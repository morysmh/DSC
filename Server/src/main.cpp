
#include "define.h"
#include "mcp2515.h"
#include "quadrature.pio.h"
#include "pio_rotary_encoder.pio.h"
#include "i2c-display-lib.h"
#include "canserver.h"
#include "clientmotor.h"
#include "sensor.h"

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
//******************************************

void Board_pin_Config();
void chagneLED(LED_Interval *led);
int8_t Keys_Read_adderss();

void Usart_init_main();
void Usart_init_wifiTest();
void Enable_A4498();
void init_board_config();
void handle_pc_communication(int32_t i_data,CanCotroll *dev);
void for_test_only();
void lcd_refresh_data(int32_t mot1,int32_t mot2,int32_t mot3,int32_t mot4);
int main()
{
    int64_t t_mili_reset_interval = 0;
    const int64_t c_reset_interval = 2500000LL;
    LED_Interval Pico_LED;
    volatile int8_t add = 0;

    volatile int32_t r_tmp_input = 0;

    char r_lcdbuff[80] = {};
    bool pc_active = false;
    volatile uint64_t t_first = time_us_64() + 25000ULL;
    int32_t i_val = 0,i_para;
    int8_t motNO = 0;


    Pico_LED.OFF = 340000;
    Pico_LED._ON = 200000;
    Pico_LED.lastCheck = 0;
    Pico_LED.stat = 1;
    Board_pin_Config();
    stdio_init_all();
    Enable_A4498();
    init_board_config();
    while(t_first > time_us_64());
    add = Keys_Read_adderss();
    mcp2515_init(14);
    lcd_init(14,15);
    lcd_setCursor(0,0);
    lcd_print("raw data");

    //for_test_only();
    CanCotroll dev_can(1);
    virtualMotor mot1(1,&dev_can);
    virtualMotor mot2(2,&dev_can);
    virtualMotor mot3(3,&dev_can);
    virtualMotor mot4(4,&dev_can);
    virtualMotor brodcast(C_DSC_BRODCAST_ADDRESS_CAN,&dev_can);
    Sensor s_start(PIN__Z___Pin);
    Sensor s_reset(PIN_SW___Pin);
    s_reset.set_normal_stat(false);
    s_start.set_normal_stat(false);
    s_start.enable();
    s_reset.enable();

    mot1.setEnableEncoder(true);
    mot1.setSensorBottomNormalStat(true);
    mot1.setDir(true);
    mot1.setDirEncoder(false);
    mot1.setSpeedUS(200LL,1000LL);
    mot1.setDefaultLow(200LL);

    mot2.setEnableEncoder(true);
    mot2.setDir(true);
    mot2.setDirEncoder(false);
    mot2.setSpeedUS(245LL,1000LL);
    mot2.setDefaultLow(245LL);
    mot2.setOtherMotorSensorStop(2);

    mot3.setEnableEncoder(true);
    mot3.setSensorBottomNormalStat(false);
    mot3.setDir(true);
    mot3.setDirEncoder(false);
    mot3.setSpeedUS(245LL,1000LL);
    mot3.setDefaultLow(245LL);
    mot3.setOtherMotorSensorStop(2);

    mot4.setEncoder_nm(1500LL);
    mot4.setSensorBottomNormalStat(false);
    mot4.setDir(true);
    mot4.setDirEncoder(false);
    mot4.setSpeedUS(245LL,1000LL);
    mot4.setDefaultLow(245LL);

    t_first = time_us_64() + 400000LL;
    while(t_first > time_us_64());
    while (1)
    {
        lcd_refresh_data(mot1.getLocation(),mot2.getLocation(),mot3.getLocation(),mot4.getLocation());
        s_reset.run();
        s_start.run();
        dev_can.run();
        mot1.run();
        mot2.run();
        mot3.run();
        mot4.run();
        chagneLED(&Pico_LED);
        PICO_LED_Stat(Pico_LED.stat);
        r_tmp_input = getchar_timeout_us(1);
        if((r_tmp_input <= 255) && (r_tmp_input >= 0))
        {
            pc_active = true;
            handle_pc_communication(r_tmp_input,&dev_can);
        }
        if(dev_can.read_new_msg(&i_para,&i_val,&motNO))
        {
            mot1.readCAN(motNO,i_para,i_val);
            mot2.readCAN(motNO,i_para,i_val);
            mot3.readCAN(motNO,i_para,i_val);
            mot4.readCAN(motNO,i_para,i_val);
        }
        if(pc_active)
            continue;
        if(s_reset.is_triged())
        {
            if(s_reset.get_sensor_stat() == 1)
            {
                if(t_mili_reset_interval > time_us_64() )
                    continue;
                t_mili_reset_interval = time_us_64() + c_reset_interval;
                mot1.GoHome();
                mot2.GoHome();
                mot3.GoHome();
                mot4.GoHome();
            }
        }
        if(s_start.is_triged())
        {
            if(s_start.get_sensor_stat() == 1)
            {
                mot1.setToGo(950000LL);
                mot2.setToGo(950000LL);
                mot3.setToGo(950000LL);
                mot4.setToGo(950000LL);
                //brodcast.start_moving();
                //brodcast.stop();
            }
        }
        
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
       //mainEncoder.set_software_inc(-1); 
    }
    if(pio0_hw->irq & 2)
    {
       //mainEncoder.set_software_inc(1); 
    }
    pio0_hw->irq = 3;
}
void handle_pc_communication(int32_t i_data,CanCotroll *dev)
{
    static volatile uint8_t r_buff[40] = {};
    static volatile uint8_t r_indx = 0;
    static volatile uint64_t t_mili = 0;
    static volatile int32_t i_val = 0;
    const uint64_t c_delay_Between_Command = 10000ULL;
    if(t_mili < time_us_64())
        r_indx = 0;
    if(r_indx >= 16)
        r_indx = 0;
    t_mili = time_us_64() + c_delay_Between_Command;
    r_buff[r_indx] = (i_data & 0xFFLL);
    r_indx++;
    if(r_indx < 9)
        return;
    r_indx = 0;
    i_val = 0;
    i_val |= (((uint32_t)r_buff[2])<<0ULL);
    i_val |= (((uint32_t)r_buff[3])<<8ULL);
    i_val |= (((uint32_t)r_buff[4])<<16ULL);
    i_val |= (((uint32_t)r_buff[5])<<24ULL);
    dev->send_data(r_buff[1],i_val,r_buff[0]);
}
void for_test_only()
{
    int32_t i_val,i_para;
    int8_t motNO;
    int32_t cmNO = 0;
    int8_t lcdLine = 1;
    char r_lcdbuff[80] ={};
    CanCotroll dev_can(1);
    virtualMotor testMOT(4,&dev_can);
    Sensor s_start(PIN__Z___Pin);
    Sensor s_reset(PIN_SW___Pin);
    s_reset.set_normal_stat(false);
    s_start.set_normal_stat(false);
    s_start.enable();
    s_reset.enable();

    testMOT.setEncoder_nm(150LL);
    testMOT.setSensorBottomNormalStat(false);
    testMOT.setDir(true);
    testMOT.setDirEncoder(false);
    testMOT.setSpeedUS(245LL,1000LL);
    testMOT.setDefaultLow(245LL);


    testMOT.setSpeedUS(250LL,1000LL);
    testMOT.setDefaultLow(250LL);
    while (1)
    {
        s_reset.run();
        s_start.run();
        dev_can.run();
        testMOT.run();
        if(dev_can.read_new_msg(&i_para,&i_val,&motNO))
        {
            testMOT.readCAN(motNO,i_para,i_val);
            sprintf(r_lcdbuff,"1:%d        ",testMOT.getLocation());
            r_lcdbuff[19] = 0;
            lcd_setCursor(0,0);
            lcd_print(r_lcdbuff);
            cmNO++;
            lcdLine++;
            if(lcdLine > 3)
                lcdLine = 1;
            sprintf(r_lcdbuff,"c:%d f:%d p:%d :%d      ",cmNO,motNO,i_para,i_val);
            r_lcdbuff[19] = 0;
            lcd_setCursor(lcdLine,0);
            lcd_print(r_lcdbuff);
        }
        if(s_reset.is_triged())
        {
            if(s_reset.get_sensor_stat() == 1)
            {
                //TODO Here
                testMOT.GoHome();
            }
        }
        if(s_start.is_triged())
        {
            if(s_start.get_sensor_stat() == 1)
            {
                //TODO Here
                testMOT.setToGo(950000LL);
                //dev_can.send_data(C_DSC_STEP_MOTOR_SET_TOGO_Location,95000LL,2);
            }
        }
        
    }
}
void lcd_refresh_data(int32_t mot1,int32_t mot2,int32_t mot3,int32_t mot4)
{
    static char r_lcdbuff[80] = {};
    static volatile int64_t mili_refresh;

    if(mili_refresh > time_us_64())
        return;
    mili_refresh = time_us_64() + 980000LL;
    sprintf(r_lcdbuff,"1:%d 2:%d       ",mot1,mot2);
    r_lcdbuff[19] = 0;
    lcd_setCursor(0,0);
    lcd_print(r_lcdbuff);
    sprintf(r_lcdbuff,"3:%d 4:%d       ",mot3,mot4);
    r_lcdbuff[19] = 0;
    lcd_setCursor(1,0);
    lcd_print(r_lcdbuff);
}