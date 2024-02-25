
#include "define.h"
#include "mcp2515.h"
#include "quadrature.pio.h"
#include "pio_rotary_encoder.pio.h"
#include "i2c-display-lib.h"
#include "canserver.h"
#include "clientmotor.h"
#include "sensor.h"

#define Absolute 1
#define Reletive 2
#define Speed 3
#define DefSpeed 1
#define NoChange 0
#define END_OF_Command 4
typedef enum{
    do_nothing = 0,
    home_motor_highspeed_except4,
    home_motor_4,
    go_to_900um,
    low_speed_motor,
    go_home_low_speed,
    wait_motor_stop,
    set_default_speed,
    stop_Homing_proceger,
    start_Homing
}homing;
typedef enum{
    noAction = 0,
    Start_Moves,
    Stop_Moves
}moveMotor;

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
extern int32_t position_speed[][5];

void Board_pin_Config();
void chagneLED(LED_Interval *led);
int8_t Keys_Read_adderss();

void Usart_init_main();
void Usart_init_wifiTest();
void Enable_A4498();
void init_board_config();
void handle_pc_communication(int32_t i_data,ServerCAN *dev,virtualMotor **mot);
void send_to_pc(int32_t iLocation,bool iBottomSensorStat,bool iTopSensorStat,bool iMotorMoving,uint8_t iMorNO);
void for_test_only();
void lcd_refresh_data(virtualMotor **ptrmot);
bool persision_home(uint8_t start,virtualMotor **ptrmot);

bool move_motor(uint8_t start,virtualMotor **ptrmot);
void set_speed(int32_t *ptr,virtualMotor **ptrmot);
void move_releative(int32_t *ptr,virtualMotor **ptrmot);
void move_absolute(int32_t *ptr,virtualMotor **ptrmot);

void SangeMile(virtualMotor **allmot);
void LoleSange(virtualMotor **allmot);

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
    int32_t iLocation = 0;
    uint8_t iData[10] = {};
    uint8_t imotNO = 0;
    bool BotSensor,TopSensor,MotorMoving;


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
    printf("Test uart");

    //for_test_only();
    ServerCAN dev_can(1);
    virtualMotor mot1(1,&dev_can);
    virtualMotor mot2(2,&dev_can);
    virtualMotor mot3(3,&dev_can);
    virtualMotor mot4(4,&dev_can);
    virtualMotor *ptrAllMot[4] = {&mot1,&mot2,&mot3,&mot4};
    Sensor s_start(PIN__Z___Pin);
    Sensor s_reset(PIN_SW___Pin);
    s_reset.set_normal_stat(false);
    s_start.set_normal_stat(false);
    s_start.enable();
    s_reset.enable();
    
    
    //SangeMile(ptrAllMot);
    LoleSange(ptrAllMot);

    t_first = time_us_64() + 400000LL;
    while(t_first > time_us_64());
    while (1)
    {
        //lcd_refresh_data(&mot1,&mot2,&mot3,&mot4);
        lcd_refresh_data(ptrAllMot);
        persision_home(do_nothing,ptrAllMot);
        move_motor(noAction,ptrAllMot);
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
            handle_pc_communication(r_tmp_input,&dev_can,ptrAllMot);
        }
        if(dev_can.read_motLocation(&iLocation,&BotSensor,&TopSensor,&MotorMoving,&imotNO));
        {
            mot1.readCAN(iLocation,BotSensor,TopSensor,MotorMoving,imotNO);
            mot2.readCAN(iLocation,BotSensor,TopSensor,MotorMoving,imotNO);
            mot3.readCAN(iLocation,BotSensor,TopSensor,MotorMoving,imotNO);
            mot4.readCAN(iLocation,BotSensor,TopSensor,MotorMoving,imotNO);
            if(pc_active)
            {
                send_to_pc(iLocation,BotSensor,TopSensor,MotorMoving,imotNO);
            }
        }
        if(pc_active)
        {
            continue;
        }
        if(s_reset.is_triged())
        {
            if(s_reset.get_sensor_stat() == 1)
            {
                if(t_mili_reset_interval > time_us_64() )
                    continue;
                t_mili_reset_interval = time_us_64() + c_reset_interval;
                move_motor(Stop_Moves,ptrAllMot);
                persision_home(start_Homing,ptrAllMot);
            }
        }
        if(s_start.is_triged())
        {
            if(s_start.get_sensor_stat() == 1)
            {
                persision_home(stop_Homing_proceger,ptrAllMot);
                move_motor(Start_Moves,ptrAllMot);
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
void handle_pc_communication(int32_t i_data,ServerCAN *dev,virtualMotor **mot)
{
    static volatile uint8_t r_buff[40] = {};
    static volatile uint8_t r_indx = 0;
    static volatile uint64_t t_mili = 0;
    static volatile int32_t i_val = 0;
    const uint64_t c_delay_Between_Command = 25000ULL;
    static uint8_t motno,reg;
    volatile static uint16_t counter_Message = 0;
    if(t_mili < time_us_64())
        r_indx = 0;
    t_mili = time_us_64() + c_delay_Between_Command;
    r_buff[r_indx] = (i_data & 0xFFLL);
    r_indx++;
    if(r_indx <= 5)
        return;
    r_indx = 0;
    counter_Message++;
    //TODO handler communication from PC
    //dev->send_data(r_buff[1],i_val,r_buff[0]);
    motno = r_buff[0] - 1;
    if(motno >= 4)
        return;
    i_val = 0;
    reg = r_buff[1];
    for(int i=0;i<4;i++)
    {
        i_val = ((int32_t)i_val<<8L);
        i_val |= r_buff[i+2];
    }
    if(reg == PC_SETTOGO)
    {
        mot[motno]->setToGo(i_val);
    }
    else if(reg == PC_SetSpeed)
    {
        mot[motno]->setSpeedUS(i_val,i_val + 1500);
    }
    else if(reg == PC_MoveMotor)
    {
        mot[motno]->Move();
    }
    else if(reg == PC_MoveAll)
    {
        for(int i=0;i<4;i++)
            mot[i]->Move();
    }
    else if(reg == PC_SetDefaulSpeed)
    {
        mot[motno]->setDefaultus(0,0);
    }
    else if(reg == PC_DisableMotor)
    {
        mot[motno]->setEnableMotor(false);
        mot[motno]->synchConfig();
    }
    else if(reg == PC_EnableMotor)
    {
        mot[motno]->setEnableMotor(true);
        mot[motno]->synchConfig();
    }
    else if(reg == PC_GoHome)
    {
        mot[motno]->GoHome();
    }
    else if(reg == PC_PersiceHome)
    {
        persision_home(start_Homing,mot);
    }
}
void send_to_pc(int32_t iLocation,bool iBottomSensorStat,bool iTopSensorStat,bool iMotorMoving,uint8_t iMorNO)
{
    int32_t mostat;
    mostat = (iBottomSensorStat<<3) | (iTopSensorStat<<2) | (iMotorMoving<<1);
    printf("%d %d %d \n\r",(int32_t)iMorNO,iLocation,mostat);
}
void for_test_only()
{
    int32_t iLocation = 0;
    bool BotSensor,TopSensor,MotorMoving;
    uint8_t imotNO;
    int8_t testmotNO;
    int32_t cmNO = 0;
    int8_t lcdLine = 1;
    char r_lcdbuff[80] ={};
    ServerCAN dev_can(1);
    testmotNO = 3;
    virtualMotor testMOT(testmotNO,&dev_can);
    Sensor s_start(PIN__Z___Pin);
    Sensor s_reset(PIN_SW___Pin);
    s_reset.set_normal_stat(false);
    s_start.set_normal_stat(false);
    s_start.enable();
    s_reset.enable();

    testMOT.setEnableEncoder(true);
    testMOT.setDir(false);
    testMOT.setDirEncoder(false);
    testMOT.setSensorBottomNormalStat(false);
    testMOT.setSensorTOPNormalStat(false);
    testMOT.setDefaultus(400,1000LL);
    testMOT.synchConfig();

    while (1)
    {
        s_reset.run();
        s_start.run();
        dev_can.run();
        testMOT.run();
        if(dev_can.read_motLocation(&iLocation,&BotSensor,&TopSensor,&MotorMoving,&imotNO))
        {
            if(testmotNO != imotNO)
                continue;
            testMOT.readCAN(iLocation,BotSensor,TopSensor,MotorMoving,imotNO);
            sprintf(r_lcdbuff,"1:%d        ",testMOT.getLocation());
            r_lcdbuff[19] = 0;
            lcd_setCursor(0,0);
            lcd_print(r_lcdbuff);
            cmNO++;
            if(cmNO > 9)
                cmNO = 0;
            lcdLine++;
            if(lcdLine > 3)
                lcdLine = 1;
            sprintf(r_lcdbuff,"%d-%db%dt%dm%d %d      ",cmNO,imotNO,BotSensor,TopSensor,MotorMoving,iLocation);
            r_lcdbuff[19] = 0;
            lcd_setCursor(lcdLine,0);
            lcd_print(r_lcdbuff);
        }
        if(s_reset.is_triged())
        {
            if(s_reset.get_sensor_stat() == 1)
            {
                testMOT.GoHome();
            }
        }
        if(s_start.is_triged())
        {
            if(s_start.get_sensor_stat() == 1)
            {
                //TODO Here
                testMOT.setToGo(950000LL);
                testMOT.Move();
                //dev_can.send_data(C_DSC_STEP_MOTOR_SET_TOGO_Location,95000LL,2);
            }
        }
        
    }
}
void lcd_buffer_write(char *ibuff,virtualMotor *mot,uint8_t iMotNo)
{
    sprintf(ibuff,"%d %d:%d              ",mot->isBusy(),iMotNo+1,mot->getLocation());
    ibuff[19] = 0;
    lcd_setCursor(iMotNo,0);
    lcd_print(ibuff);
}
void lcd_refresh_data(virtualMotor **ptrmot)
{
    static char r_lcdbuff[40] = {};
    static volatile int64_t mili_refresh;

    if(mili_refresh > time_us_64())
        return;
    mili_refresh = time_us_64() + 980000LL;
    for(uint i = 0;i<4;i++)
    {
        lcd_buffer_write(r_lcdbuff,ptrmot[i],i);
    }
}
bool persision_home(uint8_t iStart,virtualMotor **ptrmot)
{
volatile static uint8_t iCount = 0;
volatile static uint64_t t_mili = 0;
virtualMotor *mot1 = ptrmot[0],
             *mot2 = ptrmot[1],
             *mot3 = ptrmot[2],
             *mot4 = ptrmot[3];
if(iStart == stop_Homing_proceger)
{
    iCount = do_nothing;
}
if((iStart == start_Homing) && (iCount == do_nothing))
{
    iCount = home_motor_highspeed_except4;
}
if(iCount == do_nothing)
    return false;
if(t_mili > time_us_64())
    return !!(iCount);
t_mili = time_us_64() + 800000ULL;
switch (iCount)
{
case do_nothing:
    break;
case home_motor_highspeed_except4:
    mot1->setDefaultus(0,0);
    mot2->setDefaultus(0,0);
    mot3->setDefaultus(0,0);
    mot1->stop();
    mot2->GoHome();
    mot3->GoHome();
    mot4->stop();
    iCount = home_motor_4;
    break;
case home_motor_4:
    if(mot3->getLocation() > 800000LL)
        break;
    mot4->setDefaultus(0,0);
    mot4->GoHome();
    mot1->GoHome();
    iCount = go_to_900um;
    break;
case go_to_900um:
    if(mot1->isBusy())
        break;
    if(mot2->isBusy())
        break;
    if(mot3->isBusy())
        break;
    if(mot4->isBusy())
        break;
    mot1->setToGo(290000LL);
    mot2->setToGo(290000LL);
    mot3->setToGo(290000LL);
    mot4->setToGo(290000LL);
    mot1->Move();
    mot2->Move();
    mot3->Move();
    mot4->Move();
    iCount = low_speed_motor;
    break;
case low_speed_motor:
    if(mot1->isBusy())
        break;
    if(mot2->isBusy())
        break;
    if(mot3->isBusy())
        break;
    if(mot4->isBusy())
        break;
    mot1->setSpeedUS(500,2000);
    mot2->setSpeedUS(500,2000);
    mot3->setSpeedUS(500,2000);
    mot4->setSpeedUS(500,2000);
    iCount = go_home_low_speed;
    break;
case go_home_low_speed:
    mot1->GoHome();
    mot2->GoHome();
    mot3->GoHome();
    mot4->GoHome();
    iCount = wait_motor_stop;
    break;
case wait_motor_stop:
    if(mot1->isBusy())
        break;
    if(mot2->isBusy())
        break;
    if(mot3->isBusy())
        break;
    if(mot4->isBusy())
        break;
    iCount = set_default_speed;
    break;
case set_default_speed:
    mot1->setDefaultus(0,0);
    mot2->setDefaultus(0,0);
    mot3->setDefaultus(0,0);
    mot4->setDefaultus(0,0);
    iCount = do_nothing;
    break;
default:
    iCount = do_nothing;
    break;
}
return !!(iCount);
}
void move_absolute(int32_t *ptr,virtualMotor **ptrmot)
{
    if(ptr[0] != Absolute)
        return;
    for(uint i=0;i<4;i++)
    {
        if(ptr[i+1] == NoChange)
            continue;
        ptrmot[i]->setToGo(ptr[i+1]);
        ptrmot[i]->Move();
    }
}
void move_releative(int32_t *ptr,virtualMotor **ptrmot)
{
    if(ptr[0] != Reletive)
        return;
    for(uint i=0;i<4;i++)
    {
        if(ptr[i+1] == NoChange)
            continue;
        ptrmot[i]->setReleativeToGo(ptr[i+1]);
        ptrmot[i]->Move();
    }
}
void set_speed(int32_t *ptr,virtualMotor **ptrmot)
{
    if(ptr[0] != Speed)
        return;
    for(uint i=0;i<4;i++)
    {
        if(ptr[i+1] == NoChange)
            continue;
        if(ptr[i+1] == DefSpeed)
        {
            ptrmot[i]->setDefaultus(0,0);
            continue;
        }
        ptrmot[i]->setSpeedUS(ptr[i+1],ptr[i+1] + 1500);
    }
}
bool move_motor(uint8_t start,virtualMotor **ptrmot)
{
    static volatile uint16_t iLine = 0;
    static volatile uint64_t t_mili = 0;
    if(start == Stop_Moves)
    {
        iLine = 0;
    }
    if((start == Start_Moves) && (iLine == 0))
    {
        iLine = 1;
    }
    if(iLine == 0)
        return false;
    if(t_mili > time_us_64())
        return true;
    t_mili = time_us_64() + 200000ULL;
    for(uint i = 0;i<4;i++)
    {
        if(ptrmot[i]->isBusy())
            return true;
    }
    if(position_speed[iLine][0] == END_OF_Command)
    {
        iLine = 0;
        return false;
    }
    move_absolute(&position_speed[iLine][0],ptrmot);
    move_releative(&position_speed[iLine][0],ptrmot);
    set_speed(&position_speed[iLine][0],ptrmot);
    iLine++;

return false;
}
void LoleSange(virtualMotor **allmot)
{
    for(uint i=0;i<4;i++)
    {
        allmot[i]->setEnableEncoder(true);
        allmot[i]->setSensorBottomNormalStat(false);
        allmot[i]->setSensorTOPNormalStat(false);
        allmot[i]->setDir(false);
        allmot[i]->setDirEncoder(false);
        allmot[i]->synchConfig();
    }
    allmot[0]->setDefaultus(400LL,1500LL);
    allmot[1]->setDefaultus(65,1500);
    allmot[2]->setDefaultus(400,1500LL);
    allmot[3]->setDefaultus(65,1500);
}
void SangeMile(virtualMotor **allmot)
{
    for(uint i=0;i<4;i++)
    {
        allmot[i]->setEnableEncoder(true);
        allmot[i]->setSensorBottomNormalStat(false);
        allmot[i]->setSensorTOPNormalStat(false);
        allmot[i]->setDir(false);
        allmot[i]->setDirEncoder(false);
    }
    
    allmot[0]->setSensorBottomNormalStat(true);

    allmot[1]->setOtherMotorSensorStop(2);

    allmot[2]->setOtherMotorSensorStop(2);

    allmot[3]->setEnableEncoder(false);
    allmot[3]->setEncoder_nm(500LL);

    allmot[0]->setDefaultus(200LL,1500LL);
    allmot[1]->setDefaultus(25,1500);
    allmot[2]->setDefaultus(25,1500);
    allmot[3]->setDefaultus(25,1500);
    for(uint i=0;i<4;i++)
        allmot[i]->synchConfig();
}
int32_t position_speed[][5] =
{
    {Absolute,NoChange,NoChange,NoChange,NoChange}, // DO NOT DELETE THIS LINE
    //**************Change the Code Bellow***************
    {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},
    // {Absolute,1050000,1200000,12000,18250000},// above sensor
    // {Reletive,NoChange,NoChange,700000,NoChange},// first touch motor 3
    // {Reletive,NoChange,NoChange,-20000,NoChange},

    // {Speed,NoChange,NoChange,7000,NoChange},
    // {Speed,NoChange,NoChange,7000,NoChange},

    // {Reletive,NoChange,NoChange,700000,NoChange},// exact touch motor 3

    // {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},

    // {Reletive,NoChange,NoChange,-100000,NoChange},
    // {Reletive,NoChange,-300000,NoChange,NoChange},
    // {Reletive,NoChange,NoChange,105000,NoChange},
    // {Reletive,NoChange,500000,NoChange,NoChange},// first touch motor 2
    // {Reletive,NoChange,NoChange,-100000,NoChange},
    // {Reletive,NoChange,-20000,NoChange,NoChange},

    // {Speed,NoChange,7000,NoChange,NoChange},
    // {Speed,NoChange,7000,NoChange,NoChange},

    // {Reletive,NoChange,NoChange,100000,NoChange},
    // {Reletive,NoChange,600000,NoChange,NoChange},// exact touch motor 2
    // {Reletive,NoChange,NoChange,-500000,NoChange},
    // {Reletive,NoChange,NoChange,NoChange,-7000000},

    // {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},
    
    // {Reletive,NoChange,3500000,NoChange,NoChange},
    // {Reletive,NoChange,NoChange,672500,NoChange},// p1 tangent

    // {Speed,NoChange,NoChange,10000,NoChange},
    // {Speed,NoChange,NoChange,10000,NoChange},

    // {Reletive,NoChange,NoChange,170000,NoChange},// p1 grind
    // {Reletive,NoChange,-1327000,NoChange,NoChange},// p2 m2 back
    // {Reletive,140000,NoChange,NoChange,NoChange},// p2 m1 adjust

    // {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},

    // {Reletive,NoChange,NoChange,762000,NoChange},// p2 tangent

    // {Speed,NoChange,NoChange,10000,NoChange},
    // {Speed,NoChange,NoChange,10000,NoChange},

    // {Reletive,NoChange,NoChange,10000,NoChange},// p2-1 grind

    // {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},

    // {Reletive,NoChange,NoChange,-208500,NoChange},// tabdil m3 
    // {Reletive,-360000,NoChange,NoChange,NoChange},// tabdil m1
    // {Reletive,NoChange,NoChange,149500,NoChange},// tangent p2-2

    // {Speed,NoChange,NoChange,10000,NoChange},
    // {Speed,NoChange,NoChange,10000,NoChange},

    // {Reletive,NoChange,NoChange,30000,NoChange},// grind p2-2
    
    // {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},

    // {Absolute,NoChange,45000,45000,NoChange},// above sensor
    // {Absolute,30000,NoChange,NoChange,30000},// above sensor

    //DO NOT DELETE BELLOW LINE
    {END_OF_Command,NoChange,NoChange,NoChange,NoChange},
    {END_OF_Command,NoChange,NoChange,NoChange,NoChange},
};
