
#include "define.h"
#include "mcp2515.h"
#include "quadrature.pio.h"
#include "pio_rotary_encoder.pio.h"
#include "i2c-display-lib.h"
#include "canserver.h"
#include "VirtualMotor.h"
#include "sensor.h"

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
extern int32_t home_pos[][5];

void Board_pin_Config();
void chagneLED(LED_Interval *led);
int8_t Keys_Read_adderss();

void Usart_init_main();
void Usart_init_wifiTest();
void Enable_A4498();
void init_board_config();
void handle_pc_communication(int32_t i_data,ServerCAN *dev,virtualMotor **mot);
void send_to_pc(virtualMotor **ptrmot);
void for_test_only();
void lcd_refresh_data(virtualMotor **ptrmot);

void ReCoverMotorError(virtualMotor **ptrmot);
bool oneStepMove(int32_t *ptr,Sensor *sUp,Sensor *sDown,virtualMotor **ptrmot);
bool FailureCheck(virtualMotor **ptrmot);
bool move_motor(uint8_t start,virtualMotor **ptrmot,int32_t (*ptrCommand)[5]);
void set_speed(int32_t *ptr,virtualMotor **ptrmot);
void move_releative(int32_t *ptr,virtualMotor **ptrmot);
void move_absolute(int32_t *ptr,virtualMotor **ptrmot);
void setIndependent_Move(int32_t *i_ptr,independentStruct *a_mot);
void GoHome_Motor(int32_t *i_ptr,virtualMotor **ptrMot);
void Stop_Motor_Moving(int32_t *i_ptr,virtualMotor **ptrMot);
void StatusTOP_Motor(int32_t *i_ptr,virtualMotor **ptrMot);
bool move_independent_motor(independentStruct *i_indep,virtualMotor **ptrmot);


void SangeMile(virtualMotor **allmot);
void LoleSange(virtualMotor **allmot);

int main()
{
    LED_Interval Pico_LED;
    volatile int8_t add = 0;

    volatile int32_t r_tmp_input = 0;

    char r_lcdbuff[80] = {};
    bool pc_active = false;
    volatile uint64_t t_first = time_us_64() + 1750000ULL;
    int32_t iLocation = 0;
    uint8_t iData[10] = {};
    uint8_t imotNO = 0;
    int16_t statusData = 0;
    bool bFailure;


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
    lcd_init(14,15);
    lcd_setCursor(0,0);
    lcd_print("********************");
    lcd_setCursor(1,0);
    lcd_print("***Initialization***");
    lcd_setCursor(2,0);
    lcd_print("********************");
    printf("Test uart");
    while(t_first > time_us_64());

    //for_test_only();
    ServerCAN dev_can(1);
    virtualMotor mot1(1,&dev_can);
    virtualMotor mot2(2,&dev_can);
    virtualMotor mot3(3,&dev_can);
    virtualMotor mot4(4,&dev_can);
    virtualMotor *ptrAllMot[4] = {&mot1,&mot2,&mot3,&mot4};
    Sensor s_start(PIN__Z___Pin);
    s_start.setHystus(192000ULL);
    s_start.set_normal_stat(false);
    s_start.enable();
    Sensor s_reset(PIN_SW___Pin);
    s_reset.setHystus(192000ULL);
    s_reset.set_normal_stat(false);
    s_reset.enable();
    
    for(uint i=1;i;i++){
        bool failstat = false;
        SangeMile(ptrAllMot);
        //LoleSange(ptrAllMot);

        t_first = time_us_64() + 400000LL;
        while(t_first > time_us_64()){
            s_reset.run();
            s_start.run();
            dev_can.run();
            mot1.run();
            mot2.run();
            mot3.run();
            mot4.run();
        }
	    s_start.is_triged();
	    s_reset.is_triged();
        for(uint jj=0;jj<4;jj++){
            if(ptrAllMot[jj]->isError())
                failstat = true;
        }
        if((failstat == false) && (i>3))
            break;
        if(i>10){
            lcd_clear();
            lcd_setCursor(0,0);
            lcd_print("Not able to Start");
            lcd_setCursor(1,0);
            lcd_print("client failure");
            lcd_setCursor(2,0);
            lcd_print("mot ");
            if(mot1.isError())
                lcd_print("1,");
            if(mot2.isError())
                lcd_print("2,");
            if(mot3.isError())
                lcd_print("3,");
            if(mot4.isError())
                lcd_print("4");
            lcd_print(" Fault");
            while(1);
        }
    }
    lcd_clear();
    //pc_active = true;
    while (1)
    {
        bFailure = FailureCheck(ptrAllMot);
        lcd_refresh_data(ptrAllMot);
        move_motor(noAction,ptrAllMot,nullptr);
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
        if(dev_can.read_motLocation(&iLocation,&statusData,&imotNO));
        {
            mot1.readCAN(iLocation,statusData,imotNO);
            mot2.readCAN(iLocation,statusData,imotNO);
            mot3.readCAN(iLocation,statusData,imotNO);
            mot4.readCAN(iLocation,statusData,imotNO);
            //if(pc_active){
                //send_to_pc(iLocation,
                //statusData & (1<<C_DSC_BIT_STATUS_SENSOR_TOP_STATUS),
                //statusData & (1<<C_DSC_BIT_STATUS_SENSOR_BOTTOM_STATUS),
                //statusData & (1<<C_DSC_BIT_STATUS_MOTOR_MOVING),imotNO);
            //}
        }
        if(pc_active){
            send_to_pc(ptrAllMot);
            continue;
        }
        if(s_reset.is_triged()){
            if((s_reset.get_sensor_stat() == s_reset.get_normal_stat())){
                ReCoverMotorError(ptrAllMot);
                move_motor(Start_Moves,ptrAllMot,home_pos);
            }
        }
        if(s_start.is_triged()){
            if((s_start.get_sensor_stat() == s_start.get_normal_stat()) && (!bFailure)){
                move_motor(Start_Moves,ptrAllMot,position_speed);
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
        for(uint8_t i=0;i<4;i++){
            mot[i]->Move(); 
        }
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
        move_motor(Start_Moves,mot,home_pos);
    }
    else if(reg == PC_RecoverMotor){
        mot[motno]->RecoverError();
    }
}
void send_to_pc(virtualMotor **ptrmot)
{
    int32_t mostat;
    static uint64_t tmili = 0;
    if(tmili > time_us_64())
        return;
    tmili = time_us_64() + 900;
    for(uint8_t i=0;i<4;i++){
        if(!ptrmot[i]->PcDataAvailable())
            continue;
        mostat = (((uint32_t)ptrmot[i]->isHome())<<3);
        mostat |= (((uint32_t)ptrmot[i]->isBusy())<<1);
        printf("%d %d %d \n\r",(int32_t)ptrmot[i]->getMotNo(),ptrmot[i]->getLocation(),mostat);
    }
    //mostat = (iBottomSensorStat<<3) | (iTopSensorStat<<2) | (iMotorMoving<<1);
}
void for_test_only()
{
    int32_t iLocation = 0;
    int16_t statusData;
    uint8_t imotNO;
    int8_t testmotNO;
    int32_t cmNO = 0;
    int8_t lcdLine = 1;
    char r_lcdbuff[80] ={};
    ServerCAN dev_can(1);
    testmotNO = 4;
    virtualMotor testMOT(testmotNO,&dev_can);
    Sensor s_start(PIN__Z___Pin);
    Sensor s_reset(PIN_SW___Pin);
    s_reset.set_normal_stat(false);
    s_start.set_normal_stat(false);
    s_start.enable();
    s_reset.enable();

    testMOT.setEnableEncoder(false);
    testMOT.setEncoder_nm(500LL);
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
        if(dev_can.read_motLocation(&iLocation,&statusData,&imotNO))
        {
            if(testmotNO != imotNO)
                continue;
            testMOT.readCAN(iLocation,statusData,imotNO);
            //sprintf(r_lcdbuff,"1:%d        ",testMOT.getLocation());
            r_lcdbuff[19] = 0;
            lcd_setCursor(0,0);
            lcd_print(r_lcdbuff);
            cmNO++;
            if(cmNO > 9)
                cmNO = 0;
            lcdLine++;
            if(lcdLine > 3)
                lcdLine = 1;
            //sprintf(r_lcdbuff,"%d-%db%dt%dm%d %d      ",cmNO,imotNO,BotSensor,TopSensor,MotorMoving,iLocation);
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
    //sprintf(ibuff,"%d %d:%d              ",mot->isBusy(),iMotNo+1,mot->getLocation());
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
        ptrmot[i]->lcdData(r_lcdbuff);
        r_lcdbuff[19] = 0;
        lcd_setCursor(ptrmot[i]->getMotNo() - 1,0);
        lcd_print(r_lcdbuff);
    }
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
    }
    for(uint i=0;i<4;i++)
    {
        if(ptr[i+1] == NoChange)
            continue;
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
    }
    for(uint i=0;i<4;i++)
    {
        if(ptr[i+1] == NoChange)
            continue;
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
bool oneStepMove(int32_t *ptr,Sensor *sUp,Sensor *sDown,virtualMotor **ptrmot)
{
    int32_t move[5] = {};
    if(ptr[0] != StallExecution)
        return false;
    move[0] = Reletive;
    if((sUp->is_triged()) && (sUp->get_sensor_stat() == 1))
    {
        for(int i=1;i<5;i++)
            move[i] = ptr[i];
        move_releative(move,ptrmot);
    }
    if((sDown->is_triged()) && (sDown->get_sensor_stat() == 1))
    {
        for(int i=1;i<5;i++)
            move[i] = ptr[i] * (-1);
        move_releative(move,ptrmot);
    }
return true;
}
void ReCoverMotorError(virtualMotor **ptrmot){
    int8_t countError = 0;
    for(uint i=0;i<4;i++){
        if(!ptrmot[i]->isError())
            continue;
        countError++;
    }
    if(countError <= 0)
        return;
    for(uint i=0;i<4;i++){
        ptrmot[i]->RecoverError();
    }
}
bool FailureCheck(virtualMotor **ptrmot){
    static volatile uint64_t t_mili = 0;
    static bool pfaile = false;
    int8_t countError = 0;
    if(t_mili > time_us_64())
        return pfaile;
    t_mili = time_us_64() + 4000;
    pfaile = false;
    for(uint i=0;i<4;i++){
        if(!ptrmot[i]->isError())
            continue;
        t_mili = time_us_64() + 5000000ULL;
        countError++;
        pfaile = true;
    }
    if(countError == 0){
        pfaile = false;
        return false;
    }
    if(countError >= 4)
        return pfaile;
    for(uint i=0;i<4;i++){
        ptrmot[i]->shutmotor();
        t_mili = time_us_64() + 5000000ULL;
    }
    return true;

}
bool move_motor(uint8_t start,virtualMotor **ptrmot,int32_t (*ptrCommand)[5])
{
    static volatile uint16_t iLine = 0;
    static volatile uint64_t t_mili = 0;
    static independentStruct a_IndepMot[10] = {};
    static Sensor sDown(PIN__A___Pin);
    static Sensor sUp(PIN__B___Pin);
    static volatile int32_t (*pAction)[5] = position_speed;
    sUp.set_normal_stat(false);
    sDown.set_normal_stat(false);
    sUp.enable();
    sDown.enable();
    sUp.run();
    sDown.run();
    if(start == Stop_Moves)
    {
        iLine = 0;
    }
    if((start == Start_Moves) && (ptrCommand != nullptr) && (pAction != ptrCommand))
    {
        pAction = ptrCommand;
        iLine = 1;
    }
    if((start == Start_Moves) && (iLine == 0))
    {
        iLine = 1;
    }
    if((start == Start_Moves) && (pAction[iLine][0] == StallExecution))
    {
        iLine++;
    }
    if(iLine == 0)
        return false;
    move_independent_motor(a_IndepMot,ptrmot);
    Stop_Motor_Moving((int32_t *)&pAction[iLine][0],ptrmot);
    if(t_mili > time_us_64())
        return true;
    t_mili = time_us_64() + 200000ULL;
    for(uint i = 0;i<4;i++)
    {
        if(ptrmot[i]->isBusy())
            return true;
    }
    if(pAction[iLine][0] == StallExecution)
    {
        oneStepMove((int32_t *)&pAction[iLine][0],&sUp,&sDown,ptrmot);
        return true;
    }

    if(pAction[iLine][0] == Motor_delay)
    {
        t_mili = time_us_64() + ((uint64_t)pAction[iLine][1]);
        iLine++;
        return true;
    }
    if(pAction[iLine][0] == END_OF_Command)
    {
        iLine = 0;
        return false;
    }
    move_absolute((int32_t *)&pAction[iLine][0],ptrmot);
    move_releative((int32_t *)&pAction[iLine][0],ptrmot);
    set_speed((int32_t *)&pAction[iLine][0],ptrmot);
    setIndependent_Move((int32_t *)&pAction[iLine][0],a_IndepMot);
    StatusTOP_Motor((int32_t *)&pAction[iLine][0],ptrmot);
    GoHome_Motor((int32_t *)&pAction[iLine][0],ptrmot);

    iLine++;

    return true;
}
void LoleSange(virtualMotor **allmot){
    for(uint i=0;i<4;i++)
    {
        allmot[i]->setEnableEncoder(true);
        allmot[i]->setSensorBottomNormalStat(false);
        allmot[i]->setSensorTOPNormalStat(false);
        allmot[i]->setDir(false);
        allmot[i]->setDirEncoder(false);
    }
    allmot[0]->setDefaultus(600LL,2500LL);
    allmot[1]->setDefaultus(25,1500);
    allmot[2]->setDefaultus(400,2500LL);
    allmot[3]->setDefaultus(65,1500);
    for(uint i=0;i<4;i++){
        allmot[i]->synchConfig();
        allmot[i]->RecoverError();
    }
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
    allmot[1]->setSensorBottomNormalStat(true);
    allmot[1]->setSensorTOPNormalStat(true);

    allmot[2]->setOtherMotorSensorStop(2);

    allmot[3]->setEnableEncoder(false);
    allmot[3]->setEncoder_nm(500LL);

    allmot[0]->setDefaultus(200LL,1500LL);
    allmot[1]->setDefaultus(25,1500);
    allmot[2]->setDefaultus(25,1500);
    allmot[3]->setDefaultus(25,1500);
    for(uint i=0;i<4;i++){
        allmot[i]->synchConfig();
        allmot[i]->RecoverError();
    }
}
bool move_independent_motor(independentStruct *i_indep,virtualMotor **ptrmot)
{
    volatile static uint64_t t_mili = 0;
    if(t_mili > time_us_64())
        return false;
    t_mili = time_us_64() + 120000ULL;
    for(uint i=0;i<4;i++)
    {
        if(ptrmot[i]->isBusy())
            continue;
        if(i_indep[i].enable == false)
            continue;
        ptrmot[i]->setToGo(i_indep[i].nextMove);
        ptrmot[i]->Move();
        if(i_indep[i].nextMove == i_indep[i].start)
            i_indep[i].nextMove = i_indep[i].stop;   
        else
            i_indep[i].nextMove = i_indep[i].start;   
    }
    return true;
}

void setIndependent_Move(int32_t *i_ptr,independentStruct *a_mot)
{
    if(i_ptr[0] == First_Independent_absolute_Set)
    {
        for(uint i=0;i<4;i++)
        {
            if(i_ptr[i+1] != NoChange)
            {
                a_mot[i].start = i_ptr[i+1];
                a_mot[i].nextMove = i_ptr[i+1];
            }
        }
    }
    else if(i_ptr[0] == Last_Independent_absolute_Set)
    {
        for(uint i=0;i<4;i++)
        {
            if(i_ptr[i+1] != NoChange)
                a_mot[i].stop = i_ptr[i+1];
        }
    }
    else if(i_ptr[0] == Start_IndependentMove)
    {
        for(uint i=0;i<4;i++)
        {
            if(i_ptr[i+1] != NoChange)
                a_mot[i].enable = true;
        }
    }
    else if(i_ptr[0] == STOP_IndependentMove)
    {
        for(uint i=0;i<4;i++)
        {
            if(i_ptr[i+1] != NoChange)
                a_mot[i].enable = false;
        }
    }
}
void GoHome_Motor(int32_t *i_ptr,virtualMotor **ptrMot)
{
    if(i_ptr[0] != GoHomeCommand)
        return;
    for(uint i=0;i<4;i++)
    {
        if(i_ptr[i+1] == Enable)
            ptrMot[i]->GoHome();
    }

}
void Stop_Motor_Moving(int32_t *i_ptr,virtualMotor **ptrMot)
{
    if(i_ptr[0] != StopMotorMoving)
        return;
    for(uint i=0;i<4;i++)
    {
        if(i_ptr[i+1] == Enable)
            ptrMot[i]->stop();
    }

}
void StatusTOP_Motor(int32_t *i_ptr,virtualMotor **ptrMot)
{
    if(i_ptr[0] == TOP_SENSOR_STAT)
    {
        for(uint i=0;i<4;i++)
        {
            if(i_ptr[i+1] == Enable)
                ptrMot[i]->TopSensorStat(true);
            else
                ptrMot[i]->TopSensorStat(false);
        }
    }
    if(i_ptr[0] == LOCK_MOTOR)
    {
        for(uint i=0;i<4;i++)
        {
            if(i_ptr[i+1] == Enable)
                ptrMot[i]->LockStat(true);
            else
                ptrMot[i]->LockStat(false);
        }
    }
}
