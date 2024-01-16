
#ifndef MainMenu_Header
#define	MainMenu_Header

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/pio.h"


#define UART_ID uart0
#define BAUD_RATE 38400
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define ENDSTDIN -1


#define Led_Pin_PICO 25
#define PICO_LED_Stat(x) gpio_put(Led_Pin_PICO, x)

#define STEP_STEP_Pin(x) //gpio_put(PIN_TMC_STEP, x)
#define STEP_Dir_Pin(x) //gpio_put(PIN_TMC__DIR, x)

#define PIN_ADD_Pin1    0
#define PIN_ADD_Pin2    1
#define PIN_CAN__SCK    2
#define PIN_CAN_MOSI    3
#define PIN_CAN_MISO    4
#define PIN_CAN___CS    5
#define PIN_TMC_DIG0    6
#define PIN_TMC__Ain    7
#define PIN_TMC___EN    8
#define PIN_SPI_MOSI    9
#define PIN_SPI__SCK    10
#define PIN_SPI__Csn    11
#define PIN_SPI_MISO    12
#define PIN_TMC__CLK    13
//#define PIN_TMC_STEP    14
//#define PIN_TMC__DIR    15
#define PIN__A___Pin    16
#define PIN__B___Pin    17
#define PIN__Z___Pin    18
#define PIN_SW___Pin    19
#define PIN_LED__RED    20
#define PIN_LED_Gren    21
#define PIN_CAN__INT    22
#define PIN_ADD_Pin3    28


#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define STEPCIR_MAX_SIZE  256

typedef struct
{
    int64_t OFF;
    int64_t _ON;
    int64_t lastCheck;
    int8_t stat;
} LED_Interval;
typedef enum {
    C_Interface_NULL = 0,
    C_Interface_STEP_CTRL = 1,
    C_Interface_STEP_Pulse = 2,
    C_Interface_CAN = 3,
    C_Interface_INPUTSTOP = 4,
    C_Interface_ENCODER = 5,
    C_Interface_SERVER
}ENUM_Interfaces;
typedef struct
{
    int64_t value;
    int64_t para;
    ENUM_Interfaces source;
    ENUM_Interfaces dest;
    
} CallBack_Parameter;

typedef enum{
       C_Server_ADDRESS_CAN = 0xFF,
    C_Command_NULL = 0,
    C_command_STEP_Pulse_set_STOP = 1,
    C_command_STEP_Pulse_set_step = 2,
    C_command_STEP_Pulse_set_duration = 3,
    C_command_STEP_Pulse_set_acc = 4,
    C_command_STEP_Pulse_set_interval_acc = 5,
    C_command_STEP_Pulse_set_MAX = 6,
    C_command_STEP_Pulse_set_min = 7,
    C_command_STEP_Pulse_get_remain = 8,

    C_command_STEP_CTRL_set_pulse_per_100mm = 9,
    C_command_STEP_CTRL_set_microstep = 10,
    C_command_STEP_CTRL_set_direction_default = 11,
    C_command_STEP_CTRL_set_endstop = 12,
    C_command_STEP_CTRL_set_topstop = 13,
    C_command_STEP_CTRL_get_current_position = 14,
    C_command_STEP_CTRL_DisableMotor = 15,
    C_command_STEP_CTRL_EnableMotor = 16,
    C_command_STEP_CTRL_set_TO_GO_nm = 17,
    C_command_STEP_CTRL_set_encoder_feedback = 18,

    C_command_Encoder_get_position = 19,
    C_command_Encoder_set_zero = 20,
    C_command_Encoder_set_nm_per_pulse = 21,
    C_command_Encoder_get_nm_pp = 22,
    C_command_Encoder_set_direction = 23,
    C_command_STEP_CTRL_go_home = 24,

    C_command_Keys_get_is_min_trig = 25,
    C_command_Keys_get_is_MAX_trig = 26,
    C_command_Keyd_default_min = 27,
    C_command_Keyd_default_MAX = 28,
    C_command_STEP_CTRL_Home_Direction = 29,

    C_command_STEP_Pulse_get_duration = 30


}ENUM_Command_List;
#endif