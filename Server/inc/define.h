
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
#define PIN_TMC_STEP    14
#define PIN_TMC__DIR    15
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
    PC_SETTOGO = 1,
    PC_SetSpeed = 2,
    PC_MoveMotor = 3,
    PC_MoveAll = 4,
    PC_SetDefaulSpeed = 5,
    PC_DisableMotor = 6,
    PC_EnableMotor = 7,
    PC_GoHome = 8,
    PC_PersiceHome = 9


}ENUM_Command_List;

typedef enum{
    C_DSC_Server_ADDRESS_CAN = 0xFF,
    C_DSC_BRODCAST_ADDRESS_CAN = 0xF0,
    C_DSC_Null = 0,
    C_DSC_ARRAY_OPCODE = 0,
    C_DSC_OPCODE_REG_CONFIG = 1,
    C_DSC_OPCODE_REG_SPEED = 2,
    C_DSC_OPCODE_REG_TOGO = 3,
    C_DSC_OPCODE_REG_STATUS = 4,
    C_DSC_OPCODE_REG_START_STOP = 5,
    C_DSC_OPCODE_REG_PID_CONFIG = 6,
    C_DSC_OPCODE_REG_Releative_TOGO = 7,

    C_DSC_ARRAY_CONFIG_REG_CONF_1byte = 0,
    C_DSC_ARRAY_CONFIG_REG_CONF_2byte = 1,
    C_DSC_ARRAY_CONFIG_REG_POSITION_INTERVAl = 2,
    C_DSC_ARRAY_CONFIG_REG_FROM_OTHER_MOTOR_STATUS_READ = 3,
    C_DSC_ARRAY_CONFIG_REG_DIV = 4,
    C_DSC_ARRAY_CONFIG_REG_ENCODER_RES_1byte = 5,
    C_DSC_ARRAY_CONFIG_REG_ENCODER_RES_2byte = 6,

    C_DSC_ARRAY_PID_REG_KP = 0,
    C_DSC_ARRAY_PID_REG_KI = 1,
    C_DSC_ARRAY_PID_REG_KD = 2,

    C_DSC_ARRAY_SPEED_REG_LOW_us_1byte = 0,
    C_DSC_ARRAY_SPEED_REG_LOW_us_2byte = 1,
    C_DSC_ARRAY_SPEED_REG_HIGH_us_1byte = 2,
    C_DSC_ARRAY_SPEED_REG_HIGH_us_2byte = 3,

    C_DSC_ARRAY_TOGO_REG_index = 1,

    C_DSC_ARRAY_Releative_TOGO_REG_index = 1,

    C_DSC_ARRAY_STATUS_REPORT_1byte = 0,
    C_DSC_ARRAY_STATUS_LOCATION_index = 1,

    C_DSC_ARRAY_StartStop = 0,

    C_DSC_BIT_STATUS_MOTOR_MOVING = 0,
    C_DSC_BIT_STATUS_SENSOR_BOTTOM_STATUS = 1,
    C_DSC_BIT_STATUS_SENSOR_TOP_STATUS = 2,

    C_DSC_BIT_CONFIG_ENCODER_HARDWARE = 8,
    C_DSC_BIT_CONFIG_BOTTOM_SENSOR_DEFAULT = 7,
    C_DSC_BIT_CONFIG_TOP_SENSOR_DEFAULT = 6,
    C_DSC_BIT_CONFIG_SENSOR_ENABLE_BOTTOM = 5,
    C_DSC_BIT_CONFIG_SENSOR_ENABLE_TOP = 4,
    C_DSC_BIT_CONFIG_MOTOR_DEFAULT_DIRECTION = 3,
    C_DSC_BIT_CONFIG_ENCODER_DEFAULT_DIRECTION = 2,
    C_DSC_BIT_CONFIG_MOTOR_ENABLE = 1,
    C_DSC_BIT_CONFIG_PID_ENABLE = 0,


    C_DSC_BIT_START_MOVING = 0,
    C_DSC_BIT_STOP_MOVING = 1,
    C_DSC_BIT_STOP_REPORTING = 2,

}Enum_DSC;
#endif