#include "define.h"
#include "mcp2515.h"

class CanCotroll
{
public:
    CanCotroll(uint8_t i_add);
    ~CanCotroll(){}
    void run();
    void set_other_sensor_code(uint8_t iOtherMotor);
    void send_status(int32_t iLocation,bool isMoving, bool iSensorBottom,bool iSensorTOP,bool iSendNow);
    bool read_new_message(uint8_t *ptrdata,uint8_t *motNO,uint8_t *iPara);
    bool read_other_motor_stat(uint8_t *istat);
    void set_interval(uint8_t iVal);

    void ptr8_to_int32(uint8_t *ptrdata,int32_t *iout);
    void int32_to_ptrint8(int32_t iVal,uint8_t *ptrdata);
private:
    void handle_Self_OtherSensor_Status(uint8_t status);//Other Sensor address same as this handler
    uint32_t set_bv(uint8_t iBv);
    typedef struct{
        uint8_t iPara;
        uint8_t imotNO;
        uint8_t idata[7];
    }storage;
    void handle_message();
    void handle_other_sensor();
    uint8_t ringbuff_adder(uint8_t RbIndex, int8_t iVal);
    uint8_t p_address = 0;
    uint16_t p_other_sensor_address = 0;
    uint8_t p_rx_head = 0,p_rx_tail = 0;
    const uint8_t c_buff_size = 35;
    storage p_rx[40] = {};
    tCAN p_canRX = {},p_canSend = {};
    bool pSendAvailable = false;
    uint64_t c_interval_status = 15000ULL;
    uint64_t pt_interval = 0;

    bool pOtherSensorStatChange = false;
    uint8_t pOtherMotorStat = 0;
};