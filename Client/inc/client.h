#include "define.h"
#include "motor.h"
#include "sensor.h"
#include "encoder.h"
#include "can.h"



class Client
{
public:
    Client( 
            uint8_t pin_clk,
            uint8_t pin_dir,
            uint8_t pin_Bottom_Sensor,
            uint8_t pin_TOP_Sensor,
            uint8_t pin_Encoder_A,
            uint8_t pin_DIV1,
            uint8_t pin_DIV2,
            uint8_t pin_DIV3,
            uint8_t pin_en,
            uint8_t pin_sleep,
            uint8_t can_address
            );
    ~Client();
    void run();
    void CheckFailure();
    void increse_sofware_encoder(int8_t i_val);
    void software_fake_message(int32_t iPara,uint8_t *rbuff);
private:
    void check_message();
    void handle_message(uint8_t *rbuff,uint8_t motNO,uint8_t iPara);
    void send_LocationData(bool isNow);
    bool _bv(uint32_t iVal,uint8_t iBV);
    void config_register_handle(uint8_t *data);
    void speed_register_handle(uint8_t *data);
    void togo_register_handle(uint8_t *data);
    void Releative_togo_register_handle(uint8_t *data);
    void status_register_handle(uint8_t iStat);
    void Star_Stop_register_handle(uint8_t *data);
    void PID_register_handle(uint8_t *data);

    bool pReportingStaus = false;
    uint8_t pPrevReportStat = 0;
    Encoder *o_encoder;
    Sensor *o_top,*o_bottom;
    StepMotor *o_stepmotor;
    CanCotroll *o_can;

    uint64_t tFailCheck = 0;
    bool bFailure = false;
    bool bConfigure = false;
    int32_t rEncoderVal = 0;
    int32_t rFailureHyst = 0;
    const int32_t cFailureHystCount = 200;
    uint64_t c_interval_status = 15138ULL;
    uint64_t pt_interval = 0;
};
