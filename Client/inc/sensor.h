#include "define.h"
class Sensor
{
public:
    enum SensorType{
        END_Sensor,
        Zero_Sensor
    };
    Sensor(uint8_t i_pin,Sensor::SensorType i_sen = Zero_Sensor);
    ~Sensor(){}
    void set_normal_stat(bool i_normal){p_normal_stat = i_normal;}
    bool get_sensor_stat() const;
    bool is_triged();
    void run();
    void enable();
    void disable(){p_is_enable = false;}
private:
    const uint64_t c_hyst_us = 20000ULL;
    const uint64_t c_fast_run_prevent = 10ULL;
    int8_t p_pinNO = -1;
    uint64_t p_fast_run = 0,p_change_mili = 0;
    Sensor::SensorType p_sensorType;
    bool p_normal_stat = false;
    bool p_current_stat = false;
    bool p_is_trig = true;
    bool p_is_enable = false;
};