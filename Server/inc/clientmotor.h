#include "define.h"
#include "canserver.h"
#ifndef __MOTOR__CLASS__
#define __MOTOR__CLASS__
class virtualMotor
{
public:
    virtualMotor(uint8_t add,ServerCAN *ptrcan);
    void setDiv(uint8_t iDiv);
    void setPid(uint8_t kp,uint8_t ki,uint8_t kd);
    void setEnableMotor(bool iVal);
    void setDir(bool iVal);
    void setDirEncoder(bool iVal);
    void setEnablePID(bool iVal);
    void setEnableEncoder(bool iVal);
    void setSensorBottomNormalStat(bool iVal);
    void setSensorTOPNormalStat(bool iVal);
    void setSpeedUS(uint16_t iLow,uint16_t iHigh);
    void setDefaultus(uint16_t iDefLow,uint16_t iDefHigh);
    void setOtherMotorSensorStop(uint8_t iMotNo);
    void setEncoder_nm(uint16_t inm);
    void setToGo(int32_t itogo);
    void setReleativeToGo(int32_t itogo);
    void setReportInterval(uint8_t iVal);
    void stop();
    bool isBusy() const {return pMotorMoving;}
    bool isHome() const {return pSensorStatBOTTOM;}
    void readCAN(int32_t iLocation,bool iBottomSensorStat,bool iTopSensorStat,bool iMotorMoving,uint8_t iMorNO);
    void GoHome();
    void synchConfig();
    void Move();

    int32_t getLocation() const {return pCurrentPosition;}

    void run();
private:
    void writeConfigRegMotor(uint8_t bit,bool val);
    uint8_t pdataConfig[10] = {};
    uint16_t pLowDelayPulse = 5000LL;
    uint16_t pHighDelayPulse = 15000LL;
    uint16_t pDefLow = 500LL,pDefHigh = 1500LL;
    int32_t pCurrentPosition = 0;
    int32_t pToGoPosition = 0;
    void communicate_to_can(uint8_t ipara,uint8_t *idata);
    ServerCAN *ptrCAN;
    uint8_t pAddressMotor = 0;
    bool pSensorStatTOP = false;
    bool pSensorStatBOTTOM = false;
    bool pMotorMoving = false;
};

#endif