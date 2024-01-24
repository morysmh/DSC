#include "define.h"
#include "canserver.h"
#ifndef __MOTOR__CLASS__
#define __MOTOR__CLASS__
class virtualMotor
{
public:
    virtualMotor(uint8_t add,CanCotroll *ptrcan);
    void setDiv(int32_t iDiv);
    void setPid(int32_t kp,int32_t ki,int32_t kd);
    void setEnableMotor(bool iVal);
    void setDir(bool iVal);
    void setDirEncoder(bool iVal);
    void setEnablePID(bool iVal);
    void setEnableEncoder(bool iVal);
    void setSensorBottomNormalStat(bool iVal);
    void setSensorTOPNormalStat(bool iVal);
    void setSpeedUS(int32_t iLow,int32_t iHigh);
    void setDefaultLow(int32_t iDefLow);
    void setOtherMotorSensorStop(int8_t iMotNo);
    void setEncoder_nm(int32_t inm);
    void setToGo(int32_t itogo);
    void stop();
    bool isBusy();
    bool isHome();
    void readCAN(int8_t motNO,int8_t iPara,int32_t iVal);
    void GoHome();

    int32_t getLocation() const {return pCurrentPosition;}

    void run();
private:
    bool is_near();
    void readPos();
    void readSensor();
    int32_t pEncoder_nm = 500LL;
    int32_t pCurrentPosition = 0;
    int32_t pToGoPosition = 0;
    int32_t pLowDelayPulse = 500LL;
    int32_t pHighDelayPulse = 1500LL;
    int32_t pDefLow = 500LL;
    bool pMotorEnable = false;
    bool pBottomSensorStat = false;
    bool pTOPSensorTrig = false;
    CanCotroll *ptr_can;
    int8_t pAddressMotor = 0;
    int64_t miliReadPos = 0;
    int64_t miliSensorRead = 0;
    const int64_t cIntervalReadPos = 1243106LL;
    int8_t pOtherMotorSensorStop = 0;
};

#endif