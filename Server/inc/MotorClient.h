#include "define.h"
#include "canserver.h"
#ifndef __MOTOR__Client__CLASS
#define __MOTOR__Client__CLASS
class MotorClient
{
public:
    MotorClient(uint8_t add,ServerCAN *ptrcan);
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
    void setReportInterval(uint8_t iVal);
    void LockStat(bool Lock);
    void TopSensorStat(bool stat);
    void synchConfig();

    void readCAN(int32_t iLocation,int16_t statusData,uint8_t iMorNO);

    void setMotorPosition(int32_t itogo);
    void stop();

    uint8_t getMotNo() const {return pAddressMotor;}
    int32_t getLocation() const {return pCurrentPosition;}

protected:
    bool getFlag_NewReport() {
        if(!pNewReport)
            return false;
        pNewReport = false;
        return true;
        }
    void setFailureRecover();
    bool getFlag_Failued() const {return pDSCFailure;}
    bool getFlag_NotConfig() const {return pDSCnotConfig;}
    bool getFlag_Busy() const {return pMotorMoving;}
    bool getFlag_Home() const {return pSensorStatBOTTOM;}
    void MoveMotor();
private:
    bool _bv(uint32_t iVal,uint8_t iBV){
        if(iVal & (1ULL<<iBV))
            return true;
        return false;
    }
    void writeConfigRegMotor(uint8_t bit,bool val);
    void communicate_to_can(uint8_t ipara,uint8_t *idata);
    bool pNewReport = 0;
    uint64_t pTimeRecovery = 0;
    uint8_t pdataConfig[10] = {};
    uint16_t pLowDelayPulse = 5000LL;
    uint16_t pHighDelayPulse = 15000LL;
    uint16_t pDefLow = 500LL,pDefHigh = 1500LL;
    int32_t pCurrentPosition = 0;
    int32_t pToGoPosition = 0;
    ServerCAN *ptrCAN;
    uint8_t pAddressMotor = 0;
    bool pSensorStatTOP = false;
    bool pSensorStatBOTTOM = false;
    bool pMotorMoving = false;
    bool pDSCFailure = false;
    bool pDSCnotConfig = false;
};
#endif