#include "define.h"
#include "canserver.h"
#include "MotorClient.h"
#ifndef __MOTOR__CLASS__
#define __MOTOR__CLASS__
class virtualMotor : public MotorClient
{
public:
    virtualMotor(uint8_t add,ServerCAN *ptrcan) : MotorClient(add,ptrcan){
        pNewReportCount = cHystNewReport * (-5);
        pFisBusy = true;
    }
    void setToGo(int32_t itogo);
    void setReleativeToGo(int32_t itogo);
    
    void shutmotor();
    void GoHome();
    void lcdData(char *data);

    void RecoverError();


    void run();
    void Move(){MoveMotor();}

    bool isError() const {return (pFisFailued || pFisNotConfig);}
    bool isFailued() const {return pFisFailued;}
    bool isNotConfig() const {return pFisNotConfig;}
    bool isBusy() const {return (pFisBusy || isError());}
    bool isHome() const {return pFisHome;}
    bool PcDataAvailable();
private:
    uint32_t longtostr(char *data,int64_t value,int base = 10);
    uint32_t strcpstr(char *output,const char *copyfrom,int32_t size = 0);
    bool _bv(uint32_t iVal,uint8_t iBV){
        if(iVal & (1ULL<<iBV))
            return true;
        return false;
    }
    int32_t pNewReportCount = 0; 
    const int32_t cHystNewReport = 5;
    bool pFisBusy = true;
    bool pFisHome = false;
    bool pFisNotConfig = false;
    bool pFisFailued = false;
    bool pRecoverNeeded = false;
    bool pSendToPC = false;
};

#endif