#include "VirtualMotor.h"
void virtualMotor::RecoverError(){
    setEnableMotor(true);
    synchConfig();
    setFailureRecover();
    pNewReportCount = cHystNewReport * (-4);
    pFisBusy = true;
    pFisNotConfig = false;
    pFisFailued = false;
    pRecoverNeeded = false;
    pSendToPC = true;
}
bool virtualMotor::PcDataAvailable(){
    if(!pSendToPC)
        return false;
    pSendToPC = false;
    return true;
}
void virtualMotor::setToGo(int32_t itogo)
{
    if(isFailued())
        return;
    pNewReportCount = 0;
    pFisBusy = true;
    pSendToPC = true;
    setMotorPosition(itogo);
}
void virtualMotor::setReleativeToGo(int32_t itogo)
{
    setToGo(itogo + getLocation());
    //ptrCAN->int32_to_ptrint8(itogo,&ptmpBuff[C_DSC_ARRAY_Releative_TOGO_REG_index]);
    //ptrCAN->send_data(pAddressMotor,C_DSC_OPCODE_REG_Releative_TOGO,ptmpBuff);
}

void virtualMotor::shutmotor(){
    stop();
    setEnableMotor(false);
    synchConfig();
    pNewReportCount = 0;
    pFisBusy = true;
    pRecoverNeeded = true;
    pSendToPC = true;
}
void virtualMotor::run()
{
    if(getFlag_NewReport()){
        pNewReportCount++;
    }
    if(pNewReportCount > cHystNewReport){
        pNewReportCount = 0;
        pFisHome = getFlag_Home();
        pFisBusy = getFlag_Busy();
        pFisNotConfig = getFlag_NotConfig();
        pFisFailued = getFlag_Failued();
        pSendToPC = true;
    }
}
void virtualMotor::lcdData(char *idata){
    uint8_t i = 0;
    if((pFisFailued) || (pFisNotConfig) || (pRecoverNeeded)){
        i = i + strcpstr(&idata[i],"M");
        i = i + longtostr(&idata[i],getMotNo(),10);
        i = i + strcpstr(&idata[i]," Failed NC");
        i = i + longtostr(&idata[i],pFisNotConfig,10);
        i = i + strcpstr(&idata[i]," F");
        i = i + longtostr(&idata[i],pFisFailued,10);
        i = i + strcpstr(&idata[i]," R");
        i = i + longtostr(&idata[i],pRecoverNeeded,10);
        i = i + strcpstr(&idata[i],"                    ",20 - i);
        return;
    }
    i = i + longtostr(&idata[i],isBusy(),10);
    i = i + strcpstr(&idata[i]," M");
    i = i + longtostr(&idata[i],getMotNo(),10);
    i = i + strcpstr(&idata[i],":");
    if(getLocation() >= 100000LL){
        i = i + longtostr(&idata[i],getLocation()/100000LL,10);
        i = i + strcpstr(&idata[i],".");
        i = i + longtostr(&idata[i],(getLocation()%100000LL)/1000,10);
        i = i + strcpstr(&idata[i],"mm");
    }
    else if(getLocation() >= 100LL){
        i = i + longtostr(&idata[i],getLocation()/100LL,10);
        i = i + strcpstr(&idata[i],".");
        i = i + longtostr(&idata[i],(getLocation()%100LL),10);
        i = i + strcpstr(&idata[i],"um");
    }
    else{
        i = i + longtostr(&idata[i],getLocation(),10);
    }
    i = i + strcpstr(&idata[i],"                    ",20 - i);
}
// if size equal to zero copy untill it reach to *copyfrom == \0
uint32_t virtualMotor::strcpstr(char *output,const char *copyfrom,int32_t size){
    //put maximum limit of 255 to copy
    uint32_t ret = 0;
    if(size <= 0)
        size = 255;
    while((size > 0) && (*copyfrom != '\0')){
        *output++ = *copyfrom++;
        size--;
        ret++;
    }
    return ret;
}
uint32_t virtualMotor::longtostr(char *data,int64_t value,int base){
		uint64_t tmp;
		int8_t i = 0;
		char *ptrbase = data;
		char basedig[] = "0123456789abcdef";
		uint32_t ret = 0;
		if((base < 2) || (base > 16)){*data =  '\0';return 0;}
		if(value < 0){
				value *= -1;
				*ptrbase = '-';
				ptrbase++;
				ret++;
		}
		tmp = value;
		do{
				tmp /= base;
				ptrbase++;
		}while(tmp > 0);
		*ptrbase = '\0';
		tmp = value;
		do{
				ptrbase--;
				ret++;
				*ptrbase = basedig[tmp % base];
				tmp /= base;
		}while(tmp > 0);
		return ret;
}
void virtualMotor::GoHome()
{
    if(getFlag_Home())
        return;
    setMotorPosition(-500000000LL);
    MoveMotor();
}