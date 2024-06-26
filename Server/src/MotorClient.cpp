#include "MotorClient.h"

MotorClient::MotorClient(uint8_t add,ServerCAN *ptrcan)
{
    pAddressMotor = add;
    ptrCAN = ptrcan;
    if(pAddressMotor == C_DSC_BRODCAST_ADDRESS_CAN)
        return;
    if(pAddressMotor <= 0)
        pAddressMotor = 0;
    if(pAddressMotor >= 8)
        pAddressMotor = 100;
    pSensorStatTOP = false;
    setDiv(16);
    setDir(false);
    setEnableMotor(true);
    setEncoder_nm(5000LL);
    setPid(8,2,0);
    setEnablePID(true);
    stop();
    setReportInterval(80);
    setSensorBottomNormalStat(true);
    setSensorTOPNormalStat(true);
}
void MotorClient::setReportInterval(uint8_t iVal)
{
    pdataConfig[C_DSC_ARRAY_CONFIG_REG_POSITION_INTERVAl] = iVal;
}
void MotorClient::setDiv(uint8_t iDiv)
{
    switch (iDiv)
    {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
        pdataConfig[C_DSC_ARRAY_CONFIG_REG_DIV] = iDiv;
        break;
    default:
        break;
    }
}
void MotorClient::setPid(uint8_t kp,uint8_t ki,uint8_t kd)
{
    uint8_t ptmpBuff[10] = {};
    ptmpBuff[C_DSC_ARRAY_PID_REG_KP] = kp;
    ptmpBuff[C_DSC_ARRAY_PID_REG_KI] = ki;
    ptmpBuff[C_DSC_ARRAY_PID_REG_KD] = kd;
     communicate_to_can(C_DSC_OPCODE_REG_PID_CONFIG,ptmpBuff);
}
void MotorClient::writeConfigRegMotor(uint8_t bit,bool val)
{
    uint8_t *ptrData;
    ptrData = &pdataConfig[C_DSC_ARRAY_CONFIG_REG_CONF_1byte];
    if(bit > 7)
    {
        ptrData = &pdataConfig[C_DSC_ARRAY_CONFIG_REG_CONF_2byte];
        bit -= 8;
    }
    *ptrData &= (~(1<<bit)) & 0xFF;
    if(val)
        *ptrData |= (1<<bit) & 0xFF;
}
void MotorClient::setEnableMotor(bool iVal)
{
    writeConfigRegMotor(C_DSC_BIT_CONFIG_MOTOR_ENABLE,iVal);
}
void MotorClient::setDir(bool iVal)
{
    writeConfigRegMotor(C_DSC_BIT_CONFIG_MOTOR_DEFAULT_DIRECTION,iVal);
}
void MotorClient::setDirEncoder(bool iVal)
{
    writeConfigRegMotor(C_DSC_BIT_CONFIG_ENCODER_DEFAULT_DIRECTION,iVal);
}
void MotorClient::setEnablePID(bool iVal)
{
    writeConfigRegMotor(C_DSC_BIT_CONFIG_PID_ENABLE,iVal);
}
void MotorClient::setEnableEncoder(bool iVal)
{
    writeConfigRegMotor(C_DSC_BIT_CONFIG_ENCODER_HARDWARE,iVal);
}
void MotorClient::setSpeedUS(uint16_t iLow,uint16_t iHigh)
{
    uint8_t ptmpBuff[10] = {};
    if(iLow >= 15LL)
        pLowDelayPulse = iLow;

    if(iHigh >= 50LL)
        pHighDelayPulse = iHigh;
    ptmpBuff[C_DSC_ARRAY_SPEED_REG_HIGH_us_1byte] = pHighDelayPulse & 0xFF;
    ptmpBuff[C_DSC_ARRAY_SPEED_REG_HIGH_us_2byte] = (pHighDelayPulse>>8) & 0xFF;
    ptmpBuff[C_DSC_ARRAY_SPEED_REG_LOW_us_1byte] = pLowDelayPulse & 0xFF;
    ptmpBuff[C_DSC_ARRAY_SPEED_REG_LOW_us_2byte] = (pLowDelayPulse>>8) & 0xFF;
    communicate_to_can(C_DSC_OPCODE_REG_SPEED,ptmpBuff);

}
void MotorClient::setDefaultus(uint16_t iLow,uint16_t iHigh)
{
    if(iLow >= 15)
        pDefLow = iLow;
    if(iHigh >= 100)
        pDefHigh = iHigh;
    setSpeedUS(pDefLow,pDefHigh);
}
void MotorClient::setOtherMotorSensorStop(uint8_t iMotNo)
{
    pdataConfig[C_DSC_ARRAY_CONFIG_REG_FROM_OTHER_MOTOR_STATUS_READ] = iMotNo;
}
void MotorClient::setMotorPosition(int32_t itogo)
{
    if(getFlag_Failued())
        return;
    uint8_t ptmpBuff[10] = {};
    ptrCAN->int32_to_ptrint8(itogo,&ptmpBuff[C_DSC_ARRAY_TOGO_REG_index]);
    communicate_to_can(C_DSC_OPCODE_REG_TOGO,ptmpBuff);
}
void MotorClient::stop()
{
    uint8_t ptmpBuff[10] = {};
    ptmpBuff[C_DSC_ARRAY_StartStop] = 0;
    ptmpBuff[C_DSC_ARRAY_StartStop] |= (1<<C_DSC_BIT_STOP_MOVING);
    communicate_to_can(C_DSC_OPCODE_REG_START_STOP,ptmpBuff);
}
void MotorClient::MoveMotor()
{
    uint8_t ptmpBuff[10] = {};
    ptmpBuff[C_DSC_ARRAY_StartStop] = 0;
    ptmpBuff[C_DSC_ARRAY_StartStop] |= (1<<C_DSC_BIT_START_MOVING);
    communicate_to_can(C_DSC_OPCODE_REG_START_STOP,ptmpBuff);
}

void MotorClient::TopSensorStat(bool stat)
{
    writeConfigRegMotor(C_DSC_BIT_CONFIG_SENSOR_ENABLE_TOP,stat);
    synchConfig();
}
void MotorClient::LockStat(bool Lock)
{
    writeConfigRegMotor(C_DSC_BIT_CONFIG_LOCK_MOTOR,Lock);
    synchConfig();
}
void MotorClient::setSensorBottomNormalStat(bool iVal)
{
    writeConfigRegMotor(C_DSC_BIT_CONFIG_BOTTOM_SENSOR_DEFAULT,iVal);
    writeConfigRegMotor(C_DSC_BIT_CONFIG_SENSOR_ENABLE_BOTTOM,true);
}
void MotorClient::setSensorTOPNormalStat(bool iVal)
{
    writeConfigRegMotor(C_DSC_BIT_CONFIG_TOP_SENSOR_DEFAULT,iVal);
    writeConfigRegMotor(C_DSC_BIT_CONFIG_SENSOR_ENABLE_TOP,true);
}
void MotorClient::synchConfig()
{
    communicate_to_can(C_DSC_OPCODE_REG_CONFIG,pdataConfig);
}
void MotorClient::readCAN(int32_t iLocation,int16_t statusData,uint8_t iMorNO)
{
    bool iBottomSensorStat,iTopSensorStat,imotorMoving,bFailure,bNOTConfigured;
    iBottomSensorStat = _bv(statusData,C_DSC_BIT_STATUS_SENSOR_BOTTOM_STATUS);
    iTopSensorStat = _bv(statusData,C_DSC_BIT_STATUS_SENSOR_TOP_STATUS);
    imotorMoving = _bv(statusData,C_DSC_BIT_STATUS_MOTOR_MOVING);
    bFailure = _bv(statusData,C_DSC_BIT_STATUS_FAILURE_HAPPEN);
    bNOTConfigured = _bv(statusData,C_DSC_BIT_STATUS_Board_NOT_Configured);

    if(iMorNO == pdataConfig[C_DSC_ARRAY_CONFIG_REG_FROM_OTHER_MOTOR_STATUS_READ])
    {
        if(iTopSensorStat != pSensorStatTOP)
        {
            pSensorStatTOP = iTopSensorStat;
            if(pSensorStatTOP == true)
                stop();
        }
    }
    if(iMorNO != pAddressMotor)
        return;
    pNewReport = true;
    pCurrentPosition = iLocation;
    pMotorMoving = imotorMoving;
    if(pSensorStatBOTTOM != iBottomSensorStat)
    {
        pSensorStatBOTTOM = iBottomSensorStat;
        if(pSensorStatBOTTOM == true)
        {
            pToGoPosition = 0;
            pCurrentPosition = 0;
        }
    }
    if(pDSCFailure != bFailure)
    {
        pDSCFailure = bFailure;
    }
    if(pDSCnotConfig != bNOTConfigured)
    {
        pDSCnotConfig = bNOTConfigured;
    }
}

void MotorClient::communicate_to_can(uint8_t ipara,uint8_t *idata)
{
    ptrCAN->send_data(pAddressMotor,ipara,idata);
}
void MotorClient::setEncoder_nm(uint16_t inm)
{
    if(inm <= 50)
        inm = 50;
    pdataConfig[C_DSC_ARRAY_CONFIG_REG_ENCODER_RES_1byte] = inm & 0xFF;
    pdataConfig[C_DSC_ARRAY_CONFIG_REG_ENCODER_RES_2byte] = (inm>>8) & 0xFF;
}

void MotorClient::setFailureRecover()
{
    uint8_t ptmpBuff[10] = {};
    communicate_to_can(C_DSC_OPCODE_REG_Failure_Recover,ptmpBuff);
}