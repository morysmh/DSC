#include "clientmotor.h"

virtualMotor::virtualMotor(uint8_t add,CanCotroll *ptrcan)
{
    pAddressMotor = add;
    ptr_can = ptrcan;
    if(pAddressMotor == C_DSC_BRODCAST_ADDRESS_CAN)
        return;
    if(pAddressMotor <= 0)
        pAddressMotor = 0;
    if(pAddressMotor >= 8)
        pAddressMotor = 100;
    setDiv(16);
    setDir(false);
    setEnableMotor(true);
    setEncoder_nm(5000LL);
    setPid(8,2,0);
    setEnablePID(true);
    stop();
}
void virtualMotor::setDiv(int32_t iDiv)
{
    switch (iDiv)
    {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
        communicate_to_can(C_DSC_STEP_MOTOR_SET_DIV,iDiv);
        break;
    default:
        break;
    }
}
void virtualMotor::setPid(int32_t kp,int32_t ki,int32_t kd)
{
    communicate_to_can(C_DSC_STEP_MOTOR_SET_KP,kp);
    communicate_to_can(C_DSC_STEP_MOTOR_SET_KI,ki);
    communicate_to_can(C_DSC_STEP_MOTOR_SET_KD,kd);
}
void virtualMotor::setEnableMotor(bool iVal)
{
    pMotorEnable = iVal;
    if(pMotorEnable == true)
    {
        communicate_to_can(C_DSC_STEP_MOTOR_ENABLE,1);
        return;
    }
    communicate_to_can(C_DSC_STEP_MOTOR_DISABLE,1);
}
void virtualMotor::setDir(bool iVal)
{
    communicate_to_can(C_DSC_STEP_MOTOR_SET_Default_Direction,iVal);
}
void virtualMotor::setDirEncoder(bool iVal)
{
    communicate_to_can(C_DSC_ENCODER_DIRECTION,iVal);
}
void virtualMotor::setEnablePID(bool iVal)
{
    if(iVal == true)
    {
        communicate_to_can(C_DSC_STEP_MOTOR_PID_ENABLE,1);
        return;
    }
    communicate_to_can(C_DSC_STEP_MOTOR_PID_DISABLE,1);
}
void virtualMotor::setEnableEncoder(bool iVal)
{
    if(iVal == true)
    {
        communicate_to_can(C_DSC_ENCODER_HARDWARE_ENABLE,1);
        return;
    }
    communicate_to_can(C_DSC_ENCODER_HARDWARE_DISABLE,1);
}
void virtualMotor::setSpeedUS(int32_t iLow,int32_t iHigh)
{
    if(iLow <= 10LL)
        iLow = 10LL;
    if(iLow >= 65530LL) //Limit us to 16 bit variable
        iLow = 65530LL;

    if(iHigh <= 10LL)
        iHigh = 10LL;
    if(iHigh >= 65530LL) //Limit us to 16 bit
        iHigh = 65530LL;
    if(pLowDelayPulse != iLow)
        communicate_to_can(C_DSC_STEP_MOTOR_SET_low_us,iLow);
    if(pHighDelayPulse != iHigh)
        communicate_to_can(C_DSC_STEP_MOTOR_SET_max_us,iHigh);
    pLowDelayPulse = iLow;
    pHighDelayPulse = iHigh;

}
void virtualMotor::setDefaultLow(int32_t iVal)
{
    if(iVal <= 10)
        return;
    if(iVal >= 65530)
        iVal = 65530;
    pDefLow = iVal;
}
void virtualMotor::setOtherMotorSensorStop(int8_t iMotNo)
{
    if(iMotNo <= 0)
        return;
    if(iMotNo > 8)
        return;
    pOtherMotorSensorStop = iMotNo;
}
void virtualMotor::setToGo(int32_t itogo)
{
    pToGoPosition = itogo;
    pToGoPosition /= pEncoder_nm;
    pToGoPosition *= pEncoder_nm;
    communicate_to_can(C_DSC_STEP_MOTOR_TOGO_ON_COMMAND,pToGoPosition);
    communicate_to_can(C_DSC_STEP_MOTOR_START_MOVING,1);
}
void virtualMotor::stop()
{
    communicate_to_can(C_DSC_STEP_MOTOR_STOP,1);
}
void virtualMotor::setSensorBottomNormalStat(bool i_val)
{
    communicate_to_can(C_DSC_SENSOR_BOTTOM_DEFAULT_VALUE,i_val);
}
void virtualMotor::setSensorTOPNormalStat(bool i_val)
{
    communicate_to_can(C_DSC_SENSOR_TOP_DEFAULT_VALUE,i_val);
}
bool virtualMotor::isBusy()
{
    if(pToGoPosition == pCurrentPosition)
        return false;
    return true;
}
bool virtualMotor::isHome()
{
    if((pBottomSensorStat == true) && (pCurrentPosition == 0))
        return true;
    return false;
}
void virtualMotor::readCAN(int8_t motNO,int8_t iPara,int32_t iVal)
{
    if(motNO == pOtherMotorSensorStop)
    {
        if((iPara == C_DSC_SENSOR_TOP_READ_STATUS) && (iVal == 1))
        {
            miliReadPos = 1;
            pTOPSensorTrig = true;
            stop();
            readPos();
        }
    }
    if(motNO != pAddressMotor)
        return;
    switch (iPara)
    {
    case C_DSC_SENSOR_BOTTOM_READ_STATUS:
        pBottomSensorStat = !!iVal;
        if(pBottomSensorStat_LastStat == pBottomSensorStat)
            break;
        pBottomSensorStat_LastStat = pBottomSensorStat;
        if(pBottomSensorStat_LastStat == 0)
            break;
        pToGoPosition = 0;
        pCurrentPosition = 0;
        break;

    case C_DSC_ENCODER_LOCATION:
        pCurrentPosition = iVal;
        if(pTOPSensorTrig == true)
        {
            pTOPSensorTrig = false;
            pToGoPosition = iVal;
        }
        break;
    
    default:
        break;
    }
}

void virtualMotor::retransmit_data()
{

}
void virtualMotor::communicate_to_can(int8_t ipara,int32_t ival)
{
    ptr_can->send_data(ipara,ival,pAddressMotor);
}
void virtualMotor::run()
{
    readPos();
    readSensor();
}
void virtualMotor::readPos()
{
    if (miliReadPos > time_us_64())
        return;
    miliReadPos = time_us_64() + cIntervalReadPos;
    if(pToGoPosition == pCurrentPosition)
        miliReadPos = time_us_64() + (cIntervalReadPos * 10);
    communicate_to_can(C_DSC_ENCODER_LOCATION,1);
}
void virtualMotor::readSensor()
{
    if (miliSensorRead > time_us_64())
        return;
    miliSensorRead = time_us_64() + cIntervalReadPos;
    if(pToGoPosition >= 0)
        miliSensorRead = time_us_64() + (cIntervalReadPos * 10);
    communicate_to_can(C_DSC_SENSOR_BOTTOM_READ_STATUS,1);
}

void virtualMotor::setEncoder_nm(int32_t inm)
{
    if(inm <= 0)
        return;
    if(inm == pEncoder_nm)
        return;
    pEncoder_nm = inm;
    communicate_to_can(C_DSC_ENCODER_RESOLATION,pEncoder_nm);
}

bool virtualMotor::is_near()
{
    int32_t tmp = pCurrentPosition - pToGoPosition;
    if(tmp < 0)
        tmp *= -1;
    if(tmp <= (2 * pEncoder_nm))
        return true;
    return false;
}
void virtualMotor::GoHome()
{
    if(pBottomSensorStat_LastStat == true)
        return;
    setToGo(-500000000LL);
}