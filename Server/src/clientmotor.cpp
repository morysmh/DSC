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
    setEncoder_nm(500LL);
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
        ptr_can->send_data(C_DSC_STEP_MOTOR_SET_DIV,iDiv,pAddressMotor);
        break;
    default:
        break;
    }
}
void virtualMotor::setPid(int32_t kp,int32_t ki,int32_t kd)
{
    ptr_can->send_data(C_DSC_STEP_MOTOR_SET_KP,kp,pAddressMotor);
    ptr_can->send_data(C_DSC_STEP_MOTOR_SET_KI,ki,pAddressMotor);
    ptr_can->send_data(C_DSC_STEP_MOTOR_SET_KD,kd,pAddressMotor);
}
void virtualMotor::setEnableMotor(bool iVal)
{
    pMotorEnable = iVal;
    if(pMotorEnable == true)
    {
        ptr_can->send_data(C_DSC_STEP_MOTOR_ENABLE,1,pAddressMotor);
        return;
    }
    ptr_can->send_data(C_DSC_STEP_MOTOR_DISABLE,1,pAddressMotor);
}
void virtualMotor::setDir(bool iVal)
{
    ptr_can->send_data(C_DSC_STEP_MOTOR_SET_Default_Direction,iVal,pAddressMotor);
}
void virtualMotor::setDirEncoder(bool iVal)
{
    ptr_can->send_data(C_DSC_ENCODER_DIRECTION,iVal,pAddressMotor);
}
void virtualMotor::setEnablePID(bool iVal)
{
    if(iVal == true)
    {
        ptr_can->send_data(C_DSC_STEP_MOTOR_PID_ENABLE,1,pAddressMotor);
        return;
    }
    ptr_can->send_data(C_DSC_STEP_MOTOR_PID_DISABLE,1,pAddressMotor);
}
void virtualMotor::setEnableEncoder(bool iVal)
{
    if(iVal == true)
    {
        ptr_can->send_data(C_DSC_ENCODER_HARDWARE_ENABLE,1,pAddressMotor);
        return;
    }
    ptr_can->send_data(C_DSC_ENCODER_HARDWARE_DISABLE,1,pAddressMotor);
}
void virtualMotor::setSpeedUS(int32_t iLow,int32_t iHigh)
{
    pLowDelayPulse = iLow;
    pHighDelayPulse = iHigh;
    ptr_can->send_data(C_DSC_STEP_MOTOR_SET_max_us,pLowDelayPulse,pAddressMotor);
    ptr_can->send_data(C_DSC_STEP_MOTOR_SET_low_us,pHighDelayPulse,pAddressMotor);

}
void virtualMotor::setDefaultLow(int32_t iVal)
{
    if(iVal <= 10)
        return;
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
    ptr_can->send_data(C_DSC_STEP_MOTOR_SET_TOGO_Location,pToGoPosition,pAddressMotor);
}
void virtualMotor::stop()
{
    ptr_can->send_data(C_DSC_STEP_MOTOR_STOP,1,pAddressMotor);
}
void virtualMotor::setSensorBottomNormalStat(bool i_val)
{
    ptr_can->send_data(C_DSC_SENSOR_BOTTOM_DEFAULT_VALUE,i_val,pAddressMotor);
}
void virtualMotor::setSensorTOPNormalStat(bool i_val)
{
    ptr_can->send_data(C_DSC_SENSOR_TOP_DEFAULT_VALUE,i_val,pAddressMotor);
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
        if(iVal == 0)
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
    ptr_can->send_data(C_DSC_ENCODER_LOCATION,1,pAddressMotor);
}
void virtualMotor::readSensor()
{
    if (miliSensorRead > time_us_64())
        return;
    miliSensorRead = time_us_64() + cIntervalReadPos;
    if(pToGoPosition == 0)
        miliSensorRead = time_us_64() + (cIntervalReadPos * 10);
    ptr_can->send_data(C_DSC_SENSOR_BOTTOM_READ_STATUS,1,pAddressMotor);
}

void virtualMotor::setEncoder_nm(int32_t inm)
{
    if(inm <= 0)
        return;
    if(inm == pEncoder_nm)
        return;
    pEncoder_nm = inm;
    ptr_can->send_data(C_DSC_ENCODER_RESOLATION,pEncoder_nm,pAddressMotor);
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
    setToGo(-500000000LL);
}