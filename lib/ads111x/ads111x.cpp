#include "ads111x.h"
ADS1115::ADS1115(uint8_t i2cAddr)
{
    confOS = 0b1;
    confMUX = A0GND;
    confPGA = FSR2048;
    confMODE = 0x1;
    confDR = SPS128;
    confCompMode = 0x0;
    confCompPol = 0x0;
    confCompLat = 0x0;
    confCompQue = 0x3;
    i_i2cAddr = i2cAddr;

    calcInternals();
}
ADS1115::~ADS1115()
{
}
void ADS1115::begin()
{
    Wire.begin();
};
bool ADS1115::conneted()
{

    Wire.beginTransmission(i_i2cAddr);
    uint8_t error = Wire.endTransmission();
    if (error == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
void ADS1115::setGain(adsGain_t gain)
{
    confPGA = gain;
    calcInternals();
}
float ADS1115::readVoltage(adsChan_t channel,float R1,float R2)
{

    confMUX = channel;
    uint8_t config1 = (confOS << 7) | (confMUX << 4) | (confPGA << 1) | confMODE;
    uint8_t config2 = (confDR << 5) | (confCompMode << 4) | (confCompPol << 3) | (confCompLat << 2) | confCompQue;

    Wire.beginTransmission(i_i2cAddr);
    Wire.write(0x1);
    Wire.write(config1);
    Wire.write(config2);
    Wire.endTransmission();
    delay(delayTime);
    Wire.beginTransmission(i_i2cAddr);
    Wire.write(0x0);
    Wire.endTransmission();
    Wire.requestFrom(i_i2cAddr, (uint8_t) 2);
    int16_t value = ((Wire.read() << 8) | Wire.read());
    return (float)value*i_lsb*(R1+R2)/R2;
}
float ADS1115::readVoltage(adsChan_t channel)
{
    return readVoltage(channel,0,1);
}
void ADS1115::setDataRate(adsSPS_t dataRate)
{
    confDR = dataRate;
    calcInternals();
}

void ADS1115::calcInternals()
{
    switch (confDR)
    {
    case SPS8:
        delayTime = 126;
        break;
    case SPS16:
        delayTime = 64;
        break;
    case SPS32:
        delayTime = 33;
        break;
    case SPS64:
        delayTime = 17;
        break;
    case SPS128:
        delayTime = 9;
        break;
    case SPS250:
        delayTime = 5;
        break;
    case SPS475:
        delayTime = 4;
        break;
    case SPS860:
        delayTime = 3;
        break;
    default:
        delayTime = 200;
        break;
    }
    switch (confPGA)
    {
    case FSR6144:
        i_lsb = 187.5e-6;
        break;
    case FSR4096:
        i_lsb = 125e-6;
        break;
    case FSR2048:
        i_lsb = 62.5e-6;
        break;
    case FSR1024:
        i_lsb = 31.25e-6;
        break;
    case FSR0512:
        i_lsb = 15.625e-6;
        break;
    case FSR0256:
        i_lsb = 7.8125e-6;
        break;

    default:
        i_lsb = 187.5e-6;
        break;
    }
}
