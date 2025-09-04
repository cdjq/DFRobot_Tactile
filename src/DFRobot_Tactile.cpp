/*!
 * @file DFRobot_Tactile.cpp
 * @brief This is the method implementation file of the tactile sensor
 * @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [jiali](zhixinliu@dfrobot.com)
 * @version  V0.1
 * @date  2025-08-06
 * @url https://github.com/DFRobot/DFRobot_Tactile
 */

#include "DFRobot_Tactile.h"

DFRobot_Tactile::DFRobot_Tactile(uint8_t array,uint8_t addr, Stream *s,uint16_t sampleRate):DFRobot_RTU(s)
{
  _s = s;
  _addr = addr;

  _array = array;
  _sampleRate = sampleRate;
  if(array == 32){
  _arrayX = 8;
  _arrayY = 4;
  }else{
  _arrayX = 6;
  _arrayY = 6;
  }
}

int8_t DFRobot_Tactile::begin(void)
{
  delay(500);
  if(_addr < 0x0020 && _addr > 0x0000){
    if(getDevState() == false){
      return -1;
    }else{
      setSampleRate();
      return 0;
    }
  }
  else{
    return -1;
  }
}

void DFRobot_Tactile::setThld(uint16_t thld)
{
  uint8_t _sendBuf[2] = {0};
  _sendBuf[0] =(thld >> 0) & 0xFF;
  _sendBuf[1] =(thld >> 8)& 0xFF;

  writeReg(HOUTINGREG_THLD, _sendBuf, 2);
  delay(50);
}

sAdcDatas_t DFRobot_Tactile::getDatas(void)
{
  sAdcDatas_t adcDatas;
  uint8_t _recvBuf[36*2];
  uint16_t _ret= 0;
  uint8_t tempX, tempY;

  _ret = readReg(INPUTREG_GETDATAS, _recvBuf, _array * 2,INPUTREG);

  for(uint8_t i = _arrayY; i > 0 ; i--){
    tempY = _arrayY - i;
    for(uint8_t j = 0; j < _arrayX; j++){
        tempX =((i - 1) * _arrayX  + j);
        adcDatas.adcval[tempY][j] =_recvBuf[tempX * 2]<<8 | _recvBuf[tempX * 2 + 1];
    }
  }
  adcDatas.result = _ret;
  return adcDatas;
  }

sVersionInfo_t DFRobot_Tactile::getDeviceInfo(void)
{
  sVersionInfo_t versionInfo;

  versionInfo.VID = getVID();
  versionInfo.PID = getPID();
  versionInfo.version = getVersion();

  return versionInfo;
}

uint16_t DFRobot_Tactile::getVID(void)
{
  uint16_t VID = 0;
  uint8_t _recvBuf[10];

  if(readReg(INPUTREG_VID, _recvBuf, 2,INPUTREG) == 0xff){
    //Serial.println("Read PID error.");
    return 0;
  }
  VID = _recvBuf[0]<<8 | _recvBuf[1];
  return VID;
}

uint16_t DFRobot_Tactile::getPID(void)
{
  uint16_t PID = 0;
  uint8_t _recvBuf[10];
  if(readReg(INPUTREG_PID, _recvBuf, 2,INPUTREG) == 0xff){
    //Serial.println("Read PID error.");
    return 0;
  }
  PID = _recvBuf[0]<<8 | _recvBuf[1];
  return PID;
}

uint16_t DFRobot_Tactile::getVersion(void)
{
  uint16_t version = 0;
  uint8_t _recvBuf[10];
  if(readReg(INPUTREG_VERSION, _recvBuf, 2,INPUTREG) == 0xff){
    //Serial.println("Read version error.");
    return 0;
  }
  version = _recvBuf[0]<<8 | _recvBuf[1];
  return version;
}

int DFRobot_Tactile::getAdcValue(uint8_t x, uint8_t y)
{
  uint16_t adcValue = 0;
  uint8_t _recvBuf[10];

  if(x > _arrayX || y > _arrayY){
    // Serial.println("Invalid ADC coordinates.");
    return -1;
  }
  uint16_t instructionAddr = y*_arrayX+x;
  if(readReg(INPUTREG_GETDATAS + instructionAddr, _recvBuf, 2,INPUTREG) == 0xff){
    // Serial.println("Read ADC value error.");
    return -1;
  }
  adcValue = _recvBuf[0]<< 8 | _recvBuf[1];

  return adcValue;
}

void DFRobot_Tactile::resetVal(void)
{
  uint8_t _sendBuf[2] = {0};
  _sendBuf[0] = 0x00;
  _sendBuf[1] = 0x00;
  writeReg(HOUTINGREG_RESET, _sendBuf, 2);
  delay(50);
}

void DFRobot_Tactile::setDevAddr(uint8_t addr)
{
  uint8_t _sendBuf[2] = {0};
  _sendBuf[0] = addr;
  _sendBuf[1] = 0x00;
  writeReg(HOUTINGREG_ADDR, _sendBuf, 2);
  _addr = addr;
  delay(50);
}

void DFRobot_Tactile::setBaudrate(uint16_t baud,uint8_t stopBits, uint8_t parity)
{
  uint8_t _sendBuf[4] = {0};

  _sendBuf[0] = baud & 0xff;
  _sendBuf[1] = baud >>8 & 0xff;
  _sendBuf[2] = stopBits & 0xff;
  _sendBuf[3] = parity & 0xff;

  writeReg(HOUTINGREG_BAUDRATE, _sendBuf, 4);
  delay(50);
}

uint8_t DFRobot_Tactile::readReg(uint8_t reg, void *pBuf, uint8_t size, uint8_t regType)
{
  uint8_t* _pBuf = (uint8_t*)pBuf;
  if(pBuf == NULL){
    return 0;
  }
  if(regType == INPUTREG){
    return readInputRegister(_addr, reg, _pBuf, size);
  }else{
    return readHoldingRegister(_addr, reg, _pBuf, size);
  }
}

uint8_t DFRobot_Tactile::writeReg(uint8_t reg, void *pBuf, size_t size)
{
  uint8_t *_pBuf = (uint8_t*)pBuf;
  uint8_t ret = 0;
    ret = writeHoldingRegister(_addr, reg, _pBuf, size);
  
  return ret;
}

bool DFRobot_Tactile::getDevState(void)
{
  uint8_t _recvBuf[10];
  uint16_t ret = 0;
  if(readReg(INPUTREG_DEVSTATE, _recvBuf, 2,INPUTREG) == 0xff){
    //Serial.println("Read version error.");
    return false;
  }
  ret = _recvBuf[0]<<8 | _recvBuf[1];
  if(ret == 0x0001){
    return true;
  }else{
    return false;
  }
}

uint16_t DFRobot_Tactile::getModel(void)
{
  uint16_t model = 0;
  uint8_t _recvBuf[10];
  if(readReg(INPUTREG_MODEL, _recvBuf, 2,INPUTREG) == 0xff){
    return 0;
  }
  model = _recvBuf[0]<<8 | _recvBuf[1];
  return model;
}
  
void DFRobot_Tactile::setSampleRate(void)
{
  uint8_t _sendBuf[2] = {0};
  _sendBuf[0] = _sampleRate & 0xff;
  _sendBuf[1] = _sampleRate >>8 & 0xff;
  writeReg(HOUTINGREG_SAMPLE_RATE, _sendBuf, 2);
  delay(50);
}
