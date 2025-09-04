/*!
 * @file DFRobot_Tactile.h
 * @brief This file demostrates the method for using tactile sensor library.
 * @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [jiali](zhixinliu@dfrobot.com)
 * @version  V1.0
 * @date  2025-08-06
 * @url https://github.com/DFRobot/DFRobot_Tactile
 */
#ifndef DFROBOT_TACTILE_H
#define DFROBOT_TACTILE_H

#include "Arduino.h"
#include "DFRobot_RTU.h"
#include "String.h"
#if (defined ARDUINO_AVR_UNO) && (defined ESP8266)
#include "SoftwareSerial.h"
#else
#include "HardwareSerial.h"
#endif

/**
 * @struct sAdcDatas_t
 * @brief ADC data structure
 */
typedef struct{
  uint8_t result;
  uint16_t adcval[6][8];
}sAdcDatas_t;

/**
 * @struct sVersionInfo_t
 * @brief Device version information structure
*/
typedef struct{
  uint16_t VID;
  uint16_t PID;
  uint16_t version;
}sVersionInfo_t;

/**
 * @enum eBaud_t
 * @brief Baud rate enumeration
 */
typedef enum{
  BAUD_9600 = 0x0003,
  BAUD_57600 = 0x0007,
  BAUD_115200 = 0x0008,
  BAUD_921600 = 0x0009,
}eBaud_t;
/**
 * @enum eStopBit_t
 * @brief Stop bit enumeration
 */
typedef enum{
  STOP_BIT_0_5 = 0x00,
  STOP_BIT_1 = 0x01,
  STOP_BIT_1_5 = 0x02,
  STOP_BIT_2 = 0x03,
}eStopBit_t;

/**
 * @enum eParity_t
 * @brief Parity enumeration
 */
typedef enum{
  PARITY_NONE = 0x00,
  PARITY_EVEN = 0x02,
  PARITY_ODD = 0x03,
}eParity_t;

/**
 * @enum eSampleRate_t
 * @brief Sample rate enumeration
 */
typedef enum{
  eSampleRate20Hz  = 0x0000,
  eSampleRate50Hz  = 0x0001,
}eSampleRate_t;

class DFRobot_Tactile:public DFRobot_RTU{
public:
  #define BROADCAST_ADDR           0x0000  ///< Broadcast address
  //Input Register
  #define INPUTREG                          0
  #define INPUTREG_VID                      0x0000    ///< Device VID
  #define INPUTREG_PID                      0x0001    ///< VID of the device, fixed to be 0x3343
  #define INPUTREG_VERSION                  0x0005    ///< Firmware version information
  #define INPUTREG_DEVSTATE                 0x0006    ///< Device state
  #define INPUTREG_GETDATAS                 0x0007    ///< Device ADC data.36*16 bytes
  #define INPUTREG_MODEL                    0x002B    ///< Device model information

  //Holding register  
  #define HOUTINGREG                        1
  #define HOUTINGREG_ADDR                   0X0002    ///< The gesture that can trigger interrupt
  #define HOUTINGREG_BAUDRATE               0X0003    ///< The baud rate of the serial port
  #define HOUTINGREG_VERIFY_AND_STOP        0X0004    ///< Reset the device
  #define HOUTINGREG_THLD                   0X0006    ///< Sensor valve value
  #define HOUTINGREG_RESET                  0X0007    ///< Restore the default value
  #define HOUTINGREG_SAMPLE_RATE            0X0008    ///< Sample rate of the sensor
  
  DFRobot_Tactile(uint8_t array,uint8_t addr, Stream *s,uint16_t sampleRate = eSampleRate20Hz );
  ~DFRobot_Tactile(){};

  /**
   * @fn begin
   * @brief Initialize the sensor.
   * @return int8_t 
   */
  int8_t begin(void);

/**
 * @fn setThld
 * @brief Set the Thld object.
 * @param thld The threshold value of the sensor valve. Default value: 200
 */
  void setThld(uint16_t thld);
  
  /**
   * @fn getAdcValue
   * @brief Get the Datas object.
   * @return sAdcDatas_t 
   * @n       result        The result field indicates the status of the reading, 0 indicates success, defult indicates failure.
   * @n       adcval        The ADC data of the sensor,the ordered two-dimensional array that can be accessed directly
   */
  sAdcDatas_t getDatas(void);

  /**
   * @fn resetVal
   * @brief Reset the device to the default value.
   */
  void resetVal(void);

  /**
   * @fn setDevAddr
   * @brief Set the device address.
   * @param addr The device address.
   */
  void setDevAddr(uint8_t addr);

  /**
   * @fn setBaudrate
   * @brief Set the Baudrate object
   * @param baud 
   * @n     BAUD_9600
   * @n     BAUD_57600
   * @n     BAUD_115200
   * @n     BAUD_921600
   * @param stopBits 
   * @n     STOP_BIT_0_5
   * @n     STOP_BIT_1
   * @n     STOP_BIT_1_5
   * @n     STOP_BIT_2
   * @param parity 
   * @n     PARITY_NONE
   * @n     PARITY_EVEN
   * @n     PARITY_ODD
   */
  void setBaudrate(uint16_t baud, uint8_t stopBits = STOP_BIT_1, uint8_t parity = PARITY_NONE);

  /**
   * @fn getDeviceInfo
   * @brief Get the device information object.
   * @return sVersionInfo_t 
   * @n       VID           The vendor ID of the device.
   * @n       PID           The product ID of the device.
   * @n       version       The version of the device.
   */
  sVersionInfo_t getDeviceInfo(void);

protected:

  int getAdcValue(uint8_t x, uint8_t y);
  uint16_t getPID(void);
  uint16_t getVID(void);
  uint16_t getVersion(void);
  uint16_t getModel(void);
  void setSampleRate(void);
  bool getDevState(void);
  uint8_t readReg(uint8_t reg, void *pBuf, uint8_t size, uint8_t regType);
  uint8_t writeReg(uint8_t reg, void *pBuf, size_t size);

  Stream    *_s = NULL;
  uint16_t   _addr;
  uint16_t   _sampleRate;

  uint16_t _array;
  uint16_t _arrayX;
  uint16_t _arrayY;
};
#endif
