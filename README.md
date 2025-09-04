# DFRobot_Tactile

===========================

* [中文版](./README_CN.md)

The tactile sensor is a device that can convert physical contact, pressure or force distribution information into measurable electrical signals, and is often regarded as the "electronic skin" of robots. It provides crucial tactile feedback to the equipment by sensing the magnitude, location distribution and even the changing trend of the pressure acting on its surface.

![product image]

## Product Link ()
    SKU: 

## Table of Contents

  * [Summary](#summary)
  * [Installation](#installation)
  * [Methods](#methods)
  * [Compatibility](#compatibility)
  * [History](#history)
  * [Credits](#credits)

## Summary

* Supports 4*8 and 6*6 arrays<br/>
* The pressure threshold and serial port parameters can be configured<br/>
* Support Modbus RTU communication protocol

## Installation

Download the library file (https://github.com/DFRobot/DFRobot_Tactile) and its dependencies (https://github.com/DFRobot/DFRobot_RTU) before use, paste them into the \Arduino\libraries directory, then open the sample folder and run the demo in the folder.

## Methods

```C++

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
   * @n       adcval        The adcval field is an array of the ADC data of the sensor array.
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
```

## Compatibility

MCU                | Work Well    |   Work Wrong    | Untested    | Remarks
------------------ | :----------: | :-------------: | :---------: | :----:
Arduino Uno        |      √       |                 |             |
Arduino MEGA2560   |      √       |                 |             |
Arduino Leonardo   |      √       |                 |             |
FireBeetle-ESP8266 |      √       |                 |             |
FireBeetle-ESP32   |      √       |                 |             |
FireBeetle-M0      |      √       |                 |             |
Micro:bit          |              | nonsupport uart |             |

## History

- 2022-09-04 - Version 1.0.0 released.

## Credits

Written by jiali(zhixinliu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
