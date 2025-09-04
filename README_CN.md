# DFRobot_Tactile

===========================
* [English Version](./README.md)

触觉传感器是一种可以将物理接触、压力或力分布信息转换为可测量电信号的装置，常被视为机器人的“电子皮肤”。它通过感知作用在其表面上的压力的大小、位置分布甚至变化趋势，为设备提供关键的触觉反馈。

![产品效果图片]

## 产品链接（）

    SKU：

## 目录

  * [概述](#概述)
  * [库安装](#库安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [创作者](#创作者)

## 概述

- 支持4*8和6*6的ADC阵列
- 压力阈值、串口参数可配置
- 支持Modbus RTU通讯

## 库安装

要使用这个库, 首先下载库文件( https://github.com/DFRobot/DFRobot_Tactile )和依赖文件( https://github.com/DFRobot/DFRobot_RTU ), 将其粘贴到\Arduino\libraries目录中, 然后打开示例文件夹并在文件夹中运行演示。

## 方法

```C++
  /**
   * @fn begin
   * @brief 初始化传感器
   * @return int8_t 
   */
  int8_t begin(void);

/**
 * @fn setThld
 * @brief 设置传感器压力阈值
 * @param thld 传感器阀的阈值。默认值：200
 */
  void setThld(uint16_t thld);
  
  /**
   * @fn getAdcValue
   * @brief 获取ADC阵列数据值
   * @return sAdcDatas_t 
   * @n       result        result字段表示读取状态，0表示成功，default表示失败。
   * @n       adcval        adcval字段是传感器阵列的ADC数据的二维数组。
   */
  sAdcDatas_t getDatas(void);

  /**
   * @fn resetVal
   * @brief 重置传感器
   */
  void resetVal(void);

  /**
   * @fn setDevAddr
   * @brief 设置设备地址
   * @param addr
   */
  void setDevAddr(uint8_t addr);

  /**
   * @fn setBaudrate
   * @brief 设置串口配置参数
   * @param baud 
   * @n     BAUD_9600
   * @n     BAUD_57600
   * @n     BAUD_115200
   * @n     BAUD_921600
   */
  void setBaudrate(uint16_t baud, uint8_t stopBits = STOP_BIT_1, uint8_t parity = PARITY_NONE);

  /**
   * @fn getDeviceInfo
   * @brief 获取设备信息
   * @return sVersionInfo_t 
   * @n       VID           产品VID
   * @n       PID           产品PID
   * @n       version       产品版本
   */
  sVersionInfo_t getDeviceInfo(void);

```


## 兼容性

MCU                | Work Well    |   Work Wrong    | Untested    | Remarks
------------------ | :----------: | :-------------: | :---------: | :----:
Arduino Uno        |      √       |                 |             |
Arduino MEGA2560   |      √       |                 |             |
Arduino Leonardo   |      √       |                 |             |
FireBeetle-ESP8266 |      √       |                 |             |
FireBeetle-ESP32   |      √       |                 |             |
FireBeetle-M0      |      √       |                 |             |
Micro:bit          |              | nonsupport uart |             |

## 历史
- 2025-09-04 - 1.0.0 版本

## 创作者

Written by jiali(zhixinliu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
