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

本库使用到了modbus_tk, 使用本库前先检测树莓派是否成功导入modbus_tk, 若导入失败, 请通过以下命令安装modbus_tk库 python2: pip install modbus_tk python3: pip3 install modbus_tk

使用库, 首先下载库文件, 将其粘贴到指定的目录中, 然后打开Examples文件夹并在该文件夹中运行演示。


## 方法

```python
  def begin(self):
    '''!
      @brief 初始化传感器
      @return 初始化状态
      @retval True 初始化成功
      @retval False 初始化失败
    '''

  def set_thld(self, thld):
    '''!
      @brief 设置传感器的阈值
      @param thld 阈值
    '''

  def get_datas(self):
    '''!
      @brief 获取传感器的ADC数据
      @return ADC数据
      @retval result 获取数据成功
      @retval adcval 传感器阵列的ADC数据, 可以直接访问的二维数组
    '''

  def reset_val(self):
    '''!
      @brief 重置传感器
    '''

  def set_dev_addr(self,addr):
    '''!
      @brief 设置传感器的设备地址
      @param addr 设备地址
    '''

  def set_baudrate(self,baud):
    '''!
      @brief 设置传感器的波特率
      @param baud 波特率
    '''

  def get_device_info(self):
    '''!
      @brief 获取传感器的设备信息
      @return 设备信息
      @retval PID 产品ID
      @retval VID 厂商ID
      @retval Version 版本号
    '''
```

## Compatibility

* RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |     √     |            |          |         |
| RaspberryPi4 |           |            |     √    |         |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |     √     |            |          |         |
| Python3 |     √     |            |          |         |

## History

- 2022-09-04 - 1.0.0 版本

## Credits

Written by jiali(zhixinliu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))