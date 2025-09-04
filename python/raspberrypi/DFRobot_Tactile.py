'''!
  @file DFRobot_Tactile.py
  @brief DFRobot_Tactile Class infrastructure, implementation of underlying methods
  @copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license The MIT License (MIT)
  @author [jiaLi](zhixinliu@dfrobot.com)
  @version V1.0
  @date 2025-08-06
  @url https://github.com/DFRobot/DFRobot_AI10
'''
# -*- coding: utf-8 -*

import serial
import time

import os
import math
import RPi.GPIO as GPIO

import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

BROADCAST_ADDR          = 0x0000  #Broadcast address
#Input Register
INPUTREG_VID                     = 0x0000    #Device VID
INPUTREG_PID                     = 0x0001    #VID of the device, fixed to be 0x3343
INPUTREG_VERSION                 = 0x0005    #Firmware version information
INPUTREG_DEVSTATE                = 0x0006    #Device state
INPUTREG_GETDATAS                = 0x0007    #Device ADC data.36*16 bytes
INPUTREG_MODEL                  = 0x002B    #Device model

#Holding register  
HOUTINGREG_ADDR                  = 0X0002    #The gesture that can trigger interrupt
HOUTINGREG_BAUDRATE              = 0X0003    #The baud rate of the serial port
HOUTINGREG_VERIFY_AND_STOP       = 0X0004    #Reset the device
HOUTINGREG_THLD                  = 0X0006    #Sensor valve value
HOUTINGREG_RESET                 = 0X0007    #Restore the default value
HOUTINGREG_SAMPLE_RATE           = 0X0008    #Set the sample rate

class adc_data:
  def __init__(self):
    self.result = 0
    self.adcval = [[0 for _ in range(8)] for _ in range(6)]
  '''
    @brief Get the ADC data of the sensor
    @return ADC data
    @retval    result    The result of get data from the sensor
    @retval    adcval    The ADC data of the sensor,the ordered two-dimensional array that can be accessed directly
  '''

class version_info:
  def __init__(self):
    self.VID = 0
    self.PID = 0
    self.version = 0


class DFRobot_Tactile():
  '''!
    @brief A class to interact with the DFRobot Tactile Sensor
    @details This class provides methods to communicate with the tactile sensor via Modbus RTU protocol
    @n It supports reading ADC values, setting thresholds, configuring device parameters, and more
  '''
  BAUD_9600 = 0x0003
  BAUD_57600 = 0x0007
  BAUD_115200 = 0x0008
  BAUD_921600 = 0x0009

  STOP_BIT_0_5 = 0x00
  STOP_BIT_1 = 0x01
  STOP_BIT_1_5 = 0x02
  STOP_BIT_2 = 0x03

  PARITY_NONE = 0x00
  PARITY_EVEN = 0x02
  PARITY_ODD = 0x03

  SAMPLE_RATE_20HZ = 0
  SAMPLE_RATE_50HZ = 1

  def __init__(self,addr,array,baud,sample_rate = SAMPLE_RATE_20HZ):
    self._addr = addr
    self._array = array
    self._sample_rate = sample_rate

    if self._array == 32:
      self._array_x = 8
      self._array_y = 4
    else:
      self._array_x = 6
      self._array_y = 6
      
    self.master =  modbus_rtu.RtuMaster(serial.Serial(port="/dev/ttyAMA0",baudrate=baud, bytesize=8, parity='N', stopbits=1))
    self.master.set_timeout(1.0)

  def begin(self):
    '''!
      @brief Init the sensor
      @return True(init succeed)/False(init failed)
    '''
  
    time.sleep(0.5)
    if (self._addr>=0x0001 and self._addr<=0x0020):
      
      if self.get_dev_state():
        self.set_sample_rate()
        return True
      else:
        return False
    else:
      return False

  def set_thld(self, thld):
    '''!
      @brief Set the threshold of the sensor
      @param thld The threshold of the sensor
    '''
    sbuf = [((thld>>8) & 0xff) | ((thld & 0xFF) << 8)]
    self._write_reg(HOUTINGREG_THLD, sbuf)
    return

  def get_datas(self):
    '''!
      @brief Get the ADC data of the sensor
      @return ADC data
      @retval    result    The result of get data from the sensor
      @retval    adcval    The ADC data of the sensor,the ordered two-dimensional array that can be accessed directly
    '''
    rbuf = [0x0000]*(self._array)
    adc_dat = adc_data()
    rbuf = self._read_reg(INPUTREG_GETDATAS, self._array )

    for i in range(self._array_y, 0 , -1):
      tempy = self._array_y - i
      for j in range(0,self._array_x):
        tempx = (i- 1) * self._array_x + j
        adc_dat.adcval[tempy][j] = rbuf[tempx]

    adc_dat.result = 0

    return adc_dat

  def reset_val(self):
    '''!
      @brief Reset the sensor
    '''
    sbuf = [0x0000]
    self._write_reg(HOUTINGREG_RESET, sbuf)
    return

  def set_dev_addr(self,addr):
    '''!
      @brief Set the device address of the sensor
      @param addr The device address of the sensor
    '''
    sbuf = [((addr>>8) & 0xff) | ((addr & 0xFF) << 8)]

    try:
      self._write_reg(HOUTINGREG_ADDR, sbuf)
    except:
      self._addr = addr
    return

  def set_baudrate(self,baud):
    '''!
      @brief Set the baud rate of the sensor
      @param baud The baud rate of the sensor
    '''
    sbuf = [((baud>>8) & 0xff) | ((baud & 0xFF) << 8)]

    self._write_reg(HOUTINGREG_BAUDRATE, sbuf)
    return

  def get_device_info(self):
    '''!
      @brief Get device information
      @return Device information
      @retval    PID    PID of the device
      @retval    VID    VID of the device
      @retval    version    version of the device
    '''
    ver = version_info()
    ver.PID = self.get_PID()
    ver.VID  = self.get_VID()
    ver.version = self.get_Version()

    return ver

  def get_dev_state(self):
    '''!
      @brief Get device state
      @return Device state
    '''
    rbuf = self._read_reg(INPUTREG_DEVSTATE, 1)
    if rbuf[0] == 1:
      return True
    else:
      return False

  def get_adc_value(self,x, y):
    '''!
      @brief Get ADC value
      @param x The x-axis coordinate of the sensor
      @param y The y-axis coordinate of the sensor
      @return ADC value
    '''
    instruction_addr = y * self._array_x + x
  
    rbuf = self._read_reg(INPUTREG_GETDATAS + instruction_addr, 1)

    data =  rbuf[0]
    return data

  def get_PID(self):
    '''!
      @brief Get PID of the device
      @return PID of the device
    '''
    rbuf = self._read_reg(INPUTREG_PID, 1 )

    data =  rbuf[0]
    return data

  def get_VID(self):
    '''!
      @brief Get VID of the device
      @return VID of the device
    '''
    rbuf = self._read_reg(INPUTREG_VID, 1 )
    
    data =  rbuf[0]
    return data
    
  def get_Version(self):
    '''!
      @brief Get version of the device
      @return version of the device
    '''
    rbuf = self._read_reg(INPUTREG_VERSION, 1 )

    data =  rbuf[0]
    return data
  
  def set_sample_rate(self):
    '''!
      @brief Set the sample rate of the sensor
      @param rate The sample rate of the sensor
    '''
    sbuf = [((self._sample_rate>>8) & 0xff) | ((self._sample_rate & 0xFF) << 8)]

    self._write_reg(HOUTINGREG_SAMPLE_RATE, sbuf)
    return
  
  def get_Model(self):
    '''!
      @brief Get model of the device
      @return model of the device
    '''
    rbuf = self._read_reg(INPUTREG_MODEL, 1)

    data =  rbuf[0]
    return data

  def _read_reg(self, reg_addr ,length):
    '''!
      @brief Read data from the sensor
    '''
    return list(self.master.execute(self._addr, cst.READ_INPUT_REGISTERS, reg_addr, length))
  
  def _write_reg(self, reg_addr ,data):
    '''!
      @brief write data from the sensor
    '''
    return list(self.master.execute(self._addr, cst.WRITE_MULTIPLE_REGISTERS, reg_addr, output_value=data))
  