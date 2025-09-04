/*!
 * @file getTactileData.ino
 * @brief This is an example to show how to get tactile data from DFRobot Tactile Sensor.
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [jiali](zhixinliu@dfrobot.com)
 * @version V1.0
 * @date 2025-08-06
 * @url 
 */
#include "DFRobot_Tactile.h"

/* ---------------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC    |              5V                |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
/*The default baud rate is 115200, and it supports 9600,57600, and 912600*/

#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
#include <SoftwareSerial.h>
#endif

#define DFRobot_TACTILE_ARRAY_SIZE 32 // if you use (4*8)32-channel tactile sensor
//#define DFRobot_TACTILE_ARRAY_SIZE 36 // if you use (6*6)36-channel tactile sensor

#if (DFRobot_TACTILE_ARRAY_SIZE == 32)//4*8
#define DFRobot_TACTILE_ARRAY_X  8
#define DFRobot_TACTILE_ARRAY_Y  4
#else                                 //6*6
#define DFRobot_TACTILE_ARRAY_X  6
#define DFRobot_TACTILE_ARRAY_Y  6
#endif

#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
  SoftwareSerial mySerial(/*rx =*/4, /*tx =*/5);
  DFRobot_Tactile tactile(/*array =*/DFRobot_TACTILE_ARRAY_SIZE,/*addr =*/1, /*s =*/&mySerial);
#else
  DFRobot_Tactile tactile(/*array =*/DFRobot_TACTILE_ARRAY_SIZE,/*addr =*/1, /*s =*/&Serial1);
#endif

void setup()
{
  //Init MCU communication serial port
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
  mySerial.begin(115200);
#elif defined(ESP32)
  Serial1.begin(115200, SERIAL_8N1, /*rx =*/D3, /*tx =*/D2);
#else
  Serial1.begin(115200);
#endif

  Serial.begin(115200);
  while(tactile.begin() != 0){
    Serial.println(" Sensor initialize failed!!");
    delay(1000);
  }
  Serial.println(" Sensor  initialize success!!");

  sVersionInfo_t ver;
  //Get device information
  ver = tactile.getDeviceInfo();
  Serial.print(" VID: ");
  Serial.println(ver.VID);
  Serial.print(" PID: ");
  Serial.println(ver.PID);
  Serial.print(" Version: V");
  for(int i = 0;i < 4;i++){
    Serial.print(ver.version >> (12-i*4) & 0xF);
    
    if(i != 3){
      Serial.print(".");
    }else{
      Serial.println();
    }
  }
  delay(1000);
  //Set threshold value
  tactile.setThld(200);

  //The sensor returns to its default value,Addr doesn't change
  //tactile.resetVal();
  
  //Set device address
  //tactile.setDevAddr(1);

  //set uart baudrate
  //tactile.setBaudrate(BAUD_115200);

}

void loop()
{
  sAdcDatas_t adcDatas;
  //Get tactile data
  adcDatas = tactile.getDatas();
  //Print tactile data
  if(adcDatas.result == 0){
    for(int i = 0;i <DFRobot_TACTILE_ARRAY_Y;i++){

      for(int j = 0;j < DFRobot_TACTILE_ARRAY_X;j++){
        Serial.print(adcDatas.adcval[i][j]);
        Serial.print("\t");
      }
      Serial.println();
    }
  }
  else{
    Serial.println("Read ADC data failed!!");
  }
  Serial.println();
  delay(50);
}
