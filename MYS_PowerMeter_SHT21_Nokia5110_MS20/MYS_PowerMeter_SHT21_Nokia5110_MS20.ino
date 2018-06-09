/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik EKblad
 * 
 * DESCRIPTION
 * This sketch provides an example how to implement a humidity/temperature
 * sensor using DHT11/DHT-22 
 * http://www.mysensors.org/build/humidity
 */

#define DEBUG            0

// Enable MySensors Lib debug prints
//#define MY_DEBUG

// Enable and select radio type attached
#define MY_RF24_CE_PIN 7
#define MY_RF24_CS_PIN 8
#define MY_RADIO_NRF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

// Define a static node address, remove if you want auto address assignment
#define MY_NODE_ID      10

#include <SPI.h>
#include <MySensors.h>
#include "EmonLib.h" 
#include <LCD5110_Graph_SPI.h>
#include <Wire.h>
#include "HTU21D.h"
#include <EEPROM.h>  

// Uncomment the line below, to transmit battery voltage as a normal sensor value
//#define BATT_SENSOR    199

#define SKETCH_NAME "Energy Humidity Sensor"
#define SKETCH_VERSION "1.2"

// Child sensor ID's
#define CHILD_ID_TEMP  1
#define CHILD_ID_HUM   2
#define CHILD_ID_POWER 3

// How many milli seconds between each measurement
#define MEASURE_INTERVAL ( 5 * 60000 ) // 5 minut

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to the controller
#define FORCE_TRANSMIT_INTERVAL (6) // 6 * MEASURE_INTERVAL (5 minut) = 30 minut

// When MEASURE_INTERVAL is 60000 and FORCE_TRANSMIT_INTERVAL is 10, we force a transmission every 30 minutes.
// Between the forced transmissions a tranmission will only occur if the measured value differs from the previous measurement

// HUMI_TRANSMIT_THRESHOLD tells how much the humidity should have changed since last time it was transmitted. Likewise with
// TEMP_TRANSMIT_THRESHOLD for temperature threshold.
#define HUMI_TRANSMIT_THRESHOLD 0.5
#define TEMP_TRANSMIT_THRESHOLD 0.5

#define POWER_VOLTAGE       220.0

/***********************************/
/********* PIN DEFINITIONS *********/
/***********************************/
#define LED_pin 9

/*****************************/
/********* FUNCTIONS *********/
/*****************************/

/************************************/
/********* GLOBAL VARIABLES *********/
/************************************/

MyMessage msgHum(CHILD_ID_HUM, S_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, S_TEMP);
MyMessage IrmsMsg(CHILD_ID_POWER,V_WATT);
MyMessage kWhMsg(CHILD_ID_POWER,V_KWH);

HTU21D humiditySensor;

// параметры подключения дисплея
LCD5110 myGLCD(5,6,3);

// энергопотребление
EnergyMonitor emon1;

extern uint8_t SmallFont[];

// Global settings
int measureCount = 0;
boolean isMetric = true;

double Irms;
float Pcur;
double delta=0;

// Storage of old measurements
float lastTemperature = -100;
float lastHumidity = -100;
boolean transmission_occured = false;

/**********************************/
/********* IMPLEMENTATION *********/
/**********************************/
#define WINDOW 15
void setup()
{
  double cIrms = 0;
  boolean flag = false;
  double aIrms[WINDOW];  
  
  //Serial.begin(9600);

  humiditySensor.begin();

  emon1.current(A0, 29.0);             // Current: input pin, calibration. For 30A clamp - 29, for 100A - 111.1
  
  // инициализация экрана
  myGLCD.InitLCD(70);  // если экран плохо читается - поменяйте значение контраста в этой строчке
  myGLCD.setFont(SmallFont);
 
  myGLCD.print("Energy Monitor", 0, 0);
  myGLCD.print(" calibrating", 0, 15);
  
  myGLCD.update();
  
  while (!flag)
  {
    aIrms[0] = emon1.calcIrms(1480); // первое значение при измерении явно "кривое"
    for (int i=0; i<WINDOW; i++)
    {
      aIrms[i] = emon1.calcIrms(1480);
      cIrms = cIrms + aIrms[i];
      delay(100);
    }
    delta = cIrms/WINDOW;
    flag = true;
  }
  // калибровка - конец
  // датчик можно повесить на провод (как только надпиь "calibrating" пропала с экрана)  
}

void presentation()
{
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  
  present(CHILD_ID_TEMP,S_TEMP);
  present(CHILD_ID_HUM,S_HUM);
  
  present(CHILD_ID_POWER, S_POWER);  // Register this device as power sensor
}

void loop()      
{  

  measureCount ++;
  bool forceTransmit = false;
  transmission_occured = false;

  if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true; 
    measureCount = 0;
  }
    
  sendTempHumidityMeasurements(forceTransmit);
  
  raedEM();
  send(IrmsMsg.set(Pcur, 1));
#if ( DEBUG == 1 )  
  Serial.print("Watt: ");
  Serial.println(Irms*POWER_VOLTAGE);
#endif
  send(kWhMsg.set(Pcur, 1));

  // выводим данные
  displayLCD();

  //sleep(MEASURE_INTERVAL);
  delay(MEASURE_INTERVAL);
}

/*********************************************
 *
 * Sends temperature and humidity from HTU21 sensor
 *
 * Parameters
 * - force : Forces transmission of a value (even if it's the same as previous measurement)
 *
 *********************************************/
void sendTempHumidityMeasurements(bool force)
{
  bool tx = force;
  
  float hum = humiditySensor.readHumidity();
  float temp = humiditySensor.readTemperature();
   
  float diffTemp = abs(lastTemperature - temp);
  float diffHum = abs(lastHumidity - hum);
#if ( DEBUG == 1 )
  clock_prescale_set(clock_div_1);
  Serial.print(F("TempDiff :"));Serial.println(diffTemp);
  Serial.print(F("HumDiff  :"));Serial.println(diffHum); 
#endif
  if (isnan(diffHum)) tx = true; 
  if (diffTemp > TEMP_TRANSMIT_THRESHOLD) tx = true;
  if (diffHum > HUMI_TRANSMIT_THRESHOLD) tx = true;

  if (tx) {
    measureCount = 0;
#if ( DEBUG == 1 )    
    Serial.print("T: ");Serial.println(temp);
    Serial.print("H: ");Serial.println(hum);
#endif    
    send(msgTemp.set(temp,2));
    send(msgHum.set(hum, 2));
    lastTemperature = temp;
    lastHumidity = hum;
    transmission_occured = true;
  }
}

void raedEM()
{
  Irms = emon1.calcIrms(1480) - delta;  // Calculate Irms only
  Irms = abs(Irms);
  Pcur = ( Irms * POWER_VOLTAGE ) / 1000;
}


void displayLCD(){
  char buf[15];
  char tbuf[8];
  
  myGLCD.clrScr();
  myGLCD.print("Energy Monitor", 0, 0);
  dtostrf(Irms,4,2,tbuf);
  sprintf(buf, "Irms: %s A", tbuf);
  myGLCD.print(buf, 0, 10);
  dtostrf(Pcur,4,2,tbuf);
  sprintf(buf, "Pcur: %s kVt", tbuf);
  myGLCD.print(buf, 0, 20);
  dtostrf(lastHumidity,5,2,tbuf);  
  sprintf(buf, "Humid: %s %%", tbuf);
  myGLCD.print(buf, 0, 30);
  dtostrf(lastTemperature,5,2,tbuf);
  sprintf(buf, "Temp: %s C", tbuf);
  myGLCD.print(buf, 0, 40);
  myGLCD.update();
}

