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
 * Version 1.0 - Thomas Bowman Mørch
 * 
 * DESCRIPTION
 * Default sensor sketch for Sensebender Micro module
 * Act as a temperature / humidity sensor by default.
 *
 * If A0 is held low while powering on, it will enter testmode, which verifies all on-board peripherals
 *  
 * Battery voltage is as battery percentage (Internal message), and optionally as a sensor value (See defines below)
 *
 *
 * Version 1.3 - Thomas Bowman Mørch
 * Improved transmission logic, eliminating spurious transmissions (when temperatuere / humidity fluctuates 1 up and down between measurements)
 * Added OTA boot mode, need to hold A1 low while applying power. (uses slightly more power as it's waiting for bootloader messages)
 * 
 * Version 1.4 - Thomas Bowman Mørch
 * 
 * Corrected division in the code deciding whether to transmit or not, that resulted in generating an integer. Now it's generating floats as expected.
 * Simplified detection for OTA bootloader, now detecting if MY_OTA_FIRMWARE_FEATURE is defined. If this is defined sensebender automaticly waits 300mS after each transmission
 * Moved Battery status messages, so they are transmitted together with normal sensor updates (but only every 60th minute)
 * 
 */

#define DEBUG            0

// Enable MySensors Lib debug prints
//#define MY_DEBUG          0

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Define a static node address, remove if you want auto address assignment
#define MY_NODE_ID      1

/*
Node ID   | Node place        |
----------+-------------------+-----
1         | Bedroom           | +
2         | Childroom         | +
3         | Dinner            | +
4         | Outside North     |
5         | Outside South     |
6         | Balcony           |
*/

#define MY_OTA_FIRMWARE_FEATURE 1

// For Winbond W25X40
#define MY_OTA_FLASH_JDECID 0xEF30

#define USE_BMP180              1

#include <MySensors.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

#if ( USE_BMP180 == 0 )
#include <Adafruit_BME280.h>
#else
#include "HTU21D.h"
#include <Adafruit_BMP085.h>
#endif

#include <SPI.h>
#ifndef MY_OTA_FIRMWARE_FEATURE
#include "drivers/SPIFlash/SPIFlash.cpp"
#endif
#include <EEPROM.h>  
#ifdef MY_SIGNING_ATSHA204
#include <sha204_lib_return_codes.h>
#include <sha204_library.h>
#endif

//#include <avr/power.h>

// Uncomment the line below, to transmit battery voltage as a normal sensor value
//#define BATT_SENSOR    199

#define SKETCH_NAME "APMM Temp+Hum+Baro"
#define SKETCH_VERSION "1.2"

// Child sensor ID's
#define CHILD_ID_TEMP       1
#define CHILD_ID_HUM        2
#define CHILD_ID_BARO       3

const float ALTITUDE = 210; // <-- adapt this value to your own location's altitude.

const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,     // "Stable Weather Pattern"
  SUNNY = 1,      // "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,     // "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,   // "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4, // "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5     // "Unknown (More Time needed)
};


// How many milli seconds between each measurement
#define MEASURE_INTERVAL ( 5 * 60000 ) // 5 minut

#define BATTERY_SEND_INTERVAL (60)    //(1)
// How many milli seconds should we wait for OTA?
#define OTA_WAIT_PERIOD 300

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to the controller
#define FORCE_TRANSMIT_INTERVAL (6) // 6 * MEASURE_INTERVAL (5 minut) = 30 minut

// When MEASURE_INTERVAL is 60000 and FORCE_TRANSMIT_INTERVAL is 10, we force a transmission every 30 minutes.
// Between the forced transmissions a tranmission will only occur if the measured value differs from the previous measurement

// HUMI_TRANSMIT_THRESHOLD tells how much the humidity should have changed since last time it was transmitted. Likewise with
// TEMP_TRANSMIT_THRESHOLD for temperature threshold.
#define HUMI_TRANSMIT_THRESHOLD 0.5
#define TEMP_TRANSMIT_THRESHOLD 0.5
#define PRESS_TRANSMIT_THRESHOLD 0.5

// Pin definitions
#define LED_PIN        13
#define ATSHA204_PIN   17 // A3

// this CONVERSION_FACTOR is used to convert from Pa to kPa in forecast algorithm
// get kPa/h be dividing hPa by 10 
#define CONVERSION_FACTOR (1.0/10.0)

/************************************/
/********* GLOBAL VARIABLES *********/
/************************************/
#ifdef MY_SIGNING_ATSHA204
const int sha204Pin = ATSHA204_PIN;
atsha204Class sha204(sha204Pin);
#endif

#if ( USE_BMP180 == 0 )
Adafruit_BME280 bme; // I2C
#else
HTU21D humiditySensor;
Adafruit_BMP085 bme; // I2C
#endif

SPIFlash flash(8, 0xEF30); // (Cs, ID)

// Sensor messages
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgBaro(CHILD_ID_BARO, V_PRESSURE);
MyMessage msgFore(CHILD_ID_BARO, V_FORECAST);

#ifdef BATT_SENSOR
MyMessage msgBatt(BATT_SENSOR, V_VOLTAGE);
#endif

// Global settings
int measureCount = 0;
int sendBattery = 0;

boolean highfreq = true;
boolean ota_enabled = false; 
boolean transmission_occured = false;

// Storage of old measurements
float lastTemperature = -100;
float lastHumidity = -100;
float lastPressure = -100;
long lastBattery = -100;
int lastForecast = -1;

const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];

int minuteCount = 0;
bool firstRound = true;
// average value is used in forecast algorithm.
float pressureAvg;
// average after 2 hours is used as reference value for the next iteration.
float pressureAvg2;

float dP_dt;

/****************************************************
 *
 * Setup code 
 *
 ****************************************************/
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  //Serial.begin(9600);
#if ( DEBUG == 1 )
  //Serial.begin(9600);
  Serial.print(F("Arduino Pro Mini Motherboard FW "));
  Serial.print(SKETCH_VERSION);
  Serial.flush();
#endif

  // Make sure that ATSHA204 is not floating
  pinMode(ATSHA204_PIN, INPUT);
  digitalWrite(ATSHA204_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH); 

#if ( USE_BMP180 == 0 )
  bme.begin(0x76);
#else
  humiditySensor.begin();
  bme.begin();
#endif

  digitalWrite(LED_PIN, LOW);

  sendSensorMeasurements(false);
  sendBattLevel(false);
#if ( DEBUG == 1 )  
#ifdef MY_OTA_FIRMWARE_FEATURE  
  Serial.println("OTA FW update enabled");
#endif
#endif
}

void presentation()
{
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_BARO, S_BARO);
  
#ifdef BATT_SENSOR
  present(BATT_SENSOR, S_POWER);
#endif
}

/***********************************************
 *
 *  Main loop function
 *
 ***********************************************/
void loop() {
  
  measureCount ++;
  sendBattery ++;
  bool forceTransmit = false;
  transmission_occured = false;

#ifndef MY_OTA_FIRMWARE_FEATURE
  if ((measureCount == 5) && highfreq) 
  {
    if (!ota_enabled) clock_prescale_set(clock_div_8); // Switch to 1Mhz for the reminder of the sketch, save power.
    highfreq = false;
  }
#else
  //clock_prescale_set(clock_div_8); // Switch to 1Mhz for the reminder of the sketch, save power.
#endif  

  if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true; 
    measureCount = 0;
  }

  sendSensorMeasurements(forceTransmit);
#ifdef MY_OTA_FIRMWARE_FEATURE
  if (transmission_occured) {
      wait(OTA_WAIT_PERIOD);
  }
#endif
  sleep(MEASURE_INTERVAL);
 }


/*********************************************
 *
 * Sends sensors data
 *
 * Parameters
 * - force : Forces transmission of a value (even if it's the same as previous measurement)
 *
 *********************************************/
void sendSensorMeasurements(bool force)
{
  bool tx = force;

  float pressure = bme.readPressure();
  float pressHg = pressure * 0.0075006375541921;
  int forecast = sample(pressure);
  
#if ( USE_BMP180 == 0 )
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
#else
  float hum = humiditySensor.readHumidity();
  float temp = humiditySensor.readTemperature();
#endif
  float diffPress = abs(lastPressure - pressHg);
  float diffTemp = abs(lastTemperature - temp);
  float diffHum = abs(lastHumidity - hum);


#if ( DEBUG == 1 )
  clock_prescale_set(clock_div_1);
  Serial.print(F("Temperature = "));
  Serial.print(temp);
  Serial.println(F(" *C"));
  Serial.print(F("Humidity = "));
  Serial.print(temp);
  Serial.println(F(" \%"));
  Serial.print(F("Pressure = "));
  Serial.println(pressHg);
  Serial.print(F("Forecast = "));
  Serial.println(weather[forecast]);
  
  Serial.print(F("TempDiff :"));Serial.println(diffTemp);
  Serial.print(F("HumDiff  :"));Serial.println(diffHum); 
#endif
  
  if (diffTemp > TEMP_TRANSMIT_THRESHOLD) tx = true;
  if (diffHum > HUMI_TRANSMIT_THRESHOLD) tx = true;
  if (diffPress > PRESS_TRANSMIT_THRESHOLD) tx = true;
  if (forecast != lastForecast) tx = true;
  
  if (tx) {
    measureCount = 0;
#if ( DEBUG == 1 )    
    Serial.print(F("Send..."));
#endif    
    send(msgTemp.set(temp,1));
    send(msgHum.set(hum, 1));
    send(msgBaro.set(pressHg, 0));
    send(msgFore.set(weather[forecast]));
    lastTemperature = temp;
    lastHumidity = hum;
    lastPressure = pressHg;
    lastForecast = forecast;
    transmission_occured = true;
    if (sendBattery > BATTERY_SEND_INTERVAL) {
     sendBattLevel(true); // Not needed to send battery info that often
     sendBattery = 0;
    }
  }
}

/********************************************
 *
 * Sends battery information (battery percentage)
 *
 * Parameters
 * - force : Forces transmission of a value
 *
 *******************************************/
void sendBattLevel(bool force)
{
  static const float full_battery_v = 3169.0;

  if (force) lastBattery = -1;

  float level = readVcc() / full_battery_v;

  if (level != lastBattery) {
    lastBattery = level;

#ifdef BATT_SENSOR
    send(msgBatt.set(vcc));
#endif

    // Calculate percentage
    uint8_t percent = level * 100;

    sendBatteryLevel(percent);
    transmission_occured = true;
  }
}

/*******************************************
 *
 * Internal battery ADC measuring 
 *
 *******************************************/
long readVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADcdMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
 
}

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
  {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}

// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185)
  {
    minuteCount = 6;
  }

  if (minuteCount == 5)
  {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else
    {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) //first time initial 3 hour
    {
      dP_dt = change; //note this is for t = 1 hour
    }
    else
    {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else
    {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else
    {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    }
    else
    {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 3; // note this is for t = 3 hour
    }
    else
    {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
  {
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25))
  {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25)
  {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
  {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt >(-0.05)) && (dP_dt < 0.05))
  {
    forecast = STABLE;
  }
  else
  {
    forecast = UNKNOWN;
  }

  // uncomment when debugging
  Serial.print(F("Forecast at minute "));
  Serial.print(minuteCount);
  Serial.print(F(" dP/dt = "));
  Serial.print(dP_dt);
  Serial.print(F("kPa/h --> "));
  Serial.println(weather[forecast]);

  return forecast;
}
