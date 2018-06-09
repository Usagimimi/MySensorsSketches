/*
 PROJECT: MySensors / LiON charger board 
 PROGRAMMER: AWI
 DATE: 28 april 2015/ last update: 11 may 2015 / BH1750 added: 5 September 2015
 FILE: MS_Solar_2.ino
 LICENSE: Public domain

 Hardware: Ceech - ATmega328p board w/ ESP8266 and NRF24l01+ socket LTC4067 lithium battery charger
  and MySensors 1.4

  Temp & Humidity - HTU21
  Barometer & Temp - BMP085
  Light sensor - BH1750
  On board EEPROM (I2C)
  On board Li-On charger with multiple V/A measurements
  
Special:
  program with Arduino Pro 3.3V 8Mhz
  
SUMMARY:
  Reads on-board sensors and send to gateway /controller
 Remarks:
  On board EEPROM and MOSFET not used in this sketch
  Fixed node-id
*/
#define DEBUG            0

// Enable MySensors Lib debug prints
//#define MY_DEBUG          0

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#define MY_RF24_CE_PIN    7
#define MY_RF24_CS_PIN    8

// Define a static node address, remove if you want auto address assignment
#define MY_NODE_ID      5

/*
Node ID   | Node place        |
----------+-------------------+-----
1         | Bedroom           |
2         | Childroom         | +
3         | Dinner            | +
4         | Outside North     |
5         | Outside South     |
6         | Balcony           | +
*/

#include <SPI.h>
#include <MySensor.h>
#include <Wire.h>         // I2C
#include "TSL2561.h"
#include "HTU21D.h"

#define SKETCH_NAME "CEECH Temp+Humidity"
#define SKETCH_VERSION "1.0"

// Child sensor ID's
#define CHILD_ID_TEMP       1
#define CHILD_ID_HUM        2
#define CHILD_ID_LIGHT      3
#define CHILD_ID_BATT       4
#define CHILD_ID_SOLAR      5

// How many milli seconds between each measurement
#define MEASURE_INTERVAL ( 5 * 60000 ) // 5 minut

#define BATTERY_SEND_INTERVAL (60)    //(1)

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to the controller
#define FORCE_TRANSMIT_INTERVAL (6) // 6 * MEASURE_INTERVAL (5 minut) = 30 minut

// When MEASURE_INTERVAL is 60000 and FORCE_TRANSMIT_INTERVAL is 10, we force a transmission every 30 minutes.
// Between the forced transmissions a tranmission will only occur if the measured value differs from the previous measurement

// HUMI_TRANSMIT_THRESHOLD tells how much the humidity should have changed since last time it was transmitted. Likewise with
// TEMP_TRANSMIT_THRESHOLD for temperature threshold.
#define HUMI_TRANSMIT_THRESHOLD 0.5
#define TEMP_TRANSMIT_THRESHOLD 0.5
#define LUX_TRANSMIT_THRESHOLD 100

// Pin definitions
#define LED_PIN        13
#define ATSHA204_PIN   17 // A3

#define LTC4067_CHRG_PIN  A1    //analog input A1 on ATmega 328 is /CHRG signal from LTC4067
#define batteryVoltage_PIN  A0    //analog input A0 on ATmega328 is battery voltage ( /2)
#define solarVoltage_PIN  A2    //analog input A2 is solar cell voltage (/ 2)
#define solarCurrent_PIN  A6    //analog input A6 is input current ( I=V/Rclprog x 1000 )
#define batteryChargeCurrent_PIN  A7    //analog input A7 is battery charge current ( I=V/Rprog x 1000 )
#define LTC4067_SUSPEND_PIN 9   //digital output D9 - drive it high to put LTC4067 in SUSPEND mode

/************************************/
/********* GLOBAL VARIABLES *********/
/************************************/
const int sha204Pin = ATSHA204_PIN;
atsha204Class sha204(sha204Pin);

HTU21D humiditySensor;
// The address will be different depending on whether you let
// the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases
// use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively
TSL2561 tsl(TSL2561_ADDR_FLOAT); 

SPIFlash flash(8, 0xEF30); // (Cs, ID)

// Sensor messages
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgLight(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage msgBatt(CHILD_ID_BATT, V_VOLTAGE);
MyMessage msgSolar(CHILD_ID_SOLAR, V_VOLTAGE);

// Global settings
int measureCount = 0;
int sendBattery = 0;

boolean highfreq = true;
boolean ota_enabled = false; 
boolean transmission_occured = false;

// Storage of old measurements
float lastTemperature = -100;
float lastHumidity = -100;
long lastBattery = -100;
long lastSolar = -100;
uint16_t lastLux = 65535;

/****************************************************
 *
 * Setup code 
 *
 ****************************************************/
void setup()  
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(9600);
#if ( DEBUG == 1 )
  Serial.begin(9600);
  Serial.print(F("Arduino Pro Mini Motherboard FW "));
  Serial.print(SKETCH_VERSION);
  Serial.flush();
#endif

  // Make sure that ATSHA204 is not floating
  pinMode(ATSHA204_PIN, INPUT);
  digitalWrite(ATSHA204_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH); 

  // use VCC (3.3V) reference
  analogReference(DEFAULT);               // default external reference = 3.3v for Ceech board
  VccReference = 3.323 ;                  // measured Vcc input (on board LDO)
  pinMode(LTC4067_SUSPEND_PIN, OUTPUT);         // suspend of Lion charger set
  digitalWrite(LTC4067_SUSPEND_PIN,LOW);            //  active (non suspend) at start

  humiditySensor.begin();

  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2561_GAIN_0X);         // set no gain (for bright situtations)
  tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // medium integration time (medium light)
  tsl.setTiming(TSL2561_INTEGRATIONTIME_402MS);  // longest integration time (dim light)

  digitalWrite(LED_PIN, LOW);

//  sendSensorMeasurements(false);
//  sendBattLevel(false);
#if ( DEBUG == 1 )  
#ifdef MY_OTA_FIRMWARE_FEATURE  
  Serial.println("OTA FW update enabled");
#endif
#endif

 }

void presentation()
{
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  
  // Register all sensors to gateway (they will be created as child devices)
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  
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

    // Simple data read example. Just read the infrared, fullspecrtrum diode 
  // or 'visible' (difference between the two) channels.
  // This can take 13-402 milliseconds! Uncomment whichever of the following you want to read
  uint16_t lux = tsl.getLuminosity(TSL2561_VISIBLE);     
  //uint16_t lux = tsl.getLuminosity(TSL2561_FULLSPECTRUM);
  //uint16_t lux = tsl.getLuminosity(TSL2561_INFRARED);
  
  float hum = humiditySensor.readHumidity();
  float temp = humiditySensor.readTemperature();
  float diffTemp = abs(lastTemperature - temp);
  float diffHum = abs(lastHumidity - hum);
  uint16_t diffLux = abs(lastLux - lux);



#if ( DEBUG == 1 )
  clock_prescale_set(clock_div_1);
  Serial.print(F("Temperature = "));
  Serial.print(temp);
  Serial.println(F(" *C"));
  Serial.print(F("Humidity = "));
  Serial.print(temp);
  Serial.println(F(" \%"));
  Serial.print(F("Luminosity = "));
  Serial.println(lux);

  Serial.print(F("TempDiff :"));Serial.println(diffTemp);
  Serial.print(F("HumDiff  :"));Serial.println(diffHum); 
#endif
  
  if (diffTemp > TEMP_TRANSMIT_THRESHOLD) tx = true;
  if (diffHum > HUMI_TRANSMIT_THRESHOLD) tx = true;
  if (diffLux > LUX_TRANSMIT_THRESHOLD) tx = true;
  
  if (tx) {
    measureCount = 0;
#if ( DEBUG == 1 )    
    Serial.print(F("Send..."));
#endif    
    send(msgTemp.set(temp,1));
    send(msgHum.set(hum, 1));
    send(msgLight.set(lux));
    lastTemperature = temp;
    lastHumidity = hum;
    lastLux = lux;
    transmission_occured = true;
    if (sendBattery > BATTERY_SEND_INTERVAL) {
     sendBattLevel(true); // Not needed to send battery info that often
     sendBattery = 0;
    }
  }
}


void sendVoltage(void)
// battery and charging values
{
  // get Battery Voltage & charge current
  float batteryVoltage = ((float)analogRead(batteryVoltage_PIN)* VccReference/1024) * 2;  // actual voltage is double
  Serial.print("Batt: ");
  Serial.print(batteryVoltage);
  Serial.print("V ; ");
  float batteryChargeCurrent = ((float)analogRead(batteryChargeCurrent_PIN) * VccReference/1024)/ 2.5 * 1000; // current(mA) = V/Rprog(kohm)
  Serial.print(batteryChargeCurrent);
  Serial.println("mA ");

  // get Solar Voltage & charge current
  float solarVoltage = ((float)analogRead(solarVoltage_PIN)/1024 * VccReference) * 2 ;    // actual voltage is double
  Serial.print("Solar: ");
  Serial.print(solarVoltage);
  Serial.print("V ; ");
  // get Solar Current
  float solarCurrent = ((float)analogRead(solarCurrent_PIN)/1024 * VccReference)/ 2.5 * 1000;   // current(mA) = V/Rclprog(kohm)
  Serial.print(solarCurrent);
  Serial.print(" mA; charge: ");
  Serial.println(digitalRead(LTC4067_CHRG_PIN)?"No":"Yes");
  // send battery percentage for node
  int battPct = 1 ;
  if (batteryVoltage > VccMin){
    battPct = 100.0*(batteryVoltage - VccMin)/(VccMax - VccMin);
  }
  Serial.print("BattPct: ");
  Serial.print(battPct);
  Serial.println("% ");

  gw.send(batteryVoltageMsg.set(batteryVoltage, 3));      // Send (V)
  gw.send(batteryCurrentMsg.set(batteryChargeCurrent, 6));    // Send (mA)
  gw.send(solarVoltageMsg.set(solarVoltage, 3));        // Send (V)
  gw.send(solarCurrentMsg.set(solarCurrent, 6));        // Send (mA)
  gw.sendBatteryLevel(battPct);
  
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
long readVcc() {
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

/* Ceech board specifics for reference:
It provides power for the circuit and charges the backup single-cell lithium battery while greatly extends battery life. You can monitor the voltages and currents. It has suspend mode, which reduces current consumption to around 40μA. The power source is a small, 5V solar cell. Connections:

analog input A1 on ATmega 328 is /CHRG signal from LTC4067 (indicates fully charged)
analog input A0 on ATmega328 is battery voltage
analog input A2 is solar cell voltage
analog input A6 is input current ( I=V/Rclprog x 1000 )
analog input A7 is battery charge current ( I=V/Rprog x 1000 )
digital output D9 - drive it high to put LTC4067 in SUSPEND mode
All the voltages on analog inputs can be read with an analogRead() command in the Arduino IDE sketch. Those on inputs A0 an A2 represent direct values of the measured voltages divided by 2. The voltages on analog inputs A6 and A7 can be translated to currents. For example:

Let us say that the voltage on A7 is 0.12V. And the trimmer pot on PROG pin is set to 2.5kOhm. This means that the current into the battery equals to 0.12V/2500ohm x 1000, which is 48mA.

voltmeters on both battery and solar cell connections
They are connected to analog inputs A0 and A2 on the ATmega328p. The voltage dividers resistors are equal, so the measured voltage is double the shown voltage.

NRF24l01+ socket
with CE and CSN pins connected to digital pins 7 and 8 ( you use RF24 radio(7, 8); in Arduino code). There is a 4.7uF capacitor connected across Vin and GND of the port
*/

