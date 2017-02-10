#define DEBUG            1

// Enable MySensors Lib debug prints
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Define a static node address, remove if you want auto address assignment
#define MY_NODE_ID      4 // Outside North

//#define MY_OTA_FIRMWARE_FEATURE 1

// For Winbond W25X40
//#define MY_OTA_FLASH_JDECID 0xEF30

#define MY_RF24_CE_PIN    7
#define MY_RF24_CS_PIN    8
#include <MySensors.h>
#include <Wire.h>
#include "HTU21D.h"
#include <SPI.h>
#include <RunningAverage.h>

// Uncomment the line below, to transmit battery voltage as a normal sensor value
//#define BATT_SENSOR    199

#define SKETCH_NAME "CEECH Temp+Humidity"
#define SKETCH_VERSION "1.0"


#define AVERAGES 2

// Child sensor ID's
#define CHILD_ID_TEMP  1
#define CHILD_ID_HUM   2

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

/************************************/
/********* GLOBAL VARIABLES *********/
/************************************/
HTU21D humiditySensor;
// Sensor messages
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

#ifdef BATT_SENSOR
MyMessage msgBatt(BATT_SENSOR, V_VOLTAGE);
#endif

// Global settings
int measureCount = 0;
int sendBattery = 0;
boolean isMetric = true;
boolean highfreq = true;
boolean ota_enabled = false; 
boolean transmission_occured = false;

// Storage of old measurements
float lastTemperature = -100;
int lastHumidity = -100;
long lastBattery = -100;

RunningAverage raHum(AVERAGES);

void testMode();
void sendTempHumidityMeasurements(bool force);
void sendBattLevel(bool force);

float readVcc() 
{
  signed long resultVcc;
  float resultVccFloat;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(10); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  resultVcc = ADCL;
  resultVcc |= ADCH<<8;
  resultVcc = 1126400L / resultVcc; // Back-calculate AVcc in mV
  resultVccFloat = (float) resultVcc / 1000.0; // Convert to Float
  return resultVccFloat;
}

int current = A6;
int cell = A2;
int lipo = A0;
int CHRG = A7;
float vout = 0.0;
float vin = 0.0;
float R1 = 47000.0; // resistance of R1
float R2 = 10000.0; // resistance of R2
int value = 0;

void setup() 
{
#if ( DEBUG == 1 )
  Serial.begin(9600); // Actually 1200
  Serial.print(F("Arduino Pro Mini Motherboard FW "));
  Serial.print(SKETCH_VERSION);
  Serial.flush();
#endif
}

void presentation()
{
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  
  present(CHILD_ID_TEMP,S_TEMP);
  present(CHILD_ID_HUM,S_HUM);
  
#ifdef BATT_SENSOR
  present(BATT_SENSOR, S_POWER);
#endif
}


void loop() 
{
  measureCount ++;
  sendBattery ++;
  bool forceTransmit = false;
  transmission_occured = false;

   if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true; 
    measureCount = 0;
  }
  
  Serial.println("Test!");
  sleep(MEASURE_INTERVAL);
};

void loop1() 
{
  float napetost;// = readVcc();
  
  value = analogRead(cell);
  vout = (value * napetost) / 1024.0;
  vin = vout / (R2/(R1+R2)); 
  if (vin<0.09) 
  {
    vin=0.0;
  }
  
  float tok = ((analogRead(current) * napetost / 1024 ) *250) / 3.3; // convert the ADC value to miliamps
  float baterija = ( analogRead(lipo) * napetost / 1024 ) * 2; // measuring battery voltage
  int polnjenje = analogRead(CHRG);
  Serial.print("Vcc = ");
  Serial.print(napetost);
  Serial.println("V");
  delay(400);
  Serial.print("Charge current = ");
  Serial.print(tok);
  Serial.println("mA");
  delay(400);
  Serial.print("Solar cell voltage = ");
  Serial.print(vin);
  Serial.println("V");
  delay(400);
  Serial.print("Battery voltage = ");
  Serial.print(baterija);
  Serial.println("V");
  delay(400);
  Serial.print("CHRG = ");
  Serial.println(polnjenje);
  Serial.println("----------------------------");
  delay(2000);
}
/*
Improving accuracy:
To do so, simply measure your Vcc with a voltmeter and with our readVcc() function. Then, replace the constant 1107035L with a new constant:
scale_constant = internal1.1Ref * 1024 * 1000
where
internal1.1Ref = 1,1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
Example:
For instance, I measure 3,43V from my FTDI, the calculated value of Vref is 1,081V.
So (1,081 x 1000 x 1024) = 1107034,95 or 1107035L rounded up.
Use smoothing example from IDE to smooth the data from ADC.
*/

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
  
  raHum.addValue(hum);
  
  float diffTemp = abs(lastTemperature - temp);
  float diffHum = abs(lastHumidity - raHum.getAverage());
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


