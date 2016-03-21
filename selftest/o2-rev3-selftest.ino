// OpenObservatory Rev. 3 board definition
#include "board-v3.h"

// Arduino constructs
#include "Arduino.h"

// use EEPROM on chip to store calibration values for sensors
#include <EEPROM.h>     

// include sensor drivers
#include "bmp180/bmp180.h"
#include "dht11/dht.h"
#include "si1145/si1145.h"
#include "mq135/mq135.h"

// SD card access
#include "sdcard/sdcardutils.c"
#include "sdcard/sdcardutils.h"
#include "sdcard/Fat16util.h"
#include "sdcard/SdCard.h"
#include "sdcard/Fat16.h"
#include "sdcard/SdInfo.h"
#include "sdcard/Fat16Config.h"
#include "sdcard/Fat16mainpage.h"

#define RZERO_SET_ADDR 0 // a flag indicating whether the R zero has been calibrated
#define RZERO_ADDR 1  // calibration value for R zero
#define RZERO_FLAG 0xAA // a flag to set

#define SYNC_INT 2000

//#define DEBUG

#ifdef DEBUG
  #define DPrint(...) { Serial.print(__VA_ARGS__); }
  #define DPrintln(...) { Serial.println(__VA_ARGS__); }
#else
  #define DPrint(...) {}
  #define DPrintln(...) {}
#endif

#define SPrint(...) { Serial.print(F(__VA_ARGS__)); }
#define SPrintln(...) { Serial.println(F(__VA_ARGS__)); }


DHT xDHT11(DHTPIN, DHTTYPE);

MQ135 xMQ135 = MQ135(MQPIN, 5., 3.3);
//float fRZero = 76.63; // datasheet reference value
float fRZero = 184.16;

SI1145 xSi1145 = SI1145();

//BMP180 bmp;
BMP180 xBMP180;

SdCard card;
Fat16 fs;



//bool ledState;
bool cardInserted;
bool cardProtected;

bool logging = false;
bool initRequired = true; // records if initialization was attempted but failed

struct sensorData
{
  float Temp;
  float Pressure;
  float Humidity;
  float UV;
  float IR;
  float Illuminance;
  float CO2;   
};

struct sensorIntegrity
{
  bool BMP180_OK;
  bool Si1145_OK;
  bool DHT11_OK;
  bool MQ135_OK;
};

sensorData sensors;
sensorIntegrity sensorStatus;

uint64_t startTime;


void setup() {
  // put your setup code here, to run once:

  startTime = millis();
  Serial.begin(SERIAL_SPEED);
  SPrintln("\n\nStarted OpenObservatory Rev. 3 self test.");
  SPrintln("Position the board on a flat surface and keep obstacles away from sensors.\n");



  // initializing DHT11
  xDHT11.begin();
  float fHumidity, fTemp;
  if (!readHumidity(&fHumidity, &fTemp))
  {
    SPrintln("Hardware error: DHT11 humidity sensor not responding.");
  }
  else
  {
    SPrintln("Found DHT11.");
    if (fHumidity > 90.0 || fHumidity < 10.0)
      SPrintln("Warning: DHT11 humidity sensor may require calibration.");
      
    if (fTemp > 30.0 || fTemp < 10.0)
      SPrintln("Warning: DHT11 temperature sensor may require calibration.");
  }

  // initialize Si1145
  xSi1145.begin();
  float fLux, fUV, fIR;
  if (!readLight(&fLux, &fUV, &fIR))
  {
    SPrintln("Hardware error: Si1145 light sensor not responsing.");
  }
  else
  {
    SPrintln("Found Si1145.");
    // TODO: add calibration check for Si1145 here
  }

  // initialize BMP180
  xBMP180.begin();
  float fPressure;
  if (!readPressure(&fPressure, &fTemp))
  {
    SPrintln("Hardware error: BMP180 barometer not responding.");
  }
  else
  {
    SPrintln("Found BMP180.");
    if (fPressure > 110000.0 || fPressure < 80000.0)
      SPrintln("Warning: BMP180 pressure sensor may require calibration.");

    if (fTemp > 300 || fTemp < 100)
      SPrintln("Warning: BMP180 temperature sensor may require calibration.");
  }

  // initialize MQ135
  xMQ135.begin();
  float fCO2;
  if (!readCO2ppm(&fCO2, fPressure, fHumidity))
  {
    SPrintln("Hardware error: MQ135 CO2 level sensor not responding.");
  }
  else
  {
    SPrintln("Found MQ-series gas sensor, assuming MQ135.");
    // TODO: add calibration check for MQ135

    // check if the MQ135 has been calibrated, and load the calibrated RZero
    uint8_t iRZeroFlag;
    EEPROM.get(RZERO_SET_ADDR, iRZeroFlag);
    if (iRZeroFlag == RZERO_FLAG)
    {
      // calibration value for RZero has been set, load it
      EEPROM.get(RZERO_ADDR, fRZero);
  
      if (fRZero == 0)
      {
        SPrintln("Warning: MQ135 calibration data missing, resetting calibration status.");
        EEPROM.put(RZERO_SET_ADDR, 0); // reset flag
      }    
      
    }
    else
    {
      SPrintln("Warning: MQ135 requires calibration.");
    }
  }

  pinMode(CARD_LEDS, OUTPUT);
  pinMode(CARD_DETECT, INPUT);
  pinMode(CARD_WP, INPUT);

  // check the SD card slot for a card, if one is found then try to make it writable to log data:
  //loggingConfig(&card, &fs);

  uint32_t testTime = (uint32_t)(millis() - startTime);
  SPrint("Self tested completed in "); Serial.print(testTime); SPrint(" ms.");

}

void loop() {
  // put your main code here, to run repeatedly:

}

/*
 * read all sensors and store their values into the data structure
 */
void readSensorData(sensorData *data, sensorIntegrity *sensorStats)
{
  // read temperature and pressure:
  sensorStats->BMP180_OK = readPressure(&data->Pressure, &data->Temp);

  // read humidity:
  // if reading temperature from the BMP180 failed, then use the temperature reading of the DHT-11 instead
  float fTemp;
  if (!sensorStats->BMP180_OK)
    sensorStats->DHT11_OK = readHumidity(&data->Humidity, &data->Temp);
  else
    sensorStats->DHT11_OK = readHumidity(&data->Humidity, &fTemp);
    
  // read light:
  sensorStats->Si1145_OK = readLight(&data->Illuminance, &data->UV, &data->IR);

  // read CO2:
  sensorStats->MQ135_OK = readCO2ppm(&data->CO2, data->Temp, data->Humidity);
}

/*
 * read humidity and temperature data from the DHT-11 sensor
 */
bool readHumidity(float *humidity, float *temp)
{
    // read humidity and temperature from the DHT-11 sensor:
    *humidity = xDHT11.readHumidity();
    *temp = xDHT11.readTemperature();

    // if either value is not a number (NaN), return false
    return !(isnan(*humidity) || isnan(*temp));
}

/*
 * read light sensor data from the Si1145
 */
bool readLight(float *lux, float *uv, float *ir)
{
    if (!xSi1145.integrityCheck())
      return false;
      
    *lux = xSi1145.readVisible();
    *uv = xSi1145.readIR();
    *ir = xSi1145.readUV();
    return true;
}

/*
 * read pressure and temperature data from BMP180
 */
bool readPressure(float *pressure, float *temp)
{
  if (!xBMP180.integrityCheck())
    return false;
      
  *temp = xBMP180.readTemperature();
  *pressure = xBMP180.readPressure();
  return true;
}

/*
 * read CO2 concentration in the air from MQ135
 */
bool readCO2ppm(float *CO2ppm, float temp, float RH)
{
  if (!xMQ135.integrityCheck())
    return false;

  *CO2ppm = xMQ135.getCorrectedPPM(temp, RH);
  //Serial.print("temp: "); Serial.print(temp); Serial.print("; humidity: "); Serial.println(RH);
    
  return true;
}
