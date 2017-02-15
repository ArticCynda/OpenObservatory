// OpenObservatory Rev. 3 board definition
#include "O2_Rev3.h"

// Arduino constructs
#include "Arduino.h"

// use EEPROM on chip to store calibration values for sensors
#include <EEPROM.h>

#include <avr/wdt.h>

// include sensor drivers
#include "bmp180/bmp180.h"
#include "dht11/dht.h"
#include "si1145/si1145.h"
#include "mq135/mq135.h"

// SD card access
//#include "sdcard/sdcardutils.c"

#include "sdcard/sdcardutils.h"
#include "sdcard/Fat16util.h"
#include "sdcard/SdCard.h"
#include "sdcard/Fat16.h"
#include "sdcard/SdInfo.h"
#include "sdcard/Fat16Config.h"
#include "sdcard/Fat16mainpage.h"


//#define RZERO_SET_ADDR 0 // a flag indicating whether the R zero has been calibrated
//#define RZERO_ADDR 1  // calibration value for R zero
//#define RZERO_FLAG 0xAA // a flag to set

//#define SYNC_INT 2000

//#define DEBUG

#ifdef DEBUG
  #define DPrint(...) { Serial.print(__VA_ARGS__); }
  #define DPrintln(...) { Serial.println(__VA_ARGS__); }
#else
  #define DPrint(...) {}
  #define DPrintln(...) {}
#endif

//#define SPrint(...) { Serial.print(F(__VA_ARGS__)); }
//#define SPrintln(...) { Serial.println(F(__VA_ARGS__)); }


DHT xDHT11(O2_RH_SENSE, DHT11);

MQ135 xMQ135 = MQ135(O2_CO2_SENSE);
//float fRZero = 76.63; // datasheet reference value
float fRZero = 184.16;

SI1145 xSi1145 = SI1145();

//BMP180 bmp;
BMP180 xBMP180;

SdCard card;
Fat16 fs;



//bool ledState;
//bool cardInserted;
//bool cardProtected;

//bool logging = false;
//bool initRequired = true; // records if initialization was attempted but failed

/*
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
*/

//sensorData sensors;
//sensorIntegrity sensorStatus;

void watchdogSetup(void)
{
  cli(); // disables interrupts temporarily to ensure WDT setup is uninterrupted.

  wdt_reset();

  /*
  WDTCSR configuration:
  WDIE = 1: Interrupt Enable
  WDE = 1 :Reset Enable
  See table for time-out variations:
  WDP3 = 0 :For 1000ms Time-out
  WDP2 = 1 :For 1000ms Time-out
  WDP1 = 1 :For 1000ms Time-out
  WDP0 = 0 :For 1000ms Time-out
  */
  
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) |   (0<<WDP0);
  
  sei(); // re-enable interrupts

  DPrintln("Watchdog enabled, time-out 1000ms");
}


//uint64_t startTime;
#define PROGR_ADDR 0
#define RETRY_ADDR sizeof(uint8_t)
#define I2C_FLAG_ADDR  2 * sizeof(uint8_t)
#define MEMTEST_ADDR 3 * sizeof(uint8_t)

#define s_INIT      0
#define s_MEM       1
#define s_POWER     2
#define s_DHT11     3
#define s_TEMP      4
#define s_BMP180    5
#define s_MQ135     6
#define s_SI1145    7
#define s_LED       8
#define s_SD        9
#define s_FINISHED  10

void printEEPROM()
{
  Serial.println("EEPROM contents:");
  for (int i = 0; i < 3; i++)
  {
    Serial.print(EEPROM.read(i)); Serial.print(" ");
  }
  Serial.println();
}

/*
 * Attempts to initialize an SD card with a FAT16 FS, and create a new file to write data to.
 * Up to 10 files will be created following the naming pattern weather0.csv - weather9.csv
 */
bool initCard(SdCard *sd, Fat16 *file)
{
  //if (cardProtected())
  //{
  //    DPrintln(F("Card is protected against writing. Remove it, flip its WP switch, reinsert it and reboot the station."));
  //}
  //else
  //{
    if (!sd->begin(O2_SD_SS))
    {
      DPrintln(F("Problem initializing SD card."));
    }
    else
    {
      DPrintln(F("SD card intialized successfully."));
  
      if (!Fat16::init(sd))
      {
        DPrintln(F("Unable to initialize the FAT16 file system on the card."));
      }
      else
      {
        DPrintln(F("FAT16 file system initialized successfully."));
        
        // create a new file
        char name[] = "weather0.csv";
        for (uint8_t i = 0; i < 10; i++)
        {
          name[7] = i + '0';
          // O_CREAT - create the file if it does not exist
          // O_EXCL - fail if the file exists
          // O_WRITE - open for write only
          if (file->open(name, O_CREAT | O_EXCL | O_WRITE))
            break;
        }
        
        if (!file->isOpen())
        {
          DPrintln(F("Failed to open file on the card."));
        }
        else
        {
          DPrint(F("Data will be saved to file ")); DPrint(name); DPrintln(".");
          //setLEDstate(OK);
          return true;
        }
      }
    //}    
  }
  //setLEDstate(ERR);
  return false;
}

void setup() {
  // put your setup code here, to run once:



  //EEPROM.put(PROGRESS_FLAG_ADDR, 0);
  //SPrint("Debug: flag: "); Serial.print(EEPROM.get(PROGRESS_FLAG_ADDR), DEC); SPrint(" retries: "); Serial.print(EEPROM.get(PROGRESS_RETRIES_ADDR), DEC);

      // intialize serial connection for reporting:
    Serial.begin(SERIAL_SPEED);
    DPrintln("starting up");

    #ifdef DEBUG
      printEEPROM();
    #endif

  // set up the watchdog
  watchdogSetup();

  uint8_t iProgress, iTries;
  EEPROM.get(PROGR_ADDR, iProgress);
  if (iProgress > s_FINISHED) // invalid state
  {
    iProgress = 0;
    EEPROM.put(PROGR_ADDR, (uint8_t)s_INIT);
  }

  EEPROM.get(RETRY_ADDR, iTries);
  DPrint("Start Progress: "); DPrint(iProgress); DPrint(" Tries: "); DPrintln(iTries);

  if (iProgress == 0)
  {

    SPrintln("\n\nStarted OpenObservatory Rev. 3 self test.");
    SPrintln("Position the board on a flat surface, keep obstacles away from sensors, and insert 4 GB or smaller, FAT32 formatted SD card in the card slot.\n");

    DPrint("Progress: "); DPrint(iProgress); DPrint(" Tries: "); DPrintln(iTries);

    // clear I2C error flag
    EEPROM.put(I2C_FLAG_ADDR, (uint8_t)0);

    // move to next stage:
    iProgress = 1;
    EEPROM.put(PROGR_ADDR, iProgress);
    // reset the retry counter
    iTries = 0;
    EEPROM.put(RETRY_ADDR, iTries);

    DPrint("Progress 0: "); DPrint(iProgress); DPrint(" Tries: "); DPrintln(iTries);

    SPrintln("BIOS up & running.");
    SPrint("Checking memory... ");
  }

  // STAGE MEMORY
  while (iProgress == s_MEM && iTries++ < 5)
  {
    EEPROM.put(RETRY_ADDR, iTries);
    DPrint("Progress 1: "); DPrint(iProgress); DPrint(" Tries: "); DPrintln(iTries);
          
    //uint8_t iTestAddr = RETRY_ADDR + sizeof(uint8_t);
    uint8_t iTestValue = 66;
    // write test value to EEPROM:
    EEPROM.put(MEMTEST_ADDR, iTestValue);
    // check if the value is there:
    uint8_t iReadValue;
    EEPROM.get(MEMTEST_ADDR, iReadValue);

    // now clear the remaining EEPROM
    bool memtest = true;
    
    for (int i = MEMTEST_ADDR; i < EEPROM.length(); i++)
    {
      EEPROM.write(i, uint8_t(0));
      wdt_reset();
    }
    uint8_t sum = 0;
    for (int i = MEMTEST_ADDR; i < EEPROM.length(); i++)
    {
      uint8_t val;
      EEPROM.get(i, val);
      sum += val;
      EEPROM.write(i, uint8_t(1));
      wdt_reset();
    }
    memtest = (sum == 0);
    uint8_t prod = 1;
    for (int i = MEMTEST_ADDR; i < EEPROM.length(); i++)
    {
      uint8_t val;
      EEPROM.get(i, val);
      sum &= val;
      wdt_reset();
    }
    memtest &= (prod == 1);
    
    

    //delay(1500);
    //iReadValue = 12;

    if (iReadValue == iTestValue && memtest)
    {
      // test succeeded, move to next stage
      SPrintln("OK.");
      iProgress = s_POWER;
      EEPROM.put(PROGR_ADDR, iProgress);
      // reset the retry counter
      iTries = 0;
      EEPROM.put(RETRY_ADDR, iTries);
      wdt_reset();

      DPrint("Progress: "); DPrint(iProgress); DPrint(" Tries: "); DPrintln(iTries);

      SPrint("Checking power supply... ");
    }
    else
    {
      DPrint("Failed Progress: "); DPrint(iProgress); DPrint(" Tries: "); DPrintln(iTries);
          
      if (iTries == 5)
      {
        SPrintln("Failed.");
        SPrintln("Unrecoverable memory read/write error detected. Self test cannot continue.");
        // reset test status
        EEPROM.put(PROGR_ADDR, (uint8_t)0);
        EEPROM.put(RETRY_ADDR, (uint8_t)0);
        while (1)
          wdt_reset();
      }
    }
  }
  if (iProgress == s_MEM && iTries >= 5)
  {
         SPrintln("Failed.");
        SPrintln("Unrecoverable memory read/write error detected. Self test cannot continue.");
                EEPROM.put(PROGR_ADDR, (uint8_t)0);
        EEPROM.put(RETRY_ADDR, (uint8_t)0);
        while (1)
          wdt_reset();
  }

  // STAGE POWER
  while (iProgress == s_POWER && iTries++ < 5)
  {
    EEPROM.put(RETRY_ADDR, iTries);

    // measure voltage real using internal ADC
    long result; // Read 1.1V reference against AVcc 
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); 
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert 
    while (bit_is_set(ADCSRA,ADSC)); 
    result = ADCL; 
    result |= ADCH<<8; 
    result = 1126400L / result; // Back-calculate AVcc in mV return result;

    #ifdef DEBUG
      Serial.print("Voltage: "); Serial.println(result);
    #endif

    if (result < 3450 && result > 3150)
    {
        // test succeeded, move on to next stage
        SPrintln(" OK.");
        iProgress = s_DHT11;
        EEPROM.put(PROGR_ADDR, iProgress);
        // reset the retry counter
        iTries = 0;
        EEPROM.put(RETRY_ADDR, iTries);
        wdt_reset();

        SPrint("Checking humidity sensor... ");
    }
    if (iTries == 5)
    {
      if (result > 3450)
        SPrintln("Failed. Warning: supply voltage too high.")
      else 
       SPrintln("Failed. Supply voltage too low.");
       
      // move on to next stage
      iProgress = s_DHT11;
      EEPROM.put(PROGR_ADDR, iProgress);
      // reset the retry counter
      iTries = 0;
      EEPROM.put(RETRY_ADDR, iTries);
      wdt_reset();

      SPrint("Checking humidity sensor... ");
    }
  }
  if (iProgress == s_POWER && iTries >= 5)
  {
    SPrintln("Failed. CO2 sensor heater failure.");

    // move on to next stage
    iProgress = s_DHT11;
    EEPROM.put(PROGR_ADDR, iProgress);
    // reset the retry counter
    iTries = 0;
    EEPROM.put(RETRY_ADDR, iTries);
    wdt_reset();

    SPrint("Checking humidity sensor... ");
  }

  // STAGE DHT11
  float fHumidity, fTemp;
  while (iProgress == s_DHT11 && iTries++ < 5)
  {
    EEPROM.put(RETRY_ADDR, iTries);

    // try to initialize the sensor:
    xDHT11.begin();
    if (!readHumidity(&fHumidity, &fTemp))
    {
      wdt_reset();
      if (iTries == 5)
      {
        SPrintln("Failed. DHT11 humidity sensor not responding.");

        // move on to next stage
        iProgress = s_BMP180;
        EEPROM.put(PROGR_ADDR, iProgress);
        // reset the retry counter
        iTries = 0;
        EEPROM.put(RETRY_ADDR, iTries);
        wdt_reset();

        // initialize temp and humidity with default values
        fTemp = 22;
        fHumidity = 35;

        SPrint("Checking temperature sensor... ");
      }
    }
    else
    {
      SPrintln("OK.");
      if (fHumidity > 90.0 || fHumidity < 10.0)
        SPrintln("Warning: DHT11 humidity sensor may require calibration.");
        
      if (fTemp > 30.0 || fTemp < 10.0)
        SPrintln("Warning: DHT11 temperature sensor may require calibration.");

      // move on to next stage
      iProgress = s_TEMP;
      EEPROM.put(PROGR_ADDR, iProgress);
      // reset the retry counter
      iTries = 0;
      EEPROM.put(RETRY_ADDR, iTries);
      wdt_reset();

      SPrint("Checking temperature sensor... ");
    }
  }
  if (iProgress == s_DHT11 && iTries >= 5)
  {
        SPrintln("Failed. DHT11 humidity sensor not responding.");

        // move on to next stage
        iProgress = s_TEMP;
        EEPROM.put(PROGR_ADDR, iProgress);
        // reset the retry counter
        iTries = 0;
        EEPROM.put(RETRY_ADDR, iTries);
        wdt_reset();

        // initialize temp and humidity with default values
        fTemp = 22;
        fHumidity = 35;

        SPrint("Checking temperature sensor... ");
  }

  while (iProgress == s_TEMP && iTries++ < 5)
  {
    EEPROM.put(RETRY_ADDR, iTries);

    unsigned int wADC;
    double t;
  
    // The internal temperature has to be used
    // with the internal reference of 1.1V.
    // Channel 8 can not be selected with
    // the analogRead function yet.
  
    // Set the internal reference and mux.
    ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
    ADCSRA |= _BV(ADEN);  // enable the ADC
  
    delay(20);            // wait for voltages to become stable.
  
    ADCSRA |= _BV(ADSC);  // Start the ADC
  
    // Detect end-of-conversion
    while (bit_is_set(ADCSRA,ADSC));
  
    // Reading register "ADCW" takes care of how to read ADCL and ADCH.
    wADC = ADCW;
  
    // The offset of 324.31 could be wrong. It is just an indication.
    t = (wADC - 324.31 ) / 1.22;

    #ifdef DEBUG
      Serial.print("\nTemperature: "); Serial.println(t);
    #endif

    // temperature sensor ok
    SPrintln("OK.");
    if (t < 0 || t > 40)
      SPrintln("Warning: internal temperature sensor may require calibration.");

    // move on to next stage
    iProgress = s_BMP180;
    EEPROM.put(PROGR_ADDR, iProgress);
    // reset the retry counter
    iTries = 0;
    EEPROM.put(RETRY_ADDR, iTries);
    wdt_reset();

    SPrint("Checking pressure sensor... ");
  }
  if (iProgress == s_TEMP && iTries >= 5)
  {
    SPrintln("Failed. Temperature sensor not responding.");

    // move on to next stage
    iProgress = s_BMP180;
    EEPROM.put(PROGR_ADDR, iProgress);
    // reset the retry counter
    iTries = 0;
    EEPROM.put(RETRY_ADDR, iTries);
    wdt_reset();

    SPrint("Checking pressure sensor... ");
  }


  DPrint("Progress BMP180: "); DPrint(iProgress); DPrint(" Tries: "); DPrintln(iTries);

  float fPressure;
  /*
      xBMP180.begin();
      
    if (!readPressure(&fPressure, &fTemp))
    {
            SPrintln("Failed. ");
    }

    else
      SPrintln("Done. ");
      */

  //bool I2Cfailure = true;

  // STAGE BMP180

  while (iProgress == s_BMP180 && iTries++ < 5)
  {
    EEPROM.put(RETRY_ADDR, iTries);
    DPrint("Progress BMP180 inside: "); DPrint(iProgress); DPrint(" Tries: "); DPrintln(iTries);

    // try to initialize the sensor:
    xBMP180.begin();
    if (!readPressure(&fPressure, &fTemp))
    {
      if (iTries == 5)
      {
        SPrintln("Failed. BMP180 barometer not responding.");

        // move on to next stage
        iProgress = s_MQ135;
        EEPROM.put(PROGR_ADDR, iProgress);
        // reset the retry counter
        iTries = 0;
        EEPROM.put(RETRY_ADDR, iTries);
        wdt_reset();

        SPrint("Checking CO2 sensor... ");
      }

    }
    else
    {
      SPrintln("OK.");
      // clear I2C failure flag
      EEPROM.put(I2C_FLAG_ADDR, 1);

      #ifdef DEBUG
        Serial.print("Temp: "); Serial.println(fTemp);
        Serial.print("Pressure: "); Serial.println(fPressure);
      #endif
      
      if (fPressure > 110000.0 || fPressure < 80000.0)
        SPrintln("Warning: BMP180 pressure sensor may require calibration.");
  
      if (fTemp > 30 || fTemp < 10)
        SPrintln("Warning: BMP180 temperature sensor may require calibration.");
        
      // move on to next stage
      iProgress = s_MQ135;
      EEPROM.put(PROGR_ADDR, iProgress);
      // reset the retry counter
      iTries = 0;
      EEPROM.put(RETRY_ADDR, iTries);
      wdt_reset();

      SPrint("Checking CO2 sensor... ");
    }
  }
if (iProgress == s_BMP180 && iTries >= 5)
{
  SPrintln("Failed. BMP180 barometer not responding.");

  // move on to next stage
  iProgress = s_MQ135;
  EEPROM.put(PROGR_ADDR, iProgress);
  // reset the retry counter
  iTries = 0;
  EEPROM.put(RETRY_ADDR, iTries);
  wdt_reset();

  SPrint("Checking CO2 sensor... ");
 }


  // STAGE MQ135
  while (iProgress == s_MQ135 && iTries++ < 5)
  {
    EEPROM.put(RETRY_ADDR, iTries);

    // turn on heater & wait 250ms
    pinMode(O2_CO2_HEATER, OUTPUT);
    digitalWrite(O2_CO2_HEATER, HIGH);
    wdt_reset();
    
    pinMode(O2_CO2_SENSE, INPUT);

    // try to initialize the sensor:
    //xMQ135.begin();

    float fCO2 = xMQ135.getCorrectedPPM(fTemp, fHumidity);

    #ifdef DEBUG
      uint16_t adcval = analogRead(O2_CO2_SENSE);
      Serial.print("\nADC value: "); Serial.println(adcval, DEC);
      float res = ((5 * 1024.)/(adcval * 3.3) - 1) * 10000 / 1000;
      Serial.print("res: "); Serial.println(res, DEC);
      Serial.print("PPM: "); Serial.println(fCO2, DEC);
    #endif    

    uint16_t adcvalue = analogRead(O2_CO2_SENSE);

    //if (readCO2ppm(&fCO2, fPressure, fHumidity))
    
    if (adcvalue > 25 && adcvalue < 1000)
    {
      // test succeeded, move on to next stage
      SPrintln("OK.");

      #ifdef DEBUG
      if (fCO2 < 400 || fCO2 > 2000)
        SPrintln("Warning: MQ135 pressure sensor may require calibration.");
      #endif

      // move to the next stage
      iProgress = s_SI1145;
      EEPROM.put(PROGR_ADDR, iProgress);
      // reset the retry counter
      iTries = 0;
      EEPROM.put(RETRY_ADDR, iTries);
      wdt_reset();

      SPrint("Checking light and UV sensors... ");      
    }
    else
    {
      if (iTries == 5)
      {
        // test failed
        SPrintln("Failed. MQ-135 sensor not responding.")

      iProgress = s_SI1145;
      EEPROM.put(PROGR_ADDR, iProgress);
      // reset the retry counter
      iTries = 0;
      EEPROM.put(RETRY_ADDR, iTries);
      wdt_reset();

      SPrint("Checking light and UV sensors... ");     
      }
    }
  }
  if (iProgress == s_MQ135 && iTries >= 5)
{
        SPrintln("Failed. MQ-135 sensor not responding.")

      iProgress = s_SI1145;
      EEPROM.put(PROGR_ADDR, iProgress);
      // reset the retry counter
      iTries = 0;
      EEPROM.put(RETRY_ADDR, iTries);
      wdt_reset();

      SPrint("Checking light and UV sensors... ");    
 }


  // STAGE SI1145
  while (iProgress == s_SI1145 && iTries++ < 5)
  {
    EEPROM.put(RETRY_ADDR, iTries);

    xSi1145.begin();
    float fLux, fUV, fIR;
    if (!readLight(&fLux, &fUV, &fIR))
    {
      if (iTries == 5)
      {
       SPrintln("Failed. SI1145 sensor not responding.")

      iProgress = s_LED;
      EEPROM.put(PROGR_ADDR, iProgress);
      // reset the retry counter
      iTries = 0;
      EEPROM.put(RETRY_ADDR, iTries);
      wdt_reset();

      SPrint("Checking peripherals... ");  
      }
    }
    else
    {
      SPrintln("OK.")
      // clear I2C failure flag
      EEPROM.put(I2C_FLAG_ADDR, 1);

      iProgress = s_LED;
      EEPROM.put(PROGR_ADDR, iProgress);
      // reset the retry counter
      iTries = 0;
      EEPROM.put(RETRY_ADDR, iTries);
      wdt_reset();

      SPrint("Checking peripherals... ");   
    }


  }
  if (iProgress == s_SI1145 && iTries >= 5)
  {
      SPrintln("Failed. SI1145 sensor not responding.")

      iProgress = s_LED;
      EEPROM.put(PROGR_ADDR, iProgress);
      // reset the retry counter
      iTries = 0;
      EEPROM.put(RETRY_ADDR, iTries);
      wdt_reset();

      SPrint("Checking peripherals... ");   
  }

// STAGE LEDs
while (iProgress == s_LED && iTries++ < 5)
{
  EEPROM.put(RETRY_ADDR, iTries);

  pinMode(O2_USB_CONFIG, INPUT);
  pinMode(O2_USB_SUSPEND, INPUT);
  pinMode(O2_SD_LED, INPUT);
  
  #ifdef DEBUG
    Serial.print("\nUSB CONFIG: "); Serial.println(digitalRead(O2_USB_CONFIG));
    Serial.print("USB SUSPEND: "); Serial.println(digitalRead(O2_USB_SUSPEND));
    Serial.print("LED: "); Serial.println(digitalRead(O2_SD_LED));
  #endif DEBUG

  bool USB_CONF = digitalRead(O2_USB_CONFIG);
  bool USB_SUSP = digitalRead(O2_USB_SUSPEND);
  bool LED_STATE = digitalRead(O2_SD_LED);

  pinMode(O2_SD_LED, OUTPUT);
  for (int i = 0; i < 11; i++)
  {
    digitalWrite(O2_SD_LED, !digitalRead(O2_SD_LED));
    delay(125);
    wdt_reset();
  }


  if (USB_CONF && !USB_SUSP && LED_STATE)
  {
    SPrintln("OK.");

    iProgress = s_SD;
    EEPROM.put(PROGR_ADDR, iProgress);
    // reset the retry counter
    iTries = 0;
    EEPROM.put(RETRY_ADDR, iTries);
    wdt_reset();

    SPrint("Checking SD card... ");
  }
  else
  {
    if (iTries == 5)
    {
          // strange configuration detected
    if (!USB_CONF || USB_SUSP)
      SPrintln("Failed. USB connection does not initialize correctly.");

    iProgress = s_SD;
    EEPROM.put(PROGR_ADDR, iProgress);
    // reset the retry counter
    iTries = 0;
    EEPROM.put(RETRY_ADDR, iTries);
    wdt_reset();

    SPrint("Checking SD card... ");
    }
  }

//    #define O2_USB_CONFIG     2      // INPUT   USB initialization complete
//  #define O2_USB_SUSPEND    3      // INPUT   USB communication suspended by host
//    #define O2_SD_LED         9      // OUTPUT  SD card state indicator LEDs
}
if (iProgress == s_LED && iTries >= 5)
{
    SPrintln("Failed. Unable to read I/O.");

    iProgress = s_SD;
    EEPROM.put(PROGR_ADDR, iProgress);
    // reset the retry counter
    iTries = 0;
    EEPROM.put(RETRY_ADDR, iTries);
    wdt_reset();

    SPrint("Checking SD card... ");
}

// STAGE SD CARD
while (iProgress == s_SD && iTries++ < 5)
{
    EEPROM.put(RETRY_ADDR, iTries);

      pinMode(O2_USB_CONFIG, INPUT);
  pinMode(O2_USB_SUSPEND, INPUT);

  #ifdef DEBUG
    Serial.print("\nCARD DETECT: "); Serial.println(digitalRead(O2_SD_DETECT));
    Serial.print("CARD WRITE PROTECTED: "); Serial.println(digitalRead(O2_SD_WP));
  #endif DEBUG

  bool cardInstalled = !digitalRead(O2_SD_DETECT);
  bool cardProtected = digitalRead(O2_SD_WP);

  // check if a card is present
  if (cardInstalled)
  {
    // check if the card is read only
    if (!cardProtected)
    {
      if (!initCard(&card, &fs))
      {
        SPrintln("card init failed");
      }
      else
      {
        SPrintln("card init succeeded");
      }
      
    }
  }

  if (iTries == 5)
  {
    if (!cardInstalled)
    {
      SPrintln("Failed. No card installed.");
    }
    else if (cardProtected)
    {
      SPrintln("Failed. Card is read only.");
    }
    else
    {
      SPrintln("Failed. Unable to initialize card.");
    }

    iProgress = s_FINISHED;
    EEPROM.put(PROGR_ADDR, iProgress);
    iTries = 0;
    EEPROM.put(PROGR_ADDR, iTries);
    wdt_reset();
  }
}
if (iProgress == s_SD && iTries >= 5)
{
  SPrintln("Failed. Unable to initialize card.");

  iProgress = s_FINISHED;
  EEPROM.put(PROGR_ADDR, iProgress);
  iTries = 0;
  EEPROM.put(PROGR_ADDR, iTries);
  wdt_reset();
}



  // check for I2C issues
  uint8_t I2C_flag;
  EEPROM.get(I2C_FLAG_ADDR, I2C_flag);
  if (!I2C_flag)
    SPrintln("Couldn't connect to I2C sensors, check I2C bus.");
   
  EEPROM.put(I2C_FLAG_ADDR, s_INIT);
  EEPROM.put(MEMTEST_ADDR, s_INIT);
  EEPROM.put(PROGR_ADDR, s_INIT);
  EEPROM.put(RETRY_ADDR, s_INIT);
  SPrintln("Self test finished!");

  while (true)
    wdt_reset();
 

  //startTime = millis();
  



/*
  // initialize Si1145
  SPrint("Searching for Si1145... ");
  xSi1145.begin();
  float fLux, fUV, fIR;
  if (!readLight(&fLux, &fUV, &fIR))
  {
    SPrintln("\nHardware error: Si1145 light sensor not responsing.");
  }
  else
  {
    SPrintln("Found Si1145.");
    // TODO: add calibration check for Si1145 here
  }
  */

/*
  // initialize BMP180
  xBMP180.begin();
  float fPressure, fTemp, fHumidity;
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
  */

/*
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
  */



  // check the SD card slot for a card, if one is found then try to make it writable to log data:
  //loggingConfig(&card, &fs);

  //uint32_t testTime = (uint32_t)(millis() - startTime);
  //SPrint("Self tested completed in "); Serial.print(testTime); SPrint(" ms.");

}

void loop() {
  // put your main code here, to run repeatedly:

}

/*
 * read all sensors and store their values into the data structure
 */
/*void readSensorData(sensorData *data, sensorIntegrity *sensorStats)
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
*/

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
 /*
bool readCO2ppm(float *CO2ppm, float temp, float RH)
{
  if (!xMQ135.integrityCheck())
    return false;

  *CO2ppm = xMQ135.getCorrectedPPM(temp, RH);
  //Serial.print("temp: "); Serial.print(temp); Serial.print("; humidity: "); Serial.println(RH);
    
  return true;
}
*/
