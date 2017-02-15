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
#include "sdcard/sdcardutils.h"
#include "sdcard/Fat16util.h"
#include "sdcard/SdCard.h"
#include "sdcard/Fat16.h"
#include "sdcard/SdInfo.h"
#include "sdcard/Fat16Config.h"
#include "sdcard/Fat16mainpage.h"

/*
 * Debug flag. Uncomment this to enable verbose serial output.
 */
//#define DEBUG

#ifdef DEBUG
  #define DPrint(...) { Serial.print(__VA_ARGS__); }
  #define DPrintln(...) { Serial.println(__VA_ARGS__); }
#else
  #define DPrint(...) {}
  #define DPrintln(...) {}
#endif

DHT xDHT11(O2_RH_SENSE, DHT11);

MQ135 xMQ135 = MQ135(O2_CO2_SENSE);
float fRZero = 184.16;

SI1145 xSi1145 = SI1145();

BMP180 xBMP180;

SdCard card;
Fat16 fs;

/*
 * Configure watchdog timer to reset the board after 1000 ms.
 * Necessary for BMP180 and SI1145 in case of failure on the I2C bus.
 */
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

// addresses
#define PROGR_ADDR 0                        // test progress
#define RETRY_ADDR sizeof(uint8_t)          // number of consecutive attempts in the current test
#define I2C_FLAG_ADDR  2 * sizeof(uint8_t)  // a flag indicating if an I2C bus issue is likely
#define MEMTEST_ADDR 3 * sizeof(uint8_t)    // empty memory position for EEPROM stress testing

// list of tests
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

/*
 * Print the relevant EEPROM contents to serial terminal.
 * Only used for debugging.
 */
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
        return true;
      }
    } 
  }
  return false;
}

/*
 * Main routine, executing all tests sequentially
 * If the watchdog resets, because of a lock-up on I2C bus for example,
 * the microcontroller will restart, with setup() as entry point.
 */
void setup() {

  // intialize serial connection for reporting:
  Serial.begin(SERIAL_SPEED);
  DPrintln("starting up");

  #ifdef DEBUG
    // in debug mode, print the contents of the EEPROM to the serial output
    printEEPROM();
  #endif

  // set up the watchdog
  watchdogSetup();

  // check the state of the test progress and resume from the previous state if necessary
  uint8_t iProgress, iTries;
  EEPROM.get(PROGR_ADDR, iProgress);
  if (iProgress > s_FINISHED) // invalid state
  {
    iProgress = 0;
    EEPROM.put(PROGR_ADDR, (uint8_t)s_INIT);
  }
  EEPROM.get(RETRY_ADDR, iTries);
  DPrint("Start Progress: "); DPrint(iProgress); DPrint(" Tries: "); DPrintln(iTries);

  // test entry point, only executed once
  if (iProgress == 0)
  {
    // some basic information about the program
    SPrintln("\n\nStarted OpenObservatory Rev. 3 self test.");
    SPrintln("Position the board on a flat surface, keep obstacles away from sensors, and insert 4 GB or smaller, FAT32 formatted SD card in the card slot.");
    SPrintln("Warning: self test only checks board hardware, does not perform calibration!\n");

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
          
    uint8_t iTestValue = 66; // a random value to write to the EEPROM
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
    
    // to check the correct operation of the watchdog timer, uncomment the next instruction:
    //delay(1500);
    // to check correct operation of EEPROM memory check, uncomment the next instruction:
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
        // if after 5 attempts the EEPROM contents cannot be verified, stop the test here.
        // it cannot continue with faulty memory.
        SPrintln("Failed.");
        SPrintln("Unrecoverable memory read/write error detected. Self test cannot continue.");
        // reset test status
        EEPROM.put(PROGR_ADDR, (uint8_t)0);
        EEPROM.put(RETRY_ADDR, (uint8_t)0);
        // keep resetting watchdog indefnitely to prevent it from resetting the board after time-out
        while (1)
          wdt_reset();
      }
    }
  }
  // EEPROM test took too long and timed out 5 times, stop testing.
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

    // check if voltage rail is within acceptable limits
    if (result < 3450 && result > 3150)
    {
        // test succeeded, move on to next stage
        SPrintln("OK.");
        iProgress = s_DHT11;
        EEPROM.put(PROGR_ADDR, iProgress);
        // reset the retry counter
        iTries = 0;
        EEPROM.put(RETRY_ADDR, iTries);
        wdt_reset();

        SPrint("Checking humidity sensor... ");
    }
    else
    {
      // voltage rail out of range, give it some time to stabilize before retrying
      delay(250);
      wdt_reset();
    }
    if (iTries == 5)
    {
      // if the power rail doesn't stabilize, give a warming:
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
  // internal ADC timed out 5 times, skip
  if (iProgress == s_POWER && iTries >= 5)
  {
    SPrintln("Failed. Internal voltage rail monitor not responding.");

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
        // DHT-11 doesn't respond, give up after 5 attempts
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
      // test succeeded, DHT11 initialized correctly.
      SPrintln("OK.");

      // check if humidity and temperature are within credible limits:
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

  // STAGE TEMPERATURE
  while (iProgress == s_TEMP && iTries++ < 5)
  {
    EEPROM.put(RETRY_ADDR, iTries);

    /*
     * The following code uses the internal temperature sensor of the ATmega328AU as reference.
     */
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
  // internal ADC timed out, skip
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

  // first attempt at initializing I2C bus, report status before commencing in debug mode
  DPrint("Progress BMP180: "); DPrint(iProgress); DPrint(" Tries: "); DPrintln(iTries);

  // STAGE BMP180
  float fPressure;
  while (iProgress == s_BMP180 && iTries++ < 5)
  {
    EEPROM.put(RETRY_ADDR, iTries);
    DPrint("Progress BMP180 inside: "); DPrint(iProgress); DPrint(" Tries: "); DPrintln(iTries);

    // try to initialize the sensor:
    xBMP180.begin();
    if (!readPressure(&fPressure, &fTemp))
    {
      // BMP180 responded with wrong ID or didn't initialize correctly
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

      // check if values are within credible limits:
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

  // BMP180 connection timed out, possible issue with I2C bus or sensor missing
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

    // turn on heater & wait 500ms
    pinMode(O2_CO2_HEATER, OUTPUT);
    digitalWrite(O2_CO2_HEATER, HIGH);
    delay(500);
    wdt_reset();
    
    pinMode(O2_CO2_SENSE, INPUT);

    // try to initialize the sensor:
    float fCO2 = xMQ135.getCorrectedPPM(fTemp, fHumidity);

    #ifdef DEBUG
      uint16_t adcval = analogRead(O2_CO2_SENSE);
      Serial.print("\nADC value: "); Serial.println(adcval, DEC);
      float res = ((5 * 1024.)/(adcval * 3.3) - 1) * 10000 / 1000;
      Serial.print("res: "); Serial.println(res, DEC);
      Serial.print("PPM: "); Serial.println(fCO2, DEC);
    #endif    

    // read a value from the sensor
    uint16_t adcvalue = analogRead(O2_CO2_SENSE);   

    // check if it is within credible limits
    // > 1000 usually means sensor is stuck at 1
    // < 10 usually means sensor is stuck at 0
    // 10 - 360 usually means sensor is missing or poorly connected
    if (adcvalue > 360 && adcvalue < 1000)
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
      // sensor not connected, or short with power or ground
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
  // reading ADC timed out 5 times
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

    // try to initialize SI1145:
    xSi1145.begin();
    float fLux, fUV, fIR;
    if (!readLight(&fLux, &fUV, &fIR))
    {
      // sensor responded with wrong ID
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
      // sensor correctly intialized
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
  // sensor communication timed out, usually indicates sensor missing or I2C problem
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

  // configure USB status signals as inputs
  pinMode(O2_USB_CONFIG, INPUT);
  pinMode(O2_USB_SUSPEND, INPUT);
  pinMode(O2_SD_LED, INPUT);
  
  #ifdef DEBUG
    Serial.print("\nUSB CONFIG: "); Serial.println(digitalRead(O2_USB_CONFIG));
    Serial.print("USB SUSPEND: "); Serial.println(digitalRead(O2_USB_SUSPEND));
    Serial.print("LED: "); Serial.println(digitalRead(O2_SD_LED));
  #endif DEBUG

  // read status
  // USB should be active in this state (configured and not suspended)
  bool USB_CONF = digitalRead(O2_USB_CONFIG);
  bool USB_SUSP = digitalRead(O2_USB_SUSPEND);
  bool LED_STATE = digitalRead(O2_SD_LED);

  // flash the LED of the SD card, then turn it off again
  pinMode(O2_SD_LED, OUTPUT);
  for (int i = 0; i < 11; i++)
  {
    digitalWrite(O2_SD_LED, !digitalRead(O2_SD_LED));
    delay(125);
    wdt_reset();
  }
  pinMode(O2_SD_LED, INPUT);

  // check if USB signals are correct
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
}
// reading digital inputs timed out, usually indicates I/O issue
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

  // configure pins of card detection and write protection
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
  // card couldn't be detected after 5 attempts
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
// intializing card timed out, usually indicates faulty card or SPI bus problems
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

  // reset memory state
  EEPROM.put(I2C_FLAG_ADDR, s_INIT);
  EEPROM.put(MEMTEST_ADDR, s_INIT);
  EEPROM.put(PROGR_ADDR, s_INIT);
  EEPROM.put(RETRY_ADDR, s_INIT);
  SPrintln("Self test finished!");

  // keep the watchdog suspended to prevent automatic reset of the board
  while (true)
    wdt_reset();
 
} // end of void setup()

void loop() { }

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
