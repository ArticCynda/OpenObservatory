#include "O2_Rev3.h"

// Arduino constructs
#include "Arduino.h"

// use EEPROM on chip to store calibration values for sensors
#include <EEPROM.h>     

// include sensor drivers
#include "./bmp180/bmp180.h"
#include "./dht11/dht.h"
#include "./si1145/SI114X.h"
#include "./mq135/mq135.h"

//#include "./timer/TimerOne.h"
#include "./timer2/Timer2.h"

//#include "sdcard/sdcardutils.h"

//#define DEBUG

/*
 * Uncomment the following line to enable the low power mode for the station.
 * When the host suspends the device, it will turn off power hungry components and halt sending data.
 * WARNING: Ubuntu/Mint puts the device into suspended mode by default, making it look dead!
 *          Do NOT enable low power mode unless your application can switch the station to active mode!
 */
// #define ENABLE_USB_SUSPEND

// Timer 1 preloader: this configures the data update frequency. Default: 10 Hz.
static uint32_t iTimerInterval = 100000;

static bool bBootPhaseComplete = false; // a value indicating if the boot procedure was successfully completed.

#define PRI     0x1
#define SEC     0x2
#define AUX     0x3

#define HEATER_ON 0x01
#define HEATER_OFF 0x00
#define HEATER_AUTO 0x02

/*
struct sensorData
{
  float Temp;
  float Pressure;
  float Humidity;
  float HeatIndex;
  float DewPoint;
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

//static sensorData sensors;
//static sensorIntegrity sensorStatus;

uint8_t iTSensor = PRI;
uint8_t iHeater = HEATER_AUTO;

#ifdef DEBUG
  #define DPrint(...) { Serial.print(__VA_ARGS__); }
  #define DPrintln(...) { Serial.println(__VA_ARGS__); }
#else
  #define DPrint(...) {}
  #define DPrintln(...) {}
#endif

static BMP180    xBMP180;
static DHT       xDHT11(O2_RH_SENSE, DHT11);
static SI114X    xSi1145 = SI114X();
static MQ135     xMQ135 = MQ135(O2_CO2_SENSE, 5., 3.3);
//float fRZero = 76.63; // datasheet reference value
static float fRZero = 184.16;

void setup() 
{
  // initialize the serial connection
  Serial.begin(115200);

  // wait until the USB connection with the host is established before sending text:
  while (!digitalRead(O2_USB_CONFIG));
  
  Serial.println(F("\r\nStarting OpenObservatory v3.0"));
  Serial.println(F("Firmware revision 3.2.0, last updated January 2017."));

  // configure the microcontroller's IO pins
  // for a complete list of pins, see O2_Rev3.h
  pinMode(O2_SD_LED, OUTPUT);
  pinMode(O2_SD_DETECT, INPUT);
  pinMode(O2_SD_WP, INPUT);
  pinMode(O2_SD_SS, OUTPUT);

  pinMode(O2_CO2_HEATER, OUTPUT);
  pinMode(O2_CO2_SENSE, INPUT);

  pinMode(O2_USB_CONFIG, INPUT);
  pinMode(O2_USB_SUSPEND, INPUT);
  
  pinMode(O2_MCU_LED, OUTPUT);

  pinMode(O2_RH_SENSE, INPUT);

  // initializing DHT-11
  xDHT11.begin();



  if (!xBMP180.begin())
  {
    Serial.println(F("BMP180 hardware error!"));
  }


    
  // initialize Si1145
  if (!xSi1145.Begin(5))
  {
    Serial.println(F("Si1145 hardware error!"));
  }

      //Serial.println("Si1145 initialized");

  xSi1145.Init();

  xMQ135.begin();
  initMQ135();

  //Serial.println("Starting timers...");

  // configure timer for data transfer
  //noInterrupts();           // temporarily disable interrupts
  Timer2::set(500, TimerISR);
  Timer2::start();
  //interrupts();             // enable all interrupts

  Serial.println(F("OpenObservatory v3.0 now online!"));

  bBootPhaseComplete = true;

}

volatile bool iSampleFlag = 0;

void TimerISR()
{
  iSampleFlag = 1;
}

/*
 * Take a sample from each sensor and transmit its value to the pc.
 */
void SampleSensors()
{
  if (bBootPhaseComplete && iSampleFlag)
  {
    //readSensorData(&sensors, & sensorStatus);

    SPrint("\n\r"); // insert blank line

    double fRH = xDHT11.readHumidity();
    Serial.print(F("Humidity: ")); Serial.print(fRH, 0); Serial.println(" %");
    double fTemp;
    switch (iTSensor)
    {
      case PRI: fTemp = xSi1145.MeasureTemperature(); break;
      case SEC: fTemp = xBMP180.readTemperature(); break;
      case AUX: fTemp = xDHT11.readTemperature(); break;     
    }
    Serial.print(F("Temperature: ")); Serial.print(fTemp, 1); Serial.println(" deg C");   

    double fVIS = xSi1145.MeasureVIS();
    Serial.print(F("Visible light: ")); Serial.print(fVIS, 0); Serial.println(" lx");
    Serial.print(F("Infrared radiation: ")); Serial.print(xSi1145.MeasureIR(), 1); Serial.println(" mW/m^2");
    Serial.print(F("UV index: ")); Serial.println(xSi1145.MeasureUVIndex(), 2);

    Serial.print(F("CO2: ")); Serial.print(xMQ135.getCorrectedPPM(fTemp, fRH), 0); Serial.println(" ppm CO2");
    if (!digitalRead(O2_CO2_HEATER))
      Serial.println("Warning: heater turned off. Data may be inaccurate.");
    Serial.print(F("Pressure: ")); Serial.print((uint32_t)xBMP180.readPressure()); Serial.println(" Pa");

    //Serial.print(F("Dew point: ")); Serial.print(sensors.DewPoint + 293.15, 0); Serial.println(" K");
    //Serial.print(F("Heat index: ")); Serial.print(sensors.HeatIndex + 293.15, 0); Serial.println(" K");

    //digitalWrite(O2_MCU_LED, !digitalRead(O2_MCU_LED));

    iSampleFlag = 0;

  }
}

/*
 * Set MQ135 configuration values specific for the weater station board
 */
void initMQ135()
{
  xMQ135.setRLoad(47000/2);
  xMQ135.setRZero(50000); // experimental, comment this once calibration is in place!
}

/*
 * Calibrates the MQ-135 air quality sensor
 */
void calibrateMQ135()
{
  Serial.println(F("Calibrating MQ-135..."));
  Serial.println(F("Leave the weather station running outside, in approx. 20C at 35% humidity."));
  Serial.println(F("Calibration can take up to 30 minutes."));
  
  float fRZeroCal = 0;

  char progress[] = "[                    ]       %";
  uint8_t iProgress = 0;
  for (iProgress = 0; iProgress <= 100; iProgress++)
  {
    uint8_t iMarks = iProgress / 5;
    for (int i = 1; i < (iMarks + 1); i++)
      progress[i] = '*';

    if (iProgress < 100)
    {
      int iT = iProgress / 10;
      if (iT != 0)
        progress[25] = iT  + '0';
      progress[26] = (iProgress % 10) + '0';
      Serial.print("\r"); Serial.print(progress);
    }
    else
    {
      progress[24] = 1 + '0';
      progress[25] = 0 + '0';
      progress[26] = 0 + '0';
      Serial.print("\r"); Serial.println(progress);
    }

    delay(250);
  }

  // save calibration value to EEPROM and set a flag to indicate that the MQ-135 has been calibrated
  //EEPROM.put(RZERO_SET_ADDR, RZERO_FLAG);
  EEPROM.put(RZERO_ADDR, fRZeroCal);

  Serial.println(F("MQ-135 calibrated successfully"));
  
}

/*
 * Control loop listens to serial commands and adjusts operation of the station accordingly.
 * Unrecognized commands are dismissed after a period configurable with COMMAND_TIMEOUT (in ms).
 */
#define COMMAND_TIMEOUT 250
static String sCommand = "";
static uint64_t iLastCommandTime = 0;
  
void loop() {

  // sample sensors if necessary and make a transmission
  #ifdef ENABLE_USB_SUSPEND
    bool bSuspended = !digitalRead(O2_USB_SUSPEND);
    switch (iHeater)
    {
      case HEATER_ON: digitalWrite(O2_CO2_HEATER, HIGH); break;
      case HEATER_OFF: digitalWrite(O2_CO2_HEATER, LOW); break;
      case HEATER_AUTO: digitalWrite(O2_CO2_HEATER, !bSuspended); break; // turn off CO2 sensor in suspended mode to conserve power
    }
     
    if (!bSuspended)
      SampleSensors();
  #else
    digitalWrite(O2_CO2_HEATER, iHeater);
    SampleSensors();
  #endif

  // reset the command buffer if no recognized command is received within a timeout:
  if ((iLastCommandTime + COMMAND_TIMEOUT) < millis() && sCommand.length() > 0)
  {
    iLastCommandTime = 0;
    Serial.print("Unrecognized command: "); Serial.println(sCommand);
    sCommand = "";
  }

  // if any new bytes are in the RX buffer, add them to the existing command string
  if (Serial.available() > 0)
  {
    iLastCommandTime = millis();
    sCommand += (char)Serial.read();
  }

  // command to change the update interval of the sensors
  if (sCommand.substring(0, 10) == "set TXINT " && sCommand.endsWith("ms"))
  {
    //Serial.println("Command received: set transmission interval.");
    String sTimeOut = sCommand.substring(10, sCommand.length() - 3);

    Timer2::stop();
    Timer2::set(sTimeOut.toInt(), TimerISR);
    Timer2::start();
    
    sCommand = "";
    Serial.print("Transmission interval set to "); Serial.print(sTimeOut); Serial.println(" ms.");
  }

  if (sCommand.substring(0, 12) == "set TSENSOR " && sCommand.endsWith("ary"))
  {
    if (sCommand.endsWith("primary"))
    {
      Serial.println("Command received: use primary temp sensor");
      iTSensor = PRI;
    }
    else if (sCommand.endsWith("secondary"))
    {
      Serial.println("Command received: use secondary temp sensor");
      iTSensor = SEC;
    }
    else if (sCommand.endsWith("auxiliary"))
    {
      Serial.println("Command received: use auxiliary temp sensor");
      iTSensor = AUX;
    }
    sCommand = "";
  }

  if (sCommand == "set HEATER on")
  {
    iHeater = HEATER_ON;
    Serial.println("Heater turned on.");
    sCommand = "";
  }
  if (sCommand == "set HEATER off")
  {
    iHeater = HEATER_OFF;
    Serial.println("Heater turned off.");
    sCommand = "";
  }
  if (sCommand == "set HEATER auto")
  {
    iHeater = HEATER_AUTO;
    Serial.println("Heater controlled automatically.");
    sCommand = "";
  }

}

