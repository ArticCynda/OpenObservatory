/**************************************************************************/
/*!
@file     MQ135.cpp
@author   G.Krocker (Mad Frog Labs)
@license  GNU GPLv3

First version of an Arduino Library for the MQ135 gas sensor
TODO: Review the correction factor calculation. This currently relies on
the datasheet but the information there seems to be wrong.

@section  HISTORY

v2.0 - Added calibration routines, parametrized circuit, added pin configuration

v1.0 - First release
*/
/**************************************************************************/

#include "./mq135.h"

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

/**************************************************************************/
/*!
@brief  Default constructor

@param[in] pin  The analog input pin for the readout of the sensor
@param[in] Vc   The supply voltage of the sensor, default 5V
@param[in] Vref The ADC reference voltage on the Arduino, default 5V
*/
/**************************************************************************/
MQ135::MQ135(uint8_t pin, float Vc, float Vref)
{
    _pin = pin;
    _Vc = Vc;
    _Vref = Vref;
    _Rzero = RZERO;
    _AtmoCO2 = ATMOCO2;
}

MQ135::MQ135(uint8_t pin) {
  // call constructor with default supply voltage for MQ135 and ADC ref (5V)
  MQ135(pin, 5., 5.);
}

/**************************************************************************/
/*!
@brief  Initialize the sensor
*/
/**************************************************************************/
void MQ135::begin(void)
{
  pinMode(_pin, INPUT);
}

/**************************************************************************/
/*!
@brief  Fetch a default value (constant) for Rzero

@return A default value for Rzero
*/
/**************************************************************************/
float MQ135::getDefaultRzero()
{
    return RZERO;
}

/**************************************************************************/
/*!
@brief  Set the reference resistance at 20°c, 35% RH, 400 ppm CO2

@param[in] Rzero  The resistance of the sensor at 20°C, 35% RH, 400 ppm CO2
*/
/**************************************************************************/
void MQ135::setRZero(float Rzero)
{
    if (Rzero <= 0) // negative resistance values not possible!
        _Rzero = RZERO; // default value
    else
        _Rzero = Rzero;
}

/**************************************************************************/
/*!
@brief  Set the current atmospheric CO2 concentration for calibration

@param[in] CO2ppm The atmospheric CO2 concentration
*/
/**************************************************************************/
void MQ135::setAtmosphericCO2ppm(float CO2ppm)
{
    if (CO2ppm <= 0)
        _AtmoCO2 = ATMOCO2; // default value
    else
        _AtmoCO2 = CO2ppm;
}

/**************************************************************************/
/*!
@brief  Set the load resistance in the voltage divider

@param[in] RLoad  The load resistance in Ohms
*/
/**************************************************************************/
void MQ135::setRLoad(float RLoad)
{
    if (RLoad <= 0)
        _Rload = RLOAD;
    else
        _Rload = RLoad;

}

/**************************************************************************/
/*!
@brief  Get the correction factor to correct for temperature and humidity

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The calculated correction factor
*/
/**************************************************************************/
float MQ135::getCorrectionFactor(float t, float h) {
  return CORA * t * t - CORB * t + CORC - (h-33.)*CORD;
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value

@return The sensor resistance in kOhm
*/
/**************************************************************************/
float MQ135::getResistance() {
  int val = analogRead(_pin);
  //return ((1023./(float)val) * 5. - 1.)*RLOAD;
  float res = ((_Vc * 1024.)/(val * _Vref) - 1) * _Rload / 1000;
  return res;
}

/**************************************************************************/
/*!
@brief  Checks if the sensor reading is within expected boundaries

@return A value indicating if the sensor is working properly
*/
/**************************************************************************/
bool MQ135::integrityCheck()
{
  // according to datasheet, the value for Rs/R0 must be between 0.2 and 4
  float div = (float)getResistance() * 1000. / (float)_Rzero;
  return (div < MAXRSRO && div > MINRSRO);
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance kOhm
*/
/**************************************************************************/
float MQ135::getCorrectedResistance(float t, float h) {
  return getResistance()/getCorrectionFactor(t, h);
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air)

@return The ppm of CO2 in the air
*/
/**************************************************************************/
float MQ135::getPPM() {
  return PARA * pow((getResistance()/(_Rzero / 1000.)), -PARB);
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air), corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The ppm of CO2 in the air
*/
/**************************************************************************/
float MQ135::getCorrectedPPM(float t, float h) {
  return PARA * pow((getCorrectedResistance(t, h)/(_Rzero/1000.)), -PARB);
}

/**************************************************************************/
/*!
@brief  Get the resistance RZero of the sensor for calibration purposes

@return The sensor resistance RZero in kOhm
*/
/**************************************************************************/
float MQ135::getRZero() {
  return getResistance() * pow((ATMOCO2/PARA), (1./PARB));
}

/**************************************************************************/
/*!
@brief  Get the corrected resistance RZero of the sensor for calibration
        purposes

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance RZero in kOhm
*/
/**************************************************************************/
float MQ135::getCorrectedRZero(float t, float h) {
  return getCorrectedResistance(t, h) * pow((ATMOCO2/PARA), (1./PARB));
}
