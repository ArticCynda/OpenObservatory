/**************************************************************************/
/*!
@file     MQ135.h
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
#ifndef MQ135_H
#define MQ135_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

/// The load resistance on the board
#define RLOAD 10000
/// Calibration resistance at atmospheric CO2 level
#define RZERO 76630 // value from G. Krocker
/// Parameters for calculating ppm of CO2 from sensor resistance
#define PARA 116.6020682
#define PARB 2.769034857

// Boundary values for CO2, from David Gironi
#define MAXRSRO 2.428 //for CO2
#define MINRSRO 0.358 //for CO2

/// Parameters to model temperature and humidity dependence
#define CORA 0.00035
#define CORB 0.02718
#define CORC 1.39538
#define CORD 0.0018

/// Atmospheric CO2 level for calibration purposes
#define ATMOCO2 397.13

class MQ135 {
 private:
  uint8_t _pin;
  float _Vc;          // supply voltage of the MQ135 sensor, default is 5V
  float _Vref;        // ADC reference voltage of Arduino, default is 5V
  float _Rzero;       // resistance of the MQ135 at 20°C, 35% RH at 397.13 ppm (atmospheric CO2 concentration)
  float _AtmoCO2;     // CO2 concentration in the atmosphere, for callibration purposes
  float _Rload;       // load resistance in the circuit

 public:
  MQ135(uint8_t pin);
  MQ135(uint8_t pin, float Vc, float Vref);
  void begin(void);
  float getDefaultRzero(void);
  void setRZero(float RZero);
  void setRLoad(float RLoad);
  void setAtmosphericCO2ppm(float CO2ppm);
  bool integrityCheck();
  float getCorrectionFactor(float t, float h);
  float getResistance(void);
  float getCorrectedResistance(float t, float h);
  float getPPM();
  float getCorrectedPPM(float t, float h);
  float getRZero();
  float getCorrectedRZero(float t, float h);
};
#endif
