/*
 * SI114X.cpp
 * A library for Grove - Sunlight Sensor v1.0
 *
 * Copyright (c) 2015 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : Fuhua.Chen
 * Modified Time: June 2015
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SI114X.h"
#include "curves.h"
//#include "./cpoly.cpp"
#include "./poly34.cpp"
#include <Wire.h>
#include <math.h>
#include <Arduino.h>
/*--------------------------------------------------------//
default init

 */
void SI114X::Init(void)
{
  //ENABLE UV reading
  //these reg must be set to the fixed value
  WriteByte(SI114X_UCOEFF0, 0x7B);
  WriteByte(SI114X_UCOEFF1, 0x6B);
  WriteByte(SI114X_UCOEFF2, 0x01);
  WriteByte(SI114X_UCOEFF3, 0x00);

    //Serial.println("retrieving cal data");

  // calibrate the device:
  SI114X_CAL_S* cal_values;

  get_calibration( cal_values );
    //Serial.println("stop");
  set_ucoef( cal_values );

  //Serial.println("retrieved cal data");

  //WriteByte(SI114X_UCOEFF0, 0x29);
  //WriteByte(SI114X_UCOEFF1, 0x89);
  //WriteByte(SI114X_UCOEFF2, 0x02);
  //WriteByte(SI114X_UCOEFF3, 0x00);
  WriteParamData(SI114X_CHLIST, SI114X_CHLIST_ENUV | SI114X_CHLIST_ENALSIR | SI114X_CHLIST_ENALSVIS | SI114X_CHLIST_ENPS1 | SI114X_CHLIST_ENPS2 | SI114X_CHLIST_ENPS3);
  /*
   * AUX:   UV
   * IR:    Large IR (0x03)
   * VIS:   VIS
   * PS1:   Small IR (0x00)
   * PS2:   Temperature (0x65)
   * PS3:   Photo Reference (0x06)
   */

  // disable LEDs
  WriteParamData(SI114X_PSLED12_SELECT, 0x00);
  WriteParamData(SI114X_PSLED3_SELECT, 0x00);


  // by default, read the 16 most significant bits of the 17 bit ADC:
  WriteParamData(SI114X_ALS_ENCODE, 0x00);
  WriteParamData(SI114X_PS_ENCODE, 0x00);
  // 0x20: report IR LSB,  0x10: report VIS LSB

  // ADC integration time
  WriteParamData(SI114X_PS_ADC_COUNTER, SI114X_ADC_COUNTER_1ADCCLK); // 1 ADC clock
  WriteParamData(SI114X_ALS_VIS_ADC_COUNTER, SI114X_ADC_COUNTER_1ADCCLK);
  WriteParamData(SI114X_ALS_VIS_ADC_GAIN, SI114X_ADC_GAIN_DIV1);
  WriteParamData(SI114X_ALS_VIS_ADC_MISC, SI114X_ADC_MISC_LOWRANGE);



  // disable proximity measurements, enable raw ADC reading:
  WriteParamData(SI114X_PS_ADC_MISC, SI114X_ADC_MISC_ADC_RAWADC | SI114X_ADC_MISC_ADC_NORMALPROXIMITY);

  // ADC configuration
  /*
   SI114X_ADCMUX_SMALL_IR  0x00
   SI114X_ADCMUX_VISIBLE 0x02
   SI114X_ADCMUX_LARGE_IR  0x03
   SI114X_ADCMUX_NO  0x06
   SI114X_ADCMUX_GND  0x25
   SI114X_ADCMUX_TEMPERATURE  0x65
   SI114X_ADCMUX_VDD  0x75
  */

  //WriteParamData(SI114X_ALS_VIS_ADC_MUX, SI114X_ADCMUX_VISIBLE);
  // select input for ALS IR measurement:
  // 0x00: small IR photodiode
  // 0x03: large IR photodiode
  WriteParamData(SI114X_ALS_IR_ADC_MUX, SI114X_ADCMUX_SMALL_IR);
  WriteParamData(SI114X_PS1_ADCMUX, SI114X_ADCMUX_LARGE_IR);
  WriteParamData(SI114X_PS2_ADCMUX, SI114X_ADCMUX_GND);
  WriteParamData(SI114X_PS3_ADCMUX, SI114X_ADCMUX_NO);
  // measure temperature on AUX ADC
  WriteParamData(SI114X_AUX_ADC_MUX, SI114X_ADCMUX_TEMPERATURE);


  //
  //set LED1 CURRENT(22.4mA)(It is a normal value for many LED)
  //
  //WriteParamData(SI114X_PS1_ADCMUX, SI114X_ADCMUX_LARGE_IR);
  //WriteByte(SI114X_PS_LED21, SI114X_LED_CURRENT_22MA);
  //WriteParamData(SI114X_PSLED12_SELECT, SI114X_PSLED12_SELECT_PS1_LED1); //
  //
  //PS ADC SETTING
  //
  //WriteParamData(SI114X_PS_ADC_GAIN, SI114X_ADC_GAIN_DIV1);
  //WriteParamData(SI114X_PS_ADC_COUNTER, SI114X_ADC_COUNTER_511ADCCLK);
  //WriteParamData(SI114X_PS_ADC_MISC, SI114X_ADC_MISC_HIGHRANGE|SI114X_ADC_MISC_ADC_RAWADC);
  //
  //VIS ADC SETTING
  //
  WriteParamData(SI114X_ALS_VIS_ADC_GAIN, SI114X_ADC_GAIN_DIV1);
  WriteParamData(SI114X_ALS_VIS_ADC_COUNTER, SI114X_ADC_COUNTER_511ADCCLK);
  WriteParamData(SI114X_ALS_VIS_ADC_MISC, SI114X_ADC_MISC_HIGHRANGE);
  //
  //IR ADC SETTING
  //
  WriteParamData(SI114X_ALS_IR_ADC_GAIN, SI114X_ADC_GAIN_DIV1);
  WriteParamData(SI114X_ALS_IR_ADC_COUNTER, SI114X_ADC_COUNTER_511ADCCLK);
  WriteParamData(SI114X_ALS_IR_ADC_MISC, SI114X_ADC_MISC_HIGHRANGE);
  WriteParamData(SI114X_ALS_IR_ADC_MUX, SI114X_ADCMUX_LARGE_IR); // select large IR photodiode for IR measurements
  //
  //interrupt enable
  //
  WriteByte(SI114X_INT_CFG, SI114X_INT_CFG_INTOE);  // enable interrupts
  WriteByte(SI114X_IRQ_ENABLE, SI114X_IRQEN_PS3 | SI114X_IRQEN_PS2 | SI114X_IRQEN_PS1 | SI114X_IRQEN_ALS);   // only interrupt on ALS measurements
  //
  //AUTO RUN
  //
  WriteByte(SI114X_MEAS_RATE0, 0x0); // no automated measurements!
  WriteByte(SI114X_MEAS_RATE1, 0x0);
  //WriteByte(SI114X_COMMAND, SI114X_PSALS_AUTO);
  //WriteByte(SI114X_COMMAND, SI114X_ALS_FORCE); // not interested in proximity sensing

  // from datasheets
  WriteByte(SI114X_HW_KEY, SI114X_HW_KEY_VAL0);

  WriteByte(SI114X_COMMAND, SI114X_NOP);
}

static uint8_t cmd; // 3 bit rollover command counter
static uint16_t err; // last error received
static uint8_t intpin; // pin to which the interrupt line is attached

double SI114X::MeasureIR()
{

  WriteParamData(SI114X_CHLIST, SI114X_CHLIST_ENUV | SI114X_CHLIST_ENALSIR | SI114X_CHLIST_ENALSVIS | SI114X_CHLIST_ENPS1 | SI114X_CHLIST_ENPS2 | SI114X_CHLIST_ENPS3);

  //uint16_t oldresp = ReadByte(SI114X_RESPONSE);
  //Serial.print("old response: "); Serial.println(oldresp);

  //Serial.print("current cmd: "); Serial.println(oldresp, BIN);

  //uint8_t newcmd = oldresp;
  //cmd = newcmd;
  //WriteByte(SI114X_COMMAND, SI114X_PSALS_FORCE);

/*
  while (newcmd == cmd)
  {
    // read command response register to check when the command has been executed
    uint16_t response = ReadByte(SI114X_RESPONSE);
    Serial.print("loop response: "); Serial.println(response, BIN);
    //newcmd = response & 0x04; // 3 LSB are rollover counter
    newcmd = response & 0x0F;
    delay(100);
  }
*/
  uint8_t iRetVal = ExecuteCommand(SI114X_PSALS_FORCE);
//Serial.print("RetVal: "); Serial.println(iRetVal, HEX);

  //cmd = newcmd;
  //Serial.print("new command: "); Serial.println(cmd, BIN);
  //Serial.print("response: "); Serial.println(ReadByte(SI114X_RESPONSE));

  //uint16_t new_resp = ReadByte(SI114X_RESPONSE);
  //Serial.print("second response: "); Serial.println(new_resp, BIN);
  //Serial.print("mask: "); Serial.println(new_resp & 0xF0, BIN);
  //Serial.print("ref mask: "); Serial.println(0x80, BIN);


  if (iRetVal == 0x8D) // ADC overflow on IR measurement
  {
    //Serial.println("ADC overflow!");
    return -1;
  }
  uint16_t ir = ReadHalfWord(SI114X_ALS_IR_DATA0);
  //Serial.print("measured value: "); Serial.println(ir);

  uint16_t ref = ReadHalfWord(SI114X_PS3_DATA0);
  //Serial.print("reference value: "); Serial.println(ref);

  // simple divide:
  if (ir < ref) ir = ref;
  double ir_value = ((ir - ref) << 2) / 2.734; // 16 MSB reported of 17 bit ADC value, so left shift


  return ir_value;

  //uint16_t temp = ReadHalfWord(SI114X_PS2_DATA0);
  //Serial.print("temp ADC: "); Serial.println(temp);
  //double tc = ((double)temp - 11136.) / 35. + 25.; // offset is 11136 at 25 degrees, with a drift of 35 ADC counts per degree
  //Serial.print("temperature: "); Serial.println(tc);


}

uint8_t SI114X::ExecuteCommand(uint8_t command)
{
  uint8_t iPrevResp = ReadByte(SI114X_RESPONSE);
  uint8_t iResp = iPrevResp;
  WriteByte(SI114X_COMMAND, command);
  while (iResp == iPrevResp)
  {
    iResp = ReadByte(SI114X_RESPONSE);
    //Serial.print("Loop response: "); Serial.println(iResp, HEX);
  }
  //Serial.print("Received response: "); Serial.println(iResp, HEX);
  if ((iResp & 0xF0) == 0)
    return 0;
  else
  {
    // clear the error
    //Serial.println("clear error procedure entered");
    ExecuteCommand(SI114X_NOP);
    return iResp; // return false if an invalid command was encountered
  }


  /*
   * #define SI114X_RESP_INV_COMMAND 0x80
   * #define SI114X_RESP_PS1_OVERFLOW 0x88
   * #define SI114X_RESP_PS2_OVERFLOW 0x89
   * #define SI114X_RESP_PS3_OVERFLOW 0x8A
   * #define SI114X_RESP_ALS_VIS_OVERFLOW 0x8C
   * #define SI114X_RESP_ALS_IR_OVERFLOW 0x8D
   * #define SI114X_RESP_AUX_OVERFLOW 0x8E
   */

}

double SI114X::MeasureTemperature()
{
  // to measure temperature, a reference value to ground is necessary
  // reconfigure AUX ADC MUX to make this measurement
  WriteParamData(SI114X_CHLIST, SI114X_CHLIST_ENAUX | SI114X_CHLIST_ENALSIR | SI114X_CHLIST_ENALSVIS | SI114X_CHLIST_ENPS1 | SI114X_CHLIST_ENPS2 | SI114X_CHLIST_ENPS3);

    WriteParamData(SI114X_AUX_ADC_MUX, SI114X_ADCMUX_TEMPERATURE);

    uint8_t iRetVal = ExecuteCommand(SI114X_PSALS_FORCE);
    //Serial.print("Received response: "); Serial.println(iRetVal, HEX);



    uint16_t temp = ReadHalfWord(SI114X_AUX_DATA0_UVINDEX0);
    //Serial.print("Temp value: "); Serial.println(temp);
    uint16_t gndref = ReadHalfWord(SI114X_PS2_DATA0);
    //Serial.print("GND ref value:"); Serial.println(gndref);

    double tempC = (((double)temp - (double)gndref) - 11136) / 35. + 25;

    return tempC;

}

double SI114X::MeasureUVIndex()
{
  // reconfigure AUX ADC MUX for UV measurement
  WriteParamData(SI114X_CHLIST, SI114X_CHLIST_ENUV | SI114X_CHLIST_ENALSIR | SI114X_CHLIST_ENALSVIS);

  uint8_t iRetVal = ExecuteCommand(SI114X_PSALS_FORCE);
  //Serial.print("Received response: "); Serial.println(iRetVal, HEX);

  uint16_t uv = ReadHalfWord(SI114X_AUX_DATA0_UVINDEX0);
  //Serial.print("UV value: "); Serial.println(uv);

  double uvindex = uv / 100.;

  return uvindex;

}

struct zeroes
{
  double x0;
  double x1;
  double x2;
};


double SI114X::MeasureVIS()
{
  WriteParamData(SI114X_CHLIST, SI114X_CHLIST_ENUV | SI114X_CHLIST_ENALSIR | SI114X_CHLIST_ENALSVIS | SI114X_CHLIST_ENPS1 | SI114X_CHLIST_ENPS2 | SI114X_CHLIST_ENPS3);

  uint8_t iRetVal = ExecuteCommand(SI114X_PSALS_FORCE);

  if (iRetVal == 0x8D) // ADC overflow on IR measurement
  {
    //Serial.println("ADC overflow!");
    return -1;
  }
  uint16_t ir = ReadHalfWord(SI114X_ALS_IR_DATA0);
  //Serial.print("measured value: "); Serial.println(ir);

  uint16_t ref = ReadHalfWord(SI114X_PS3_DATA0);
  //Serial.print("reference value: "); Serial.println(ref);

  uint16_t vis = ReadHalfWord(SI114X_ALS_VIS_DATA0);



  // simple divide:
  if (ir < ref) ir = ref;
  if (vis < ref) vis = ref;

  //double ir_value = ((ir - ref) << 2) / 2.734; // 16 MSB reported of 17 bit ADC value, so left shift


  //double zeroes[] = {0,0,0,0,0};
  //double zeroi[]={0,0,0,0,0};
  double opr[5];


  double a, b; // measured values;
  b = (double)((vis - ref) << 2);
  a = (double)((ir - ref) << 2);
  //Serial.print("a: "); Serial.println(a);
  //Serial.print("b: "); Serial.println(b);

  //a = 200;
  //b = 40;
  double p = RATIO_P;

  //double xr[3];

  double zeroes[4];
  double* xr = zeroes;

  opr[0] = b * IR_fA * pow(10, IR_eA) / p - a * VIS_fA * pow(10, VIS_eA);
  opr[1] = (b * IR_fB * pow(10, IR_eB) / p - a * VIS_fB * pow(10, VIS_eB)) / opr[0];
  opr[2] = (b * IR_fC / p - a * VIS_fC) / opr[0];
  opr[3] = (b * IR_fD / p - a * VIS_fD) / opr[0];
  opr[4] = (b * IR_fE / p - a * VIS_fE) / opr[0];

  // add a test vector for verification purposes
  //opr[0] = 1.;
  //opr[1] = -20.;
  //opr[2] = 140.;
  //opr[3] = -400.;
  //opr[4] = 384.;

  //opr[0] = 1.;
  //opr[1] = -2082.32;
  //opr[2] = 1.30419 * pow(10, 6);
  //opr[3] = -2.42559 * pow(10, 8);


  //double opi[] = {0, 0, 0, 0, 0};

  // Note to self: the zeroes change only VERY slightly with light intensity, this indicates the wavelength estimation is independent of illuminance!!
  SolveP4(xr, opr[1], opr[2], opr[3], opr[4]);

  //SolveP3(xr, opr[1], opr[2], opr[3]);

  //for (uint8_t cnt = 0; cnt < 4; cnt++)
  //{
  //  Serial.print("x0: "); Serial.println(zeroes[cnt]);
  //}

  // estimate wavelength by rejection of zeroes that are out of range
  double wavelength = -1;
  for (uint8_t zeropoint = 0; zeropoint < sizeof(zeroes); zeropoint++)
  {
    if (zeroes[zeropoint] > 400 && zeroes[zeropoint] < 1000)
    {
      wavelength = zeroes[zeropoint];
      break;
    }
  }
  if (wavelength == -1) // wavelength estimation failed, default to average value
    wavelength = 750;

  //Serial.print("estimated wavelength: "); Serial.println(wavelength);

  // calculate amplitude with A = b/f(k) where k is the wavelength:
  double fk = VIS_fA * pow(10, VIS_eA) * pow(wavelength, 4) + VIS_fB * pow(10, VIS_eB) * pow(wavelength, 3) + VIS_fC * pow(wavelength, 2) + VIS_fD * wavelength + VIS_fE;
  //Serial.print("fk: "); Serial.println(fk);
  double gk = IR_fA * pow(10, IR_eA) * pow(wavelength, 4) + IR_fB * pow(10, IR_eB) * pow(wavelength, 3) + IR_fC * pow(wavelength, 2) + IR_fD * wavelength + IR_fE;
  //Serial.print("gk: "); Serial.println(gk);
  double Amp = b / fk;
  //Serial.print("amp: "); Serial.println(Amp);

  // finally, calculate visible light in lux, and substract light pollution from the LEDs:
  double VIS = Amp * fk - 20;
  if (VIS < 0) VIS = 0;

  //Serial.print("illuminance: "); Serial.print(VIS); Serial.println(" lx");


  //*(xr + 2) = 4;
  //for (uint8_t j = 0; j < 3; j++)
  //{
  //  Serial.print("x: "); Serial.println(*(xr + j), 5);
  //}

  //int x = poly(opr, opi, 5, zeroes, zeroi);
  //Serial.print("x: "); Serial.println(x);

  //Serial.print("response main: "); Serial.println(x);
  //for (uint8_t i = 0; i < 5; i++)
  //{
  //  Serial.print("Z"); Serial.print(i); Serial.print(": "); Serial.print(*(zeroes + i), 5); Serial.print(","); Serial.println(*(zeroi + i), 5);
  //}


  return VIS;


}



/*--------------------------------------------------------//
 Init the si114x and begin to collect data
 */
bool SI114X::Begin(uint8_t int_pin)
{
  //Serial.println("\n\r");
  //Serial.println("entered begin");
  Wire.begin();
  delay(25); // Si114X needs 25 ms startup time
  //
  //Init IIC  and reset si1145
  //
  uint8_t iPartID = 0;
  switch (ReadByte(SI114X_PART_ID))
  {
    case 0x45: iPartID = 45; break;
    case 0x46: iPartID = 46; break;
    case 0x47: iPartID = 47; break;
  }

  //if (iPartID != 0)
  //{
  //  Serial.print("Found Si11"); Serial.println(iPartID);
  //}
  //else
  //{
  //  Serial.println("No Si114X series sensor detected!");
  //}

  int Rev = ReadByte(SI114X_REV_ID);
  int Seq = ReadByte(SI114X_SEQ_ID); // sequencer revision:
  /*
  Note  that  for  the  Si1145/6/7  with  SEQ_ID=
0x01,  there  is  a  code  error  that  places
MEAS_RATE0  at  0x0A  and  MEAS_RATE1  at
0x08  instead.  This  will  be  fixed  in
future revisions of the Si1145/6/7.


CHIP STAT: 2 = running, 1 = suspended, 0 = sleep
*/

  //Serial.println("initializing Si1145");

  //Reset();
  //
  //INIT
  //
  //Init();
  return true;
}

/*
responses: MSB set = error
0x80: Invalid Command Encountered during command processing
0x88: ADC Overflow encountered during PS1 measurement
0x89: ADC Overflow encountered during PS2 measurement
0x8A: ADC Overflow encountered during PS3 measurement
0x8C: ADC Overflow encountered during ALS-VIS measurement
0x8D: ADC Overflow encountered during ALS-IR measurement
0x8E: ADC Overflow encountered during AUX measurement
MSB cleared: 3:0 = roll-over counter

*/


/*--------------------------------------------------------//
reset the si114x
inclue IRQ reg, command regs...

 */
void SI114X::Reset(void)
{
  // reset instructions from the datasheet
  WriteByte(SI114X_MEAS_RATE0, 0);
  WriteByte(SI114X_MEAS_RATE1, 0);
  WriteByte(SI114X_IRQ_ENABLE, 0);
  WriteByte(SI114X_IRQ_MODE1, 0);
  WriteByte(SI114X_IRQ_MODE2, 0);
  WriteByte(SI114X_INT_CFG, 0);
  WriteByte(SI114X_IRQ_STATUS, 0xFF);

  WriteByte(SI114X_COMMAND, SI114X_RESET);
  delay(10);
  WriteByte(SI114X_HW_KEY, SI114X_HW_KEY_VAL0);
  delay(10);
}



/*
float SI114X::IRtoLux()
{
   // irlux = ir * 14.5 / 2.44 for range = high and gain = 1;
   uint16_t ir = ReadIR();
   float lux = 2.44;
   float irlux = 0;
   float multiplier = 0;
   unsigned char range = 0;
   unsigned char sensitivity = 0;
   float gain = 1;
   // Get gain multipler
   range = ReadByte(SI114X_ALS_IR_ADC_MISC);
   if ((range & 32) == 32) gain = 14.5;
   // Get sensitivity
   sensitivity = ReadByte(SI114X_ALS_IR_ADC_GAIN);
   //if ((sensitivity & 7) == 0) multiplier = 1;
   //if ((sensitivity & 7) == 1) multiplier = 2;
   //if ((sensitivity & 7) == 2) multiplier = 4;
   // ((sensitivity & 7) == 3) multiplier = 8;
  //  if ((sensitivity & 7) == 4) multiplier = 16;
  // if ((sensitivity & 7) == 5) multiplier = 32;
  // if ((sensitivity & 7) == 6) multiplier = 64;
   //if ((sensitivity & 7) == 7) multiplier = 128;
   multiplier = pow(2, (sensitivity & 7));
   irlux = ir * (gain / (lux * multiplier));
   return(irlux);
};
*/

float SI114X::UVIndex()
{
  return ReadUV() / 100.;
}


/*--------------------------------------------------------//
write one byte into si114x's reg

 */
void SI114X::WriteByte(uint8_t Reg, uint8_t Value)
{
  Wire.beginTransmission(SI114X_ADDR);
  Wire.write(Reg);
  Wire.write(Value);
  Wire.endTransmission();
}
/*--------------------------------------------------------//
read one byte data from si114x

 */
uint8_t SI114X::ReadByte(uint8_t Reg)
{
    Wire.beginTransmission(SI114X_ADDR);
    Wire.write(Reg);
    Wire.endTransmission();
    Wire.requestFrom(SI114X_ADDR, 1);
    return Wire.read();
}
/*--------------------------------------------------------//
read half word(2 bytes) data from si114x

 */
uint16_t SI114X::ReadHalfWord(uint8_t Reg)
{
  uint16_t Value;
  Wire.beginTransmission(SI114X_ADDR);
  Wire.write(Reg);
  Wire.endTransmission();
  Wire.requestFrom(SI114X_ADDR, 2);
  Value = Wire.read();
  Value |= (uint16_t)Wire.read() << 8;
  return Value;
}

void SI114X::ReadBytes(uint8_t Reg, uint8_t Count, uint8_t* buffer)
{
  Wire.beginTransmission(SI114X_ADDR);
  Wire.write(Reg);
  Wire.endTransmission();
  Wire.requestFrom(SI114X_ADDR, Count);
  for (uint8_t pos = 0; pos < Count; pos++)
  {
    buffer[pos] = Wire.read();
  }
}

/*--------------------------------------------------------//
read param data

 */
uint8_t SI114X::ReadParamData(uint8_t Reg)
{
	WriteByte(SI114X_COMMAND, Reg | SI114X_QUERY);
	return ReadByte(SI114X_RD);
}
/*--------------------------------------------------------//
writ param data

 */
uint8_t SI114X::WriteParamData(uint8_t Reg,uint8_t Value)
{
	//write Value into PARAMWR reg first
   WriteByte(SI114X_WR, Value);
   WriteByte(SI114X_COMMAND, Reg | SI114X_SET);
   //SI114X writes value out to PARAM_RD,read and confirm its right
   return ReadByte(SI114X_RD);
}

/*--------------------------------------------------------//
Read Visible Value

 */
 uint16_t SI114X::ReadVisible(void)
{
  return ReadHalfWord(SI114X_ALS_VIS_DATA0);
}
 /*--------------------------------------------------------//
Read IR Value

 */
 uint16_t SI114X::ReadIR(void)
{
  return ReadHalfWord(SI114X_ALS_IR_DATA0);
}

 /*--------------------------------------------------------//
Read UV Value
this function is a int value ,but the real value must be div 100

 */
uint16_t SI114X::ReadUV(void)
{
  return (ReadHalfWord(SI114X_AUX_DATA0_UVINDEX0));
}


#define  TwoPi  6.28318530717958648
const double eps=1e-14;
//---------------------------------------------------------------------------
// x - array of size 3
// In case 3 real roots: => x[0], x[1], x[2], return 3
//         2 real roots: x[0], x[1],          return 2
//         1 real root : x[0], x[1] � i*x[2], return 1
int SI114X::SolveP3(double *x,double a,double b,double c)
{ // solve cubic equation x^3 + a*x^2 + b*x + c
  double a2 = a*a;
  //Serial.print("a2: "); Serial.println(a2);
    double q  = (a2 - 3*b)/9;
    //Serial.print("q :"); Serial.println(q);
  double r  = (a*(2*a2-9*b) + 27*c)/54;
  //Serial.print("r: "); Serial.println(r);
    double r2 = r*r;
    //Serial.print("r2: "); Serial.println(r2);
  double q3 = q*q*q;
  //Serial.print("q3: "); Serial.println(q3);
  double A,B, adiv;
    if(r2<q3) {
        double t=r/sqrt(q3);
        //Serial.print("t: "); Serial.println(t);
    if( t<-1) t=-1;
    if( t> 1) t= 1;
        t=acos(t);
        //Serial.print("acos t: "); Serial.println(t);
        adiv = a/3; q=-2*sqrt(q);
        //Serial.print("a/3: "); Serial.println(adiv);
        //Serial.print("q: "); Serial.println(q);


        double x0=q*cos(t/3)-adiv;
        //Serial.print("x0: "); Serial.println(x0);

        double x1= q*cos((t+TwoPi)/3)-adiv;
        //Serial.print("x1: "); Serial.println(x1);

        double x2 = q*cos((t-TwoPi)/3)-adiv;
        //Serial.print("x2: "); Serial.println(x2);

        //Serial.print("cos((t-twoPi)/3): "); Serial.println(q *cos((t-TwoPi)/3));
        //Serial.print("a: "); Serial.println(adiv);

        x[0] = x0;
        x[1] = x1;
        x[2] = x2;

                //Serial.print("roots: "); for (uint8_t izer = 0; izer < 3; izer++){Serial.print(x[izer]);Serial.print(" ");}
        return(3);
    } else {
        A =-pow(fabs(r)+sqrt(r2-q3),1./3);
    if( r<0 ) A=-A;
    B = A==0? 0 : B=q/A;

    a/=3;
    x[0] =(A+B)-a;
        x[1] =-0.5*(A+B)-a;
        x[2] = 0.5*sqrt(3.)*(A-B);
        //Serial.print("roots: "); for (uint8_t izer = 0; izer < 3; izer++){Serial.print(x[izer]);Serial.print(" ");}
    if(fabs(x[2])<eps) { x[2]=x[1]; return(2); }
        return(1);
    }
}// SolveP3(double *x,double a,double b,double c) {

//-----------------------------------------------------------------------------
// x - array of size 4
// return 4: 4 real roots x[0], x[1], x[2], x[3], possible multiple roots
// return 2: 2 real roots x[0], x[1] and complex x[2]�i*x[3],
// return 0: two pair of complex roots: x[0]�i*x[1],  x[2]�i*x[3],
int SI114X::SolveP4(double *x,double a,double b,double c,double d) {  // solve equation x^4 + a*x^3 + b*x^2 + c*x + d by Dekart-Euler method
  // move to a=0:
  double d1 = d + 0.25*a*( 0.25*b*a - 3./64*a*a*a - c);
  double c1 = c + 0.5*a*(0.25*a*a - b);
  double b1 = b - 0.375*a*a;
  int res = SolveP4De( x, b1, c1, d1);
  if( res==4) { x[0]-= a/4; x[1]-= a/4; x[2]-= a/4; x[3]-= a/4; }
  else if (res==2) { x[0]-= a/4; x[1]-= a/4; x[2]-= a/4; }
  else             { x[0]-= a/4; x[2]-= a/4; }
  // one Newton step for each real root:
  if( res>0 )
  {
    x[0] = N4Step(x[0], a,b,c,d);
    x[1] = N4Step(x[1], a,b,c,d);
  }
  if( res>2 )
  {
    x[2] = N4Step(x[2], a,b,c,d);
    x[3] = N4Step(x[3], a,b,c,d);
  }
  return res;
}

//-----------------------------------------------------------------------------
double SI114X::N4Step(double x, double a,double b,double c,double d)  // one Newton step for x^4 + a*x^3 + b*x^2 + c*x + d
{
  double fxs= ((4*x+3*a)*x+2*b)*x+c;  // f'(x)
  if( fxs==0 ) return 1e99;
  double fx = (((x+a)*x+b)*x+c)*x+d;  // f(x)
  return x - fx/fxs;
}

//---------------------------------------------------------------------------
int   SI114X::SolveP4De(double *x, double b, double c, double d)  // solve equation x^4 + b*x^2 + c*x + d
{
  //if( c==0 ) return SolveP4Bi(x,b,d); // After that, c!=0
  if( fabs(c)<1e-14*(fabs(b)+fabs(d)) ) return SolveP4Bi(x,b,d); // After that, c!=0

  int res3 = SolveP3( x, 2*b, b*b-4*d, -c*c); // solve resolvent
  // by Viet theorem:  x1*x2*x3=-c*c not equals to 0, so x1!=0, x2!=0, x3!=0
  if( res3>1 )  // 3 real roots,
  {
    dblSort3(x[0], x[1], x[2]); // sort roots to x[0] <= x[1] <= x[2]
    // Note: x[0]*x[1]*x[2]= c*c > 0
    if( x[0] > 0) // all roots are positive
    {
      double sz1 = sqrt(x[0]);
      double sz2 = sqrt(x[1]);
      double sz3 = sqrt(x[2]);
      // Note: sz1*sz2*sz3= -c (and not equal to 0)
      if( c>0 )
      {
        x[0] = (-sz1 -sz2 -sz3)/2;
        x[1] = (-sz1 +sz2 +sz3)/2;
        x[2] = (+sz1 -sz2 +sz3)/2;
        x[3] = (+sz1 +sz2 -sz3)/2;
        return 4;
      }
      // now: c<0
      x[0] = (-sz1 -sz2 +sz3)/2;
      x[1] = (-sz1 +sz2 -sz3)/2;
      x[2] = (+sz1 -sz2 -sz3)/2;
      x[3] = (+sz1 +sz2 +sz3)/2;
      return 4;
    } // if( x[0] > 0) // all roots are positive
    // now x[0] <= x[1] < 0, x[2] > 0
    // two pair of comlex roots
    double sz1 = sqrt(-x[0]);
    double sz2 = sqrt(-x[1]);
    double sz3 = sqrt( x[2]);

    if( c>0 ) // sign = -1
    {
      x[0] = -sz3/2;
      x[1] = ( sz1 -sz2)/2;   // x[0]�i*x[1]
      x[2] =  sz3/2;
      x[3] = (-sz1 -sz2)/2;   // x[2]�i*x[3]
      return 0;
    }
    // now: c<0 , sign = +1
    x[0] =   sz3/2;
    x[1] = (-sz1 +sz2)/2;
    x[2] =  -sz3/2;
    x[3] = ( sz1 +sz2)/2;
    return 0;
  } // if( res3>1 ) // 3 real roots,
  // now resoventa have 1 real and pair of compex roots
  // x[0] - real root, and x[0]>0,
  // x[1]�i*x[2] - complex roots,
  double sz1 = sqrt(x[0]);
  double szr, szi;
  CSqrt(x[1], x[2], szr, szi);  // (szr+i*szi)^2 = x[1]+i*x[2]
  if( c>0 ) // sign = -1
  {
    x[0] = -sz1/2-szr;      // 1st real root
    x[1] = -sz1/2+szr;      // 2nd real root
    x[2] = sz1/2;
    x[3] = szi;
    return 2;
  }
  // now: c<0 , sign = +1
  x[0] = sz1/2-szr;     // 1st real root
  x[1] = sz1/2+szr;     // 2nd real root
  x[2] = -sz1/2;
  x[3] = szi;
  return 2;
} // SolveP4De(double *x, double b, double c, double d) // solve equation x^4 + b*x^2 + c*x + d

//---------------------------------------------------------------------------
int   SI114X::SolveP4Bi(double *x, double b, double d)  // solve equation x^4 + b*x^2 + d = 0
{
  double D = b*b-4*d;
  if( D>=0 )
  {
    double sD = sqrt(D);
    double x1 = (-b+sD)/2;
    double x2 = (-b-sD)/2;  // x2 <= x1
    if( x2>=0 )       // 0 <= x2 <= x1, 4 real roots
    {
      double sx1 = sqrt(x1);
      double sx2 = sqrt(x2);
      x[0] = -sx1;
      x[1] =  sx1;
      x[2] = -sx2;
      x[3] =  sx2;
      return 4;
    }
    if( x1 < 0 )        // x2 <= x1 < 0, two pair of imaginary roots
    {
      double sx1 = sqrt(-x1);
      double sx2 = sqrt(-x2);
      x[0] =    0;
      x[1] =  sx1;
      x[2] =    0;
      x[3] =  sx2;
      return 0;
    }
    // now x2 < 0 <= x1 , two real roots and one pair of imginary root
      double sx1 = sqrt( x1);
      double sx2 = sqrt(-x2);
      x[0] = -sx1;
      x[1] =  sx1;
      x[2] =    0;
      x[3] =  sx2;
      return 2;
  } else { // if( D < 0 ), two pair of compex roots
    double sD2 = 0.5*sqrt(-D);
    CSqrt(-0.5*b, sD2, x[0],x[1]);
    CSqrt(-0.5*b,-sD2, x[2],x[3]);
    return 0;
  } // if( D>=0 )
} // SolveP4Bi(double *x, double b, double d) // solve equation x^4 + b*x^2 d

 //---------------------------------------------------------------------------
#define SWAP(a,b) { t=b; b=a; a=t; }
void SI114X::dblSort3( double &a, double &b, double &c) // make: a <= b <= c
{
 double t;
  if( a>b ) SWAP(a,b);  // now a<=b
  if( c<b ) {
    SWAP(b,c);      // now a<=b, b<=c
    if( a>b ) SWAP(a,b);// now a<=b
  }
}

//---------------------------------------------------------------------------
// a>=0!
void  SI114X::CSqrt( double x, double y, double &a, double &b) // returns:  a+i*s = sqrt(x+i*y)
{
  double r  = sqrt(x*x+y*y);
  if( y==0 ) {
    r = sqrt(r);
    if(x>=0) { a=r; b=0; } else { a=0; b=r; }
  } else {    // y != 0
    a = sqrt(0.5*(x+r));
    b = 0.5*y/a;
  }
}
