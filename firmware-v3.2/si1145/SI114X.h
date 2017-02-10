#ifndef _SI114X_H_
#define _SI114X_H_
#include <Arduino.h>
/*------------------------------------------------------// 
Registers,Parameters and commands

*/
//
//commands
//
#define SI114X_QUERY 0X80
#define SI114X_SET 0XA0
#define SI114X_NOP 0X0
#define SI114X_RESET    0X01
#define SI114X_BUSADDR    0X02
#define SI114X_PS_FORCE    0X05
#define SI114X_GET_CAL    0X12
#define SI114X_ALS_FORCE    0X06
#define SI114X_PSALS_FORCE    0X07
#define SI114X_PS_PAUSE    0X09
#define SI114X_ALS_PAUSE    0X0A
#define SI114X_PSALS_PAUSE    0XB
#define SI114X_PS_AUTO    0X0D
#define SI114X_ALS_AUTO   0X0E
#define SI114X_PSALS_AUTO 0X0F
//
//IIC REGISTERS
//
#define SI114X_PART_ID  0X00
#define SI114X_REV_ID  0X01
#define SI114X_SEQ_ID  0X02
#define SI114X_INT_CFG  0X03
#define SI114X_IRQ_ENABLE  0X04
#define SI114X_IRQ_MODE1 0x05
#define SI114X_IRQ_MODE2 0x06
#define SI114X_HW_KEY  0X07
#define SI114X_MEAS_RATE0 0X08
#define SI114X_MEAS_RATE1  0X09
#define SI114X_PS_RATE  0X0A
#define SI114X_PS_LED21  0X0F
#define SI114X_PS_LED3  0X10
#define SI114X_UCOEFF0  0X13
#define SI114X_UCOEFF1  0X14
#define SI114X_UCOEFF2  0X15
#define SI114X_UCOEFF3  0X16
#define SI114X_WR  0X17
#define SI114X_COMMAND  0X18
#define SI114X_RESPONSE  0X20
#define SI114X_IRQ_STATUS  0X21
#define SI114X_ALS_VIS_DATA0 0X22
#define SI114X_ALS_VIS_DATA1 0X23
#define SI114X_ALS_IR_DATA0 0X24
#define SI114X_ALS_IR_DATA1 0X25
#define SI114X_PS1_DATA0 0X26
#define SI114X_PS1_DATA1 0X27
#define SI114X_PS2_DATA0 0X28
#define SI114X_PS2_DATA1 0X29
#define SI114X_PS3_DATA0 0X2A
#define SI114X_PS3_DATA1 0X2B
#define SI114X_AUX_DATA0_UVINDEX0 0X2C
#define SI114X_AUX_DATA1_UVINDEX1 0X2D
#define SI114X_RD 0X2E
#define SI114X_CHIP_STAT 0X30
//
//Parameters
//
#define SI114X_I2C_ADDR 0X00

#define SI114X_CHLIST   0X01
#define SI114X_CHLIST_ENUV 0x80
#define SI114X_CHLIST_ENAUX 0x40
#define SI114X_CHLIST_ENALSIR 0x20
#define SI114X_CHLIST_ENALSVIS 0x10
#define SI114X_CHLIST_ENPS1 0x01
#define SI114X_CHLIST_ENPS2 0x02
#define SI114X_CHLIST_ENPS3 0x04

#define SI114X_PSLED12_SELECT   0X02
#define SI114X_PSLED3_SELECT   0X03

#define SI114X_PS_ENCODE   0X05
#define SI114X_ALS_ENCODE  0X06

#define SI114X_PS1_ADCMUX   0X07
#define SI114X_PS2_ADCMUX   0X08
#define SI114X_PS3_ADCMUX   0X09

#define SI114X_PS_ADC_COUNTER   0X0A
#define SI114X_PS_ADC_GAIN 0X0B
#define SI114X_PS_ADC_MISC 0X0C

#define SI114X_ALS_IR_ADC_MUX   0X0E
#define SI114X_AUX_ADC_MUX   0X0F

#define SI114X_ALS_VIS_ADC_COUNTER   0X10
#define SI114X_ALS_VIS_ADC_GAIN 0X11
#define SI114X_ALS_VIS_ADC_MISC 0X12

#define SI114X_LED_REC 0X1C

#define SI114X_ALS_IR_ADC_COUNTER   0X1D
#define SI114X_ALS_IR_ADC_GAIN 0X1E
#define SI114X_ALS_IR_ADC_MISC 0X1F
//
//USER SETTINGS DEFINE
//
//ADCMUX
#define SI114X_ADCMUX_SMALL_IR  0x00
#define SI114X_ADCMUX_VISIBLE 0x02
#define SI114X_ADCMUX_LARGE_IR  0x03
#define SI114X_ADCMUX_NO  0x06
#define SI114X_ADCMUX_GND  0x25
#define SI114X_ADCMUX_TEMPERATURE  0x65
#define SI114X_ADCMUX_VDD  0x75
//LED SEL
#define SI114X_PSLED12_SELECT_PS1_NONE 0x00
#define SI114X_PSLED12_SELECT_PS1_LED1 0x01
#define SI114X_PSLED12_SELECT_PS1_LED2 0x02
#define SI114X_PSLED12_SELECT_PS1_LED3 0x04
#define SI114X_PSLED12_SELECT_PS2_NONE 0x00
#define SI114X_PSLED12_SELECT_PS2_LED1 0x10
#define SI114X_PSLED12_SELECT_PS2_LED2 0x20
#define SI114X_PSLED12_SELECT_PS2_LED3 0x40
#define SI114X_PSLED3_SELECT_PS2_NONE 0x00
#define SI114X_PSLED3_SELECT_PS2_LED1 0x10
#define SI114X_PSLED3_SELECT_PS2_LED2 0x20
#define SI114X_PSLED3_SELECT_PS2_LED3 0x40
//ADC GAIN DIV
#define SI114X_ADC_GAIN_DIV1 0X00
#define SI114X_ADC_GAIN_DIV2 0X01
#define SI114X_ADC_GAIN_DIV4 0X02
#define SI114X_ADC_GAIN_DIV8 0X03
#define SI114X_ADC_GAIN_DIV16 0X04
#define SI114X_ADC_GAIN_DIV32 0X05
//LED CURRENT
#define SI114X_LED_CURRENT_5MA 0X01
#define SI114X_LED_CURRENT_11MA 0X02
#define SI114X_LED_CURRENT_22MA 0X03
#define SI114X_LED_CURRENT_45MA 0X04
//Recovery period the  ADC takes before making a PS measurement
#define SI114X_ADC_COUNTER_1ADCCLK 0X00
#define SI114X_ADC_COUNTER_7ADCCLK 0X01
#define SI114X_ADC_COUNTER_15ADCCLK 0X02
#define SI114X_ADC_COUNTER_31ADCCLK 0X03
#define SI114X_ADC_COUNTER_63ADCCLK 0X04
#define SI114X_ADC_COUNTER_127ADCCLK 0X05
#define SI114X_ADC_COUNTER_255ADCCLK 0X06
#define SI114X_ADC_COUNTER_511ADCCLK 0X07
//ADC MISC
#define SI114X_ADC_MISC_LOWRANGE 0X00
#define SI114X_ADC_MISC_HIGHRANGE 0X20
#define SI114X_ADC_MISC_ADC_NORMALPROXIMITY 0X00
#define SI114X_ADC_MISC_ADC_RAWADC 0X04
//INT OE
#define SI114X_INT_CFG_INTOE 0X01
//IRQ ENABLE
#define SI114X_IRQEN_ALS 0x01
#define SI114X_IRQEN_PS1 0x04
#define SI114X_IRQEN_PS2 0x08
#define SI114X_IRQEN_PS3 0x10

#define SI114X_ADDR 0X60

#define SI114X_HW_KEY_VAL0               0x17

// RESPONSES
#define SI114X_RESP_INV_COMMAND 0x80
#define SI114X_RESP_PS1_OVERFLOW 0x88
#define SI114X_RESP_PS2_OVERFLOW 0x89
#define SI114X_RESP_PS3_OVERFLOW 0x8A
#define SI114X_RESP_ALS_VIS_OVERFLOW 0x8C
#define SI114X_RESP_ALS_IR_OVERFLOW 0x8D
#define SI114X_RESP_AUX_OVERFLOW 0x8E

#define FLT_TO_FX20(x)       ((x*1048576)+.5)
#define FX20_ONE             FLT_TO_FX20( 1.000000)
#define FX20_BAD_VALUE       0xffffffff

//                                                   msb   lsb   align
//                                                   i2c   i2c   ment
//                                                   addr  addr
#define SIRPD_ADCHI_IRLED    (collect(buffer, 0x23, 0x22,  0))
#define SIRPD_ADCLO_IRLED    (collect(buffer, 0x22, 0x25,  1))
#define SIRPD_ADCLO_WHLED    (collect(buffer, 0x24, 0x26,  0))
#define VISPD_ADCHI_WHLED    (collect(buffer, 0x26, 0x27,  1))
#define VISPD_ADCLO_WHLED    (collect(buffer, 0x28, 0x29,  0))
#define LIRPD_ADCHI_IRLED    (collect(buffer, 0x29, 0x2a,  1))
#define LED_DRV65            (collect(buffer, 0x2b, 0x2c,  0))

//
// Structure Definition for calref array 
//
struct cal_ref_t 
{
    uint32_t sirpd_adchi_irled;
    uint32_t sirpd_adclo_irled;
    uint32_t sirpd_adclo_whled;
    uint32_t vispd_adchi_whled;
    uint32_t vispd_adclo_whled;
    uint32_t lirpd_adchi_irled;
    uint32_t ledi_65ma;
    uint8_t  ucoef[4];
};

//
// This structure is used to store the result of the calibration retrieval
//

typedef struct 
{
    uint32_t     vispd_correction;
    uint32_t     irpd_correction;
    uint32_t     adcrange_ratio;
    uint32_t     irsize_ratio;
    uint32_t     ledi_ratio;
    uint8_t      *ucoef_p;
} SI114X_CAL_S;


class SI114X {
 public:
  bool Begin(uint8_t int_pin);
  void Reset(void);
  void WaitUntilSleep(void);
  void Init(void);
  float IRtoLux(void);
  float UVIndex(void);

  double MeasureIR();
  double MeasureTemperature();
  double MeasureUVIndex();
  double MeasureVIS();
  
  
  uint8_t  ReadParamData(uint8_t Reg);
  uint8_t  WriteParamData(uint8_t Reg,uint8_t Value);
  uint16_t ReadVisible(void);
  uint16_t ReadIR(void);
  
  uint16_t ReadUV(void);
 private:
  void  WriteByte(uint8_t Reg, uint8_t Value);
  uint8_t  ReadByte(uint8_t Reg);
  uint16_t ReadHalfWord(uint8_t Reg);
  void ReadBytes(uint8_t Reg, uint8_t Count, uint8_t* buffer);

  uint8_t ExecuteCommand(uint8_t Cmd);


  int16_t set_ucoef( SI114X_CAL_S *si114x_cal );
  int16_t get_calibration(SI114X_CAL_S *si114x_cal);
  
 int16_t get_cal_index(uint8_t *buffer );
 uint32_t irsize_ratio(uint8_t *buffer);
 uint32_t adcrange_ratio(uint8_t *buffer);
 uint32_t irpd_correction(uint8_t *buffer);
 uint32_t vispd_correction(uint8_t *buffer);
 int16_t cal_index( uint8_t *buffer );
uint32_t fx20_multiply( struct operand_t *operand_p );
uint32_t fx20_divide( struct operand_t *operand_p );
void fx20_round( uint32_t *value_p );
int8_t align( uint32_t *value_p, int8_t direction );
void shift_left(uint32_t *value_p, int8_t shift);
uint32_t collect(uint8_t *buffer, uint8_t msb_addr, uint8_t lsb_addr, uint8_t alignment);
uint32_t decode(uint32_t input);


int   SolveP3(double *x,double a,double b,double c);      // solve cubic equation x^3 + a*x^2 + b*x + c = 0
int   SolveP4(double *x,double a,double b,double c,double d); // solve equation x^4 + a*x^3 + b*x^2 + c*x + d = 0 by Dekart-Euler method

// x - array of size 4
// return 4: 4 real roots x[0], x[1], x[2], x[3], possible multiple roots
// return 2: 2 real roots x[0], x[1] and complex x[2]�i*x[3], 
// return 0: two pair of complex roots: x[0]�i*x[1],  x[2]�i*x[3], 
//int   SolveP5(double *x,double a,double b,double c,double d,double e);  // solve equation x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0

int   SolveP4Bi(double *x, double b, double d);       // solve equation x^4 + b*x^2 + d = 0
int   SolveP4De(double *x, double b, double c, double d); // solve equation x^4 + b*x^2 + c*x + d = 0
void  CSqrt( double x, double y, double &a, double &b);   // returns as a+i*s,  sqrt(x+i*y)
double N4Step(double x, double a,double b,double c,double d);// one Newton step for x^4 + a*x^3 + b*x^2 + c*x + d

//  double SolveP5_1(double a,double b,double c,double d,double e); // return real root of x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0

// Solve2: let f(x ) = a*x^2 + b*x + c and 
//     f(x0) = f0,
//     f(x1) = f1,
//     f(x2) = f3
// Then r1, r2 - root of f(x)=0.
// Returns 0, if there are no roots, else return 2.
//int Solve2( double x0, double x1, double x2, double f0, double f1, double f2, double &r1, double &r2); 

 void dblSort3( double &a, double &b, double &c); // make: a <= b <= c


};



#endif




