
#include "SI114X.h"
//#include <Wire.h>
#include <math.h>
#include <Arduino.h>


//
// Factory Calibration Reference Values
//
struct cal_ref_t calref[2] =
{
    {
        FLT_TO_FX20( 4.021290),  // sirpd_adchi_irled
        FLT_TO_FX20(57.528500),  // sirpd_adclo_irled
        FLT_TO_FX20( 2.690010),  // sirpd_adclo_whled
        FLT_TO_FX20( 0.042903),  // vispd_adchi_whled
        FLT_TO_FX20( 0.633435),  // vispd_adclo_whled
        FLT_TO_FX20(23.902900),  // lirpd_adchi_irled
        FLT_TO_FX20(56.889300),  // ledi_65ma
        {0x7B, 0x6B, 0x01, 0x00} // default ucoef
    },
    {
        FLT_TO_FX20( 2.325484),  // sirpd_adchi_irled
        FLT_TO_FX20(33.541500),  // sirpd_adclo_irled
        FLT_TO_FX20( 1.693750),  // sirpd_adclo_whled
        FLT_TO_FX20( 0.026775),  // vispd_adchi_whled
        FLT_TO_FX20( 0.398443),  // vispd_adclo_whled
        FLT_TO_FX20(12.190900),  // lirpd_adchi_irled
        FLT_TO_FX20(56.558200),  // ledi_65ma
        {0xdb, 0x8f, 0x01, 0x00} // default ucoef
    }
};

//
// Converts the 12-bit factory test value from the Si114x and returns the
// fixed-point representation of this 12-bit factory test value.
//
uint32_t SI114X::decode(uint32_t input)
{
    int32_t  exponent, exponent_bias9;
    uint32_t  mantissa;

    if (input==0) return 0.0;

    exponent_bias9 = (input & 0x0f00) >> 8;
    exponent       = exponent_bias9 - 9;

    mantissa       = input & 0x00ff; // fraction
    mantissa       |=        0x0100; // add in integer

    // representation in 12 bit integer, 20 bit fraction
    mantissa       = mantissa << (12+exponent);
    return mantissa;
}

//
// The buffer[] is assumed to point to a byte array that containst the
// factory calibration values after writing 0x12 to the command register
// This function takes the 12 bytes from the Si114x, then converts it
// to a fixed point representation, with the help of the decode() function
//
uint32_t SI114X::collect(uint8_t *buffer, uint8_t msb_addr, uint8_t lsb_addr, uint8_t alignment)
{
    uint16_t value;
    uint8_t  msb_ind = msb_addr - 0x22;
    uint8_t  lsb_ind = lsb_addr - 0x22;

    if (alignment == 0)
    {
        value =  buffer[msb_ind]<<4;
        value += buffer[lsb_ind]>>4;
    }
    else
    {
        value =  buffer[msb_ind]<<8;
        value += buffer[lsb_ind];
        value &= 0x0fff;
    }

    if (    ( value == 0x0fff )
         || ( value == 0x0000 ) ) return FX20_BAD_VALUE;
    else return decode( value );
}

//
// This performs a shift_left function. For convenience, a negative
// shift value will shift the value right. Value pointed will be
// overwritten.
//
void SI114X::shift_left(uint32_t *value_p, int8_t shift)
{
    if (shift > 0)
        *value_p = *value_p<<shift ;
    else
        *value_p = *value_p>>(-shift) ;
}

//
// Aligns the value pointed by value_p to either the LEFT or the RIGHT
// the number of shifted bits is returned. The value in value_p is
// overwritten.
//
#define ALIGN_LEFT   1
#define ALIGN_RIGHT -1
int8_t SI114X::align( uint32_t *value_p, int8_t direction )
{
    int8_t  local_shift, shift ;
    uint32_t mask;

    // Check invalid value_p and *value_p, return without shifting if bad.
    if( value_p  == NULL )  return 0;
    if( *value_p == 0 )     return 0;

    // Make sure direction is valid
    switch( direction )
    {
        case ALIGN_LEFT:
            local_shift =  1 ;
            mask  = 0x80000000;
            break;

        case ALIGN_RIGHT:
            local_shift = -1 ;
            mask  = 0x00000001;
            break;

        default:
            // Invalid direction, return without shifting
            return 0;
    }

    shift = 0;
    while(1)
    {
        if (*value_p & mask ) break;
        shift++;
        shift_left( value_p, local_shift );
    }
    return shift;
}

//
// fx20_round Rounds the u32 value pointed by ptr, by the number
// of bits specified by round.
//
// This compile switch used only to experiment with
// various rounding precisions. The flexibility has
// a small performance price.
//
void SI114X::fx20_round( uint32_t *value_p )
{
    int8_t  shift;

    // Use the following to force round = 16
    uint32_t mask1  = 0xffff8000;
    uint32_t mask2  = 0xffff0000;
    uint32_t lsb    = 0x00008000;

    shift = align( value_p, ALIGN_LEFT );
    if( ( (*value_p)&mask1 ) == mask1 )
    {
        *value_p = 0x80000000;
        shift -= 1;
    }
    else
    {
        *value_p += lsb;
        *value_p &= mask2;
    }

    shift_left( value_p, -shift );
}

//
// The fx20_divide and fx20_multiply uses this structure to pass
// values into it.
//
struct operand_t
{
   uint32_t op1;
   uint32_t op2;
};

//
// Returns a fixed-point (20-bit fraction) after dividing op1/op2
//
uint32_t SI114X::fx20_divide( struct operand_t *operand_p )
{
    int8_t  numerator_sh=0, denominator_sh=0;
    uint32_t result;
    uint32_t *numerator_p;
    uint32_t *denominator_p;

    if ( operand_p == NULL ) return FX20_BAD_VALUE;

    numerator_p   = &operand_p->op1;
    denominator_p = &operand_p->op2;

    if (   (*numerator_p   == FX20_BAD_VALUE)
        || (*denominator_p == FX20_BAD_VALUE)
        || (*denominator_p == 0             ) ) return FX20_BAD_VALUE;

    fx20_round  ( numerator_p   );
    fx20_round  ( denominator_p );
    numerator_sh   = align ( numerator_p,   ALIGN_LEFT  );
    denominator_sh = align ( denominator_p, ALIGN_RIGHT );

    result = *numerator_p / ( (u16)(*denominator_p) );
    shift_left( &result , 20-numerator_sh-denominator_sh );

    return result;
}

//
// Returns a fixed-point (20-bit fraction) after multiplying op1*op2
//
uint32_t SI114X::fx20_multiply( struct operand_t *operand_p )
{
    uint32_t result;
    int8_t  val1_sh, val2_sh;
    uint32_t *val1_p;
    uint32_t *val2_p;

    if( operand_p == NULL ) return FX20_BAD_VALUE;

    val1_p = &(operand_p->op1);
    val2_p = &(operand_p->op2);

    fx20_round( val1_p );
    fx20_round( val2_p );

    val1_sh = align( val1_p, ALIGN_RIGHT );
    val2_sh = align( val2_p, ALIGN_RIGHT );


    result = (uint32_t)( ( (uint32_t)(*val1_p) ) * ( (uint32_t)(*val2_p) ) );
    shift_left( &result, -20+val1_sh+val2_sh );

    return result;
}

//
// Due to small differences in factory test setup, the reference calibration
// values may have slight variation. This function retrieves the calibration
// index stored in the Si114x so that it is possible to know which calibration
// reference values to use.
//
int16_t SI114X::cal_index( uint8_t *buffer )
{
    int16_t index;
    uint8_t  size;

    // buffer[12] is the LSB, buffer[13] is the MSB
    index = ( int16_t )( buffer[12] + ( (uint16_t)( buffer[13] ) << 8 ) );

    switch( index )
    {
        case -1:
            index = 0;
            break;
        default:
            index = -(2+index) ;
    }

    size = sizeof(calref)/sizeof(calref[0]);

    if ( index < size )
    {
        return  index;
    }
    else
    {
        return -1;
    }
}



//
// Returns the calibration ratio to be applied to VIS measurements
//
uint32_t SI114X::vispd_correction(uint8_t *buffer)
{

    struct operand_t op;
    uint32_t              result;
    int16_t              index = cal_index( buffer );

    if ( index < 0 ) result = FX20_ONE;

    op.op1 = calref[ index ].vispd_adclo_whled;
    op.op2 = VISPD_ADCLO_WHLED;
    result = fx20_divide( &op );

    if ( result == FX20_BAD_VALUE ) result = FX20_ONE;

    return result;
}

//
// Returns the calibration ratio to be applied to IR measurements
//
uint32_t SI114X::irpd_correction(uint8_t *buffer)
{
    struct operand_t op;
    uint32_t              result;
    int16_t              index = cal_index( buffer );

    if ( index < 0 ) result = FX20_ONE;

    // op.op1 = SIRPD_ADCLO_IRLED_REF; op.op2 = SIRPD_ADCLO_IRLED;
    op.op1 = calref[ index ].sirpd_adclo_irled;
    op.op2 = SIRPD_ADCLO_IRLED;
    result = fx20_divide( &op );

    if ( result == FX20_BAD_VALUE ) result = FX20_ONE;

    return result;
}

//
// Returns the ratio to correlate between x_RANGE=0 and x_RANGE=1
// It is typically 14.5, but may have some slight component-to-component
// differences.
//
uint32_t SI114X::adcrange_ratio(uint8_t *buffer)
{
    struct operand_t op;
    uint32_t              result;

    op.op1 = SIRPD_ADCLO_IRLED  ; op.op2 = SIRPD_ADCHI_IRLED  ;
    result = fx20_divide( &op );

    if ( result == FX20_BAD_VALUE ) result = FLT_TO_FX20( 14.5 );

    return result;
}

//
// Returns the ratio to correlate between measurements made from large PD
// to measurements made from small PD.
//
uint32_t SI114X::irsize_ratio(uint8_t *buffer)
{
    struct operand_t op;
    uint32_t              result;

    op.op1 = LIRPD_ADCHI_IRLED  ; op.op2 = SIRPD_ADCHI_IRLED  ;
    result = fx20_divide( &op );

    if ( result == FX20_BAD_VALUE ) result = FLT_TO_FX20(  6.0 );

    return  result;
}

//
// This is a helper function called from si114x_get_calibration()
// Writes 0x11 to the Command Register, then populates buffer[12]
// and buffer[13] with the factory calibration index
//
int16_t SI114X::get_cal_index(uint8_t *buffer )
{
    int16_t retval, response;

    //if ( ( si114x_handle == NULL ) || ( buffer == NULL ) )
    //    return -1;

    // Check to make sure that the device is ready to receive commands
    //do
    //{
    //    retval = Si114xNop( si114x_handle );
    //    if( retval != 0 ) return -1;

    //    response = Si114xReadFromRegister( si114x_handle, REG_RESPONSE );

    //} while ( response != 0 );

    // Retrieve the index
    //retval = Si114xWriteToRegister( si114x_handle, REG_COMMAND, 0x11 );
    retval = ExecuteCommand(0x11);
    //_waitUntilSleep(si114x_handle);

    //if( retval != 0 ) return -1;

    //retval = Si114xBlockRead( si114x_handle, REG_PS1_DATA0, 2, &buffer[12] );
    buffer[12] = ReadByte(SI114X_PS1_DATA0);
    buffer[13] = ReadByte(SI114X_PS1_DATA1);
    //if( retval != 0 ) return -1;

    return 0;
}


int16_t SI114X::get_calibration(SI114X_CAL_S *si114x_cal)
{
    uint8_t             buffer[14];
    int16_t retval = 0, response;
    struct operand_t    op;
    uint16_t                 cal_index;

    //if ( si114x_handle == NULL ) { retval = -4; goto error_exit; }

    //if ( si114x_cal    == NULL ) { retval = -4; goto error_exit; }


    // Request for the calibration data
    //retval = Si114xWriteToRegister( si114x_handle, REG_COMMAND, 0x12 );
    retval = ExecuteCommand(0x12);
    //_waitUntilSleep(si114x_handle);

    //if( retval != 0 ) { retval = -2; goto error_exit; }


    // Retrieve the 12 bytes from the interface registers
    //retval = Si114xBlockRead( si114x_handle, REG_ALS_VIS_DATA0, 12, buffer );
    ReadBytes(SI114X_ALS_VIS_DATA0, 12, buffer);
    //if( retval != 0 ) { retval = -2; goto error_exit; }

/*
        Serial.println("command 0x12 sent");
    for (int i = 0; i < 14; i++)
    {
      Serial.print(buffer[i]); Serial.print(" ");
    }
    */

    //DEBUG_PRINT_OUTPUT;

    /*
      January 2017 modification:
      VIS correction has been disabled because it crashes the firmware of OpenObservatory 3.0.
    */
    //si114x_cal->vispd_correction = vispd_correction(buffer);

    si114x_cal->irpd_correction  = irpd_correction(buffer);

    si114x_cal->adcrange_ratio   = adcrange_ratio(buffer);

    si114x_cal->irsize_ratio     = irsize_ratio(buffer);

    //si114x_cal->ledi_ratio       = ledi_ratio(buffer);
    si114x_cal->ucoef_p          = calref[cal_index].ucoef;

    //DEBUG_PRINT_OUTPUT_2;
        //Serial.println("executed");

    return 0;

//error_exit:
    //si114x_cal->vispd_correction = FX20_ONE;
    //si114x_cal->irpd_correction  = FX20_ONE;
    //si114x_cal->adcrange_ratio   = FLT_TO_FX20( 14.5 );
    //si114x_cal->irsize_ratio     = FLT_TO_FX20(  6.0 );
    //si114x_cal->ledi_ratio       = FX20_ONE;
    //si114x_cal->ucoef_p          = NULL;
    //return retval;
}

int16_t SI114X::set_ucoef( SI114X_CAL_S *si114x_cal )
{
    //int16_t     response;
    int16_t     temp;
    uint32_t     vc=FX20_ONE, ic=FX20_ONE, long_temp;
    struct operand_t    op;
    uint8_t     *ref_ucoef = si114x_cal->ucoef_p;
    uint8_t     out_ucoef[4];

    if (si114x_cal != 0)
    {
        if (si114x_cal->vispd_correction > 0) vc = si114x_cal->vispd_correction;
        if (si114x_cal->irpd_correction  > 0) ic = si114x_cal->irpd_correction;
    }

    op.op1 = ref_ucoef[0] + ((ref_ucoef[1])<<8);
    op.op2 = vc;
    long_temp   = fx20_multiply( &op );
    out_ucoef[0] = (long_temp & 0x00ff);
    out_ucoef[1] = (long_temp & 0xff00)>>8;

    op.op1 = ref_ucoef[2] + (ref_ucoef[3]<<8);
    op.op2 = ic;
    long_temp   = fx20_multiply( &op );
    out_ucoef[2] = (long_temp & 0x00ff);
    out_ucoef[3] = (long_temp & 0xff00)>>8;

    //DEBUG_UCOEF

    //response = Si114xBlockWrite( si114x_handle, REG_UCOEF0 , 4, out_ucoef);
    WriteByte(SI114X_UCOEFF0, out_ucoef[0]);
    WriteByte(SI114X_UCOEFF0, out_ucoef[1]);
    WriteByte(SI114X_UCOEFF0, out_ucoef[2]);
    WriteByte(SI114X_UCOEFF0, out_ucoef[3]);

    //return response;
}
