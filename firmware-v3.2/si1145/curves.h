#ifndef __SI1145_CURVES__
  #define __SI1145_CURVES__

  #define WM_COEFF

  #ifdef WM_COEFF
  
    #define IR_fA -6.87272
    #define IR_eA -11
    #define IR_fB -3.92322
    #define IR_eB -6
    #define IR_fC 0.0070775
    #define IR_eC 0
    #define IR_fD -3.41809
    #define IR_eD 0
    #define IR_fE 517.005
    #define IR_eE 0
    //#define IR_R2 0.9778090562
    
    #define VIS_fA -1.81964
    #define VIS_eA -8
    #define VIS_fB 0.000058374
    #define VIS_eB 0
    #define VIS_fC -0.0678693
    #define VIS_eC 0
    #define VIS_fD 33.3319
    #define VIS_eD 0
    #define VIS_fE -5600.49
    #define VIS_eE 0
    //#define VIS_R2 0.868897427
  #else
  
    #define IR_fA -1.39262756163917
    #define IR_eA -10
    #define IR_fB -3.72918430479665
    #define IR_eB -6
    #define IR_fC 0.0068849276
    #define IR_eC 0
    #define IR_fD -3.3364616849
    #define IR_eD 0
    #define IR_fE 504.5568994438
    #define IR_eE 0
    #define IR_R2 0.9778090562
    
    #define VIS_fA -1.83445880915405
    #define VIS_eA -8
    #define VIS_fB 5.88160816226291
    #define VIS_eB -5
    #define VIS_fC 0.068347328
    #define VIS_eC 0
    #define VIS_fD 33.5534376897
    #define VIS_eD 0
    #define VIS_fE 5637.4978888156
    #define VIS_eE 0
    #define VIS_R2 0.868897427

  #endif
  
  //#define RATIO_P 7.83585

  // calculate p
// 8.277 / 40 = 2734 / (279 * p)
// 40 and 279 are proportion values read from wavelenght graph at wavelength 875 nm (known wavelength from datasheet)
#define RATIO_P (2734 / 279) / (40 / 8.277)
 
 
#endif
