#ifndef _BOARDV3_H_
#define _BOARDV3_H_


  #define DHTPIN 7 // connection for DHT-11
  
  #define USB_SUS 2 // USB suspend on PD2 pin 32
  #define USB_CONF 3  // USB config on PD3 pin 1
  
  #define MQPIN A1 // MQ-135 pin
  
  #define CARD_LEDS A0 // card ok LEDs on PC0 (ADC0) pin 23
  #define CARD_DETECT 5  // card detection input on PD5, pin 9
  #define CARD_WP   6  // card write protection input on PD6, pin 10
  
  
  
  #define DHTTYPE DHT11 // type of sensor
  
  
  #define ERR 0
  #define OK 1

  #define SERIAL_SPEED 19200


    
#endif
