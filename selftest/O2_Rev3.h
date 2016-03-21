#ifndef _O2BOARD_H_
#define _O2BOARD_H_

  #include "Arduino.h"

  // On board communication protocols: IIC, UART, SPI
  #define O2_SDA            A4     // I/O     IIC Serial Data & Address
  #define O2_SCL            A5     // OUTPUT  IIC Serial Clock Line
  #define O2_RX             0      // INPUT   UART Receiver
  #define O2_TX             1      // OUTPUT  UART Transmitter
  #define O2_MOSI           11     // OUTPUT  SPI Master Out, Slave In
  #define O2_MISO           12     // INPUT   SPI Master In, Slave Out
  #define O2_SCK            13     // OUTPUT  SPI Clock
  
  #define O2_SD_LED         9      // OUTPUT  SD card state indicator LEDs
  #define O2_SD_DETECT      5      // INPUT   SD card inserted
  #define O2_SD_WP          6      // INPUT   SD card write protected switch
  #define O2_SD_SS          10     // OUTPUT  SD card Slave Select
  
  #define O2_CO2_HEATER     A0     // OUTPUT  CO2 sensor heater enable
  #define O2_CO2_SENSE      A1     // INPUT   CO2 sensor analog input
  
  #define O2_USB_CONFIG     2      // INPUT   USB initialization complete
  #define O2_USB_SUSPEND    3      // INPUT   USB communication suspended by host
  
  #define O2_MCU_LED        4      // OUTPUT  MCU general purpose LED
  
  #define O2_RH_SENSE       7      // I/O     Humidity sensor
  
  #define ERR 0
  #define OK 1
  
  #define SERIAL_SPEED      19200  // baud rate for serial communication over UART

#endif
