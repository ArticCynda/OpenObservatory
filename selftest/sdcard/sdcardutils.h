#ifndef _SDCARD_H_
    #define _SDCARD_H_

    #include "../O2_Rev3.h"
    
   

    #include "Arduino.h"

    #define cardInserted() (digitalRead(O2_SD_DETECT) == 0)
    #define cardProtected() (digitalRead(O2_SD_WP) != 0)

    //#include "board-v3.h"

    void setLEDstate(int state);
    //bool initCard(SdCard *card);

#endif
