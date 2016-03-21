#ifndef _SDCARD_H_
    #define _SDCARD_H_

    #include "../board-v3.h"
    
   

    #include "Arduino.h"

    #define cardInserted() (digitalRead(CARD_DETECT) == 0)
    #define cardProtected() (digitalRead(CARD_WP) != 0)

    //#include "board-v3.h"

    void setLEDstate(int state);
    //bool initCard(SdCard *card);

#endif
