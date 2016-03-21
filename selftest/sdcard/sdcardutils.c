//#include "board-v3.h"
#include "sdcard.h"
//#include "board-v3.h"

/*
 * Lights up a green or red LED
 */
void setLEDstate(int state)
{
    digitalWrite(CARD_LEDS, (state == 0));
}



