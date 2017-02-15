//#include "board-v3.h"
//#include "SdCard.h"
//#include "board-v3.h"
#include "../O2_Rev3.h"

/*
 * Lights up a green or red LED
 */
void setLEDstate(int state)
{
    digitalWrite(O2_SD_LED, (state == 0));
}



