#ifndef _BUTTON_H_
#define _BUTTON_H_

#include "Arduino.h"

class Buttons
{
private:
    uint8_t BUTTON_PIN;
    uint16_t LONG_PRESS_TIME;
    long buttonTimer = 0;

    boolean buttonActive = false;
    boolean longPressActive = false;

public:
    Buttons(uint8_t BUTTON_PIN);
    uint8_t getButton();
};

#endif
