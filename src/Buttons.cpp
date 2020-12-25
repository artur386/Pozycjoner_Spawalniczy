#include "Buttons.h"

Buttons::Buttons(uint8_t BUTTON_PIN)
{
    this->BUTTON_PIN = BUTTON_PIN;
    this->LONG_PRESS_TIME = 1000;
    // this->SHORT_PRESS_TIME = 250;
    pinMode(this->BUTTON_PIN, INPUT);
};

uint8_t Buttons::getButton()
{

    if (digitalRead(this->BUTTON_PIN) == HIGH)
    {

        if (this->buttonActive == false)
        {

            this->buttonActive = true;
            this->buttonTimer = millis();
        }

        if ((millis() - this->buttonTimer > this->LONG_PRESS_TIME) && (this->longPressActive == false))
        {

            this->longPressActive = true;
            return 2;
        }
    }
    else
    {

        if (this->buttonActive == true)
        {
            this->buttonActive = false;
            if (this->longPressActive == true)
            {
                this->longPressActive = false;
            }
            else
            {
                return 1;
            }
        }
    }
    return 0;
};
