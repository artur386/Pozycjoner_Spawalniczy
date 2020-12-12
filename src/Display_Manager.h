#ifndef _DISPLAY_MANAGER_h
#define _DISPLAY_MANAGER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#endif

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

class Display_ManagerClass
{
protected:
private:
    LiquidCrystal_I2C *p_lcd;
    //Parametry menu:
    byte *CURRENT_MENU_POS;
    byte *LAST_MENU_POS;
    byte *CURRENT_MENU_LEVEL;
    byte *LAST_MENU_LEVEL;

    bool *MENU_IS_ON;
    byte *HOME_SCREEN_CURSOR_POSITION;

    unsigned long *BUTTON_LAST_PRESS_TIME;
    unsigned int *BACK_TO_MAIN_SCREEN_TIME;

public:
    //void init();
    void init(LiquidCrystal_I2C *_p_lcd);
    void setCurrentMenuPos(byte *CURRENT_MENU_POS);
    void setLastMenuPos(byte *CURRENT_MENU_POS);
    void setCurrentMenuLevel(byte *CURRENT_MENU_LEVEL);
    void setLastMenuLevel(byte *LAST_MENU_LEVEL);
    void setMenuIsOn(bool *MENU_IS_ON);
    void setHomeScreenCursorPos(byte *HOME_SCREEN_CURSOR_POSITION);
    void drawMainScreen();
    void drawMenu();
};

extern Display_ManagerClass Display_Manager;

#endif
