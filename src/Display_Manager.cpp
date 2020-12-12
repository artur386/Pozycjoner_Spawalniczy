#include "Display_Manager.h"

void Display_ManagerClass::init(LiquidCrystal_I2C *_p_lcd)
{
    p_lcd = _p_lcd;
    p_lcd->init();  // inicjalizacja wyświetlacza lcd
    p_lcd->clear(); // czyszczenie wyświetlacza lcd
    p_lcd->backlight();
}

void Display_ManagerClass::setCurrentMenuPos(byte *CURRENT_MENU_POS)
{
    this->CURRENT_MENU_POS = CURRENT_MENU_POS;
}

void Display_ManagerClass::setLastMenuPos(byte *CURRENT_MENU_POS)
{
    this->LAST_MENU_POS = LAST_MENU_POS;
}

void Display_ManagerClass::setCurrentMenuLevel(byte *CURRENT_MENU_LEVEL)
{
    this->CURRENT_MENU_LEVEL = CURRENT_MENU_LEVEL;
}

void Display_ManagerClass::setLastMenuLevel(byte *LAST_MENU_LEVEL)
{
    this->LAST_MENU_LEVEL = LAST_MENU_LEVEL;
}

void Display_ManagerClass::setMenuIsOn(bool *MENU_IS_ON)
{
    this->MENU_IS_ON = MENU_IS_ON;
}
void Display_ManagerClass::setHomeScreenCursorPos(byte *HOME_SCREEN_CURSOR_POSITION)
{
    this->HOME_SCREEN_CURSOR_POSITION = HOME_SCREEN_CURSOR_POSITION;
}
void Display_ManagerClass::drawMainScreen()
{
    p_lcd->print("test1");
}

Display_ManagerClass Display_Manager;