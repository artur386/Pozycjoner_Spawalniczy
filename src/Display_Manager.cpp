#include "Display_Manager.h"

Display_ManagerClass::Display_ManagerClass(LiquidCrystal_I2C *_lcd, parameterStruct *params, const byte *maxParam)
{
    lcd = _lcd;
    // lcd = new LiquidCrystal_I2C(0x27, LCD_ROWS, LCD_COLS); // inicjalizacja wyświetlacza lcd
    lcd->clear(); // czyszczenie wyświetlacza lcd
    lcd->backlight();
    lcd->createChar(0, FI_CHAR);

    this->Parameters = params;
    this->MAX_PARAM = maxParam;
    // lcd->createChar(1, this->dash);

    this->LAST_DISPLAY_SCREEN = 127;
    this->DISPLAY_SCREEN = 0;
    this->QUICK_PARAM_NB = 0;
}

// void menuInit()
// {
//     menu = LcdMenu(LCD_ROWS, LCD_COLS);
//     menu->passLcdWithMenu(this->lcd, mainMenu);
// }

void Display_ManagerClass::updateScreen()
{
    if (this->DISPLAY_SCREEN != this->LAST_DISPLAY_SCREEN)
    {
        this->DISPLAY_SCREEN = this->LAST_DISPLAY_SCREEN;
        switch (this->DISPLAY_SCREEN)
        {
        case 0:
            MENU_IS_ON = false;
            drawMainScreen();
            break;

        default:
            MENU_IS_ON = true;
            drawMenu();
            break;
        }
    }
}

// Declare the call back function
void toggleBacklight();

void Display_ManagerClass::drawMenu()
{
}

void Display_ManagerClass::drawMainScreen()
{

    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print(F(" STA:"));
    lcd->setCursor(0, 1);
    lcd->print(F("READ:"));
    lcd->setCursor(0, 2);
    lcd->print(F(" DIA:"));
    lcd->setCursor(0, 3);
    lcd->print(F(" SET:"));
}
void Display_ManagerClass::UpdateStatus(byte status, bool ccw)
{
    if (status == 4)
    {
        status = 5;
    }
    if (status == 3 && ccw == true)
    {
        status = 4;
    }

    lcd->setCursor(6, 0);
    switch (status)
    {
    case 0:
        lcd->print(F("MOTOR STOP  "));
        break;
    case 1:
        lcd->print(F("PAUSE       "));
        break;
    case 2:
        lcd->print(F("SOFT START  "));
        break;
    case 3:
        lcd->print(F("OBR. W PRAWO"));
        break;
    case 4:
        lcd->print(F("OBR. W LEWO "));
        break;
    case 5:
        lcd->print(F("SMOOTH STOP "));
        break;
    default:
        break;
    }
}
void Display_ManagerClass::UpdateDiameterLcd(int diaVal)
{
    lcd->setCursor(5, 2);
    if (diaVal < 100)
    {
        lcd->print(' ');
    }
    lcd->print(diaVal);
}
void Display_ManagerClass::UpdateReadLcd(float readVal)
{
    lcd->setCursor(5, 1);
    if (readVal < 10.0)
    {
        lcd->print(' ');
    }
    lcd->print(readVal, 1);
}
void Display_ManagerClass::UpdateSetLcd(float setVal)
{
    lcd->setCursor(5, 3);
    if (setVal < 10.0)
    {
        lcd->print(' ');
    }
    lcd->print(setVal, 1);
}
void Display_ManagerClass::UpdateTime()
{
    this->timeold = millis();
}
void Display_ManagerClass::cutText()
{

    struct Cursor
    {
        byte x;
        byte y;
    };

    Cursor Cur1;

    lcd->clear();
    String bufor[4];

    // String text[4] = {
    //     "  _____   ______  _____  _______ _     _  _____  ",
    //     " |_____] |_____/ |     | |______ |____/  |     | ",
    //     " |       |    |_ |_____| |______ |    |_ |_____| ",
    //     "                                                 "};
    String text[4] = {
        "  _____  _____  _____  _____  _____  _____  ",
        " |  _  || __  ||     ||   __||  |  ||     | ",
        " |   __||    -||  |  ||   __||    -||  |  | ",
        " |__|   |__|__||_____||_____||__|__||_____| "};
    while (true)
    {
        /* code */

        Cur1.x = 20;
        Cur1.y = 0;
        int START_TEXT_INDEX = 0;
        int STOP_TEXT_INDEX = 0;
        int TextLen = text[0].length();
        int LCD_Width = 20;

        for (int i = (TextLen + LCD_Width * 2 + 1); i > 0; i--)
        {
            if ((bool)Cur1.x)
            {
                Cur1.x--;
            }
            if ((STOP_TEXT_INDEX - START_TEXT_INDEX) < LCD_Width)
            {
                STOP_TEXT_INDEX++;
            }
            else if ((STOP_TEXT_INDEX - START_TEXT_INDEX) == LCD_Width)
            {
                if (STOP_TEXT_INDEX != TextLen)
                {
                    STOP_TEXT_INDEX++;
                    START_TEXT_INDEX++;
                }
                else
                {
                    START_TEXT_INDEX++;
                }
            }
            for (size_t j = 0; j < 4; j++)
            {
                Cur1.y = j;
                lcd->setCursor(Cur1.x, Cur1.y);
                lcd->print(text[j].substring(START_TEXT_INDEX, STOP_TEXT_INDEX));
                // Serial.println(text[j].substring(START_TEXT_INDEX, STOP_TEXT_INDEX));
            }
            delay(350);
        }
    }
}

// Display_ManagerClass Display_Manager;