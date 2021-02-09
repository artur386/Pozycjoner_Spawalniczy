#include "DisplayManager.h"

DisplayManager::DisplayManager(LiquidCrystal_I2C *_lcd, uint8_t *MotorSt)
{
    this->lcd = _lcd;
    this->MOTOR_STATE = MotorSt;
    this->lastRefreshTime = millis();
    this->LastMotorState = 255;
    this->LastParamDIA = 65000;
    this->LastParamRPM = 999;
    this->LastParamRPS = 999;
    this->LastRealRPM = 99;
    this->LastParamMMSEC = 999;
    // lcd = new LiquidCrystal_I2C(0x27, LCD_ROWS, LCD_COLS); // inicjalizacja wyświetlacza lcd
    // lcd->createChar(0, FI_CHAR);
};

void DisplayManager::SetREAL_RPM_P(double *realRpm)
{
    REAL_RPM = realRpm;
}
void DisplayManager::drawMainScreen()
{

    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print(F(" STA:"));
    lcd->setCursor(0, 1);
    lcd->print(F("READ:"));
    if (*paramSpeedMethod == 2)
    {
        lcd->setCursor(0, 2);
        lcd->print(F(" DIA:"));
    }
    lcd->setCursor(0, 3);
    lcd->print(F(" SET:"));
}
void DisplayManager::Update_MOT_Status()
{
    if (this->LastMotorState != *this->MOTOR_STATE)
    {
        lcd->setCursor(6, 0);
        switch (int(*this->MOTOR_STATE))
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
        case 6:
            lcd->print(F("GO TO STOP  "));
            break;
        default:
            break;
        }
        this->LastMotorState = *this->MOTOR_STATE;
    }
}

void DisplayManager::UpdateSetDIA()
{
    if (this->LastParamDIA != *this->paramDIA)
    {
        char *buf = (char *)malloc(sizeof(char) * 7);
        sprintf(buf, "%3d mm", *this->paramDIA);
        lcd->setCursor(7, 2);
        lcd->print(buf);
        free(buf);
        this->LastParamDIA = *this->paramDIA;
    }
}
void DisplayManager::BindSpeed(double *paramRPM, double *paramRPS, double *paramMMSEC)
{
    this->paramRPM = paramRPM;
    this->paramRPS = paramRPS;
    this->paramMMSEC = paramMMSEC;
}
void DisplayManager::BindDia(uint32_t *paramDIA)
{
    this->paramDIA = paramDIA;
}
void DisplayManager::ForceRefresh()
{
    this->LastMotorState = 255;
    this->LastParamDIA = 65000;
    this->LastParamRPM = 999;
    this->LastParamRPS = 999;
    this->LastRealRPM = 99;
    this->LastParamMMSEC = 999;
}
void DisplayManager::BindSpeedMethod(uint8_t *paramSpeedMethod)
{
    this->paramSpeedMethod = paramSpeedMethod;
}
void DisplayManager::DrawRealSPEED()
{
    if (this->LastRealRPM != *this->REAL_RPM)
    {
        this->LastRealRPM = *this->REAL_RPM;
        char *buf = (char *)malloc(sizeof(char) * 15);
        char *realValCh = (char *)malloc(sizeof(char) * 6);
        char *unit = (char *)malloc(sizeof(char) * 6);
        float val;
        if (*paramSpeedMethod == 0)
        {
            val = *this->REAL_RPM;
            sprintf(unit, "RPM");
        }
        else if (*paramSpeedMethod == 1)
        {
            val = *this->REAL_RPM / 60.0f;
            sprintf(unit, "RPS");
        }
        else if (*paramSpeedMethod == 2)
        {
            val = M_PER_MIN_to_MM_SEC(RPM_TO_VC(*this->REAL_RPM));
            sprintf(unit, "MM/SEC");
        }
        dtostrf(val, 1, 2, realValCh);
        sprintf(buf, "%5s %s", realValCh, unit);
        lcd->setCursor(5, 1);
        lcd->print(buf);
        free(realValCh);
        free(buf);
        free(unit);
    }
}
void DisplayManager::UpdateSetSPEED()
{
    if (*this->paramRPM != this->LastParamRPM || *this->paramRPS != this->LastParamRPS || *this->paramMMSEC != this->LastParamMMSEC)
    {
        char *buf = (char *)malloc(sizeof(char) * 15);
        char *realValCh = (char *)malloc(sizeof(char) * 6);
        char *unit = (char *)malloc(sizeof(char) * 6);
        float val;
        this->LastParamRPM = *this->paramRPM;
        this->LastParamRPS = *this->paramRPS;
        this->LastParamMMSEC = *this->paramMMSEC;

        if (*paramSpeedMethod == 0)
        {
            val = *this->paramRPM;
            sprintf(unit, "RPM");
        }
        else if (*paramSpeedMethod == 1)
        {
            val = *this->paramRPS;
            sprintf(unit, "RPS");
        }
        else if (*paramSpeedMethod == 2)
        {
            val = *this->paramMMSEC;
            sprintf(unit, "MM/SEC");
        }
        dtostrf(val, 1, 2, realValCh);
        sprintf(buf, "%5s %s", realValCh, unit);
        lcd->setCursor(7, 3);
        lcd->print(buf);

        free(realValCh);
        free(buf);
        free(unit);
    }
}
void DisplayManager::UpdateCursor()
{
    if (this->LastCursorState != *this->CursorState)
    {
        if (*this->CursorState)
        {
            lcd->setCursor(5, 2);
            lcd->print(' ');
            lcd->setCursor(5, 3);
            lcd->write(0x7E);
        }
        else
        {
            lcd->setCursor(5, 2);
            lcd->write(0x7E);
            lcd->setCursor(5, 3);
            lcd->print(' ');
        }
        this->LastCursorState = *this->CursorState;
    }
}
void DisplayManager::updateScreen()
{
    // if ((millis() - lastRefreshTime) > refreshTime)
    // {
    //     lastRefreshTime = millis();

    this->UpdateSetSPEED();
    this->DrawRealSPEED();
    if (*paramSpeedMethod == 2)
    {
        this->UpdateSetDIA();
        this->UpdateCursor();
    }
    this->Update_MOT_Status();
    // }
}

void DisplayManager::bindFocus(bool *curFocus)
{
    this->CursorState = curFocus;
    this->LastCursorState = !(*this->CursorState);
}

// void DisplayManager::UpdateTime()
// {
//     this->timeold = millis();
// }
void DisplayManager::cutText()
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
