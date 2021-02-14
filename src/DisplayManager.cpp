#include "DisplayManager.h"

DisplayManager::DisplayManager(LiquidCrystal_I2C *_lcd, uint8_t *MotorSt)
{
    this->lcd = _lcd;
    this->MOTOR_STATE = MotorSt;
    this->lastRefreshTime = millis();
    this->LastMotorState = 255;
    this->LastParamDIA = 65000;
    this->LastParamRPM = 999;
    // this->LastParamRPS = 999;
    this->LastRealRPM = 99;
    this->LastParamMMSEC = 999;
    // lcd = new LiquidCrystal_I2C(0x27, LCD_ROWS, LCD_COLS); // inicjalizacja wyświetlacza lcd
    // lcd->createChar(0, FI_CHAR);
};

void DisplayManager::SetREAL_RPM_P(double *realRpm)
{
    RealRpm = realRpm;
}
void DisplayManager::drawMainScreen()
{

    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print(F(" STA:"));
    lcd->setCursor(0, 1);
    lcd->print(F("READ:"));
    if ((*(parameters_p + 11)).val == 1)
    {
        lcd->setCursor(0, 2);
        lcd->print(F(" DIA:"));
    }
    lcd->setCursor(0, 3);
    lcd->print(F(" SET:"));
}
void DisplayManager::BindLoadingBar(uint16_t *loadingBar)
{
    this->loadingBar = loadingBar;
}

void DisplayManager::Update_MOT_Status()
{
    if (this->LastMotorState != *this->MOTOR_STATE)
    {
        lcd->setCursor(6, 0);
        switch (int(*this->MOTOR_STATE))
        {
        case 0:
            lcd->print(F("MOTOR STOP"));
            break;
        case 1:
            lcd->print(F("PAUSE IN"));
            break;
        case 2:
            lcd->print(F("MOTOR START"));
            break;
        case 3:
            lcd->print(F(" MOTOR CW"));
            break;
        case 4:
            lcd->print(F("MOTOR CCW"));
            break;
        case 5:
            lcd->print(F("SMOOTH OUT"));
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
    if (this->LastParamDIA != (*(parameters_p + 2)).val)
    {
        char *buf = (char *)malloc(sizeof(char) * 7);
        sprintf(buf, "%3d mm", (*(parameters_p + 2)).val);
        lcd->setCursor(7, 2);
        lcd->print(buf);
        free(buf);
        this->LastParamDIA = (*(parameters_p + 2)).val;
    }
}
// void DisplayManager::BindLoadBar(double *paramRPM, double *paramRPS, double *paramMMSEC)
// {
//     this->paramRPM = paramRPM;
//     this->paramRPS = paramRPS;
//     this->paramMMSEC = paramMMSEC;
// }
// void DisplayManager::BindDia(uint32_t *paramDIA)
// {
//     this->paramDIA = paramDIA;
// }
void DisplayManager::ForceRefresh()
{
    this->LastMotorState = 255;
    this->LastParamDIA = 65000;
    this->LastParamRPM = 999;
    this->LastRealRPM = 99;
    this->LastParamMMSEC = 999;
}
// void DisplayManager::BindSpeedMethod(uint8_t *paramSpeedMethod)
// {
//     this->paramSpeedMethod = paramSpeedMethod;
// }
void DisplayManager::BindParameters(Parametr *parameters_p)
{
    this->parameters_p = parameters_p;
}
void DisplayManager::DrawRealSPEED()
{
    if (this->LastRealRPM != *this->RealRpm)
    {
        this->LastRealRPM = *this->RealRpm;
        char *buf = (char *)malloc(sizeof(char) * 15);
        char *realValCh = (char *)malloc(sizeof(char) * 6);
        char *unit = (char *)malloc(sizeof(char) * 6);
        float val;
        if ((*(parameters_p + 11)).val == 0)
        {
            val = *this->RealRpm;
            sprintf(unit, "RPM");
        }
        else if ((*(parameters_p + 11)).val == 1)
        {
            val = RPM_TO_MMSEC(*this->RealRpm, (*(parameters_p + 2)).val);
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
    if ((*(parameters_p)).val != this->LastParamRPM || (*(parameters_p + 1)).val != this->LastParamMMSEC)
    {
        char *buf = (char *)malloc(sizeof(char) * 15);
        char *realValCh = (char *)malloc(sizeof(char) * 6);
        char *unit = (char *)malloc(sizeof(char) * 6);
        float val;
        uint8_t prec;
        this->LastParamRPM = (*(parameters_p + 0)).val;
        // this->LastParamRPS = *this->paramRPS;
        this->LastParamMMSEC = (*(parameters_p + 1)).val;

        if ((*(parameters_p + 11)).val == 0)
        {
            sprintf(unit, "RPM");
        }

        else if ((*(parameters_p + 11)).val == 1)
        {
            sprintf(unit, "MM/SEC");
        }
        val = (*(parameters_p + (*(parameters_p + 11)).val)).val;
        prec = (*(parameters_p + (*(parameters_p + 11)).val)).precision;
        // DBG(val);
        // DBG(prec);
        dtostrf((val / pow(10, prec)), 1, prec, realValCh);
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
    if ((*(parameters_p + 11)).val == 1)
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
void DisplayManager::DrawLoadingBar(int dc, int y = 1)
{
    uint8_t load;
    if (dc > 100)
    {
        dc = 100;
    }
    else if (dc < 0)
    {
        dc = 0;
    }
    load = map(dc, 0, 100, 0, 17);
    // lcd->setCursor(0, y);
    // lcd->print(F("                    "));
    for (size_t i = 3; i < 20; i++)
    {
        lcd->setCursor(i, y);
        if (i <= load + 3)
        {
            lcd->write(0xff);
        }
        else
        {
            lcd->write(0x20);
        }
    }
    char *buf = (char *)malloc(sizeof(char) * 5);
    sprintf(buf, "%2d%%", dc);
    lcd->setCursor(0, y);
    lcd->print(buf);
    free(buf);
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
