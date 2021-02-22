#ifndef _DISPLAYMANAGER_h
#define _DISPLAYMANAGER_h

#include <Arduino.h>

// #include "MenuItem.h"
#include <Wire.h>
#include <LiquidCrystal.h>
#include "enums.h"
#include "Parametr.h"

// extern MenuItem mainMenu[];
class DisplayManager
{
private:
    LiquidCrystal *lcd;
    uint8_t *GEAR;
    uint16_t refreshTime = 1000;
    double *RealRpm, LastRealRPM;
    uint16_t divSetRpm = 10, divRealRpm = 10;
    unsigned long lastRefreshTime, cursorLastTime;
    unsigned long curTimeBlink = 300;
    bool *CursorState;
    bool LastCursorState;
    double LastParamRPM, LastParamMMSEC;
    int LastParamDIA;
    uint16_t *loadingBar;
    Parametr *parameters_p;
    // custom char

    uint8_t *MOTOR_STATE, LastMotorState;

public:
    //void init();
    DisplayManager(LiquidCrystal *_lcd, uint8_t *motorState);
    void SetREAL_RPM_P(double *realRpm);
    void BindParameters(Parametr *parameters_p);
    void BindLoadingBar(uint16_t *loadingBar);

    // void BindSpeed(double *paramRPM, double *paramRPS, double *paramMMSEC);
    // void BindDia(uint32_t *paramDIA);
    // void BindSpeedMethod(uint8_t *paramSpeedMethod);

    void Update_MOT_Status();
    void UpdateSetDIA();
    void DrawRealSPEED();
    void UpdateSetSPEED();
    void drawMainScreen();
    void updateScreen();
    void Display_Manage();
    void cutText();
    void bindVariables(unsigned long *timeold);
    void UpdateTime();
    void bindFocus(bool *curFocus);
    void UpdateCursor();
    void ForceRefresh();
    void DrawLoadingBar(int dc, int y);
};

// extern DisplayManagerClass DisplayManager;

#endif
