#ifndef _DISPLAY_MANAGER_h
#define _DISPLAY_MANAGER_h

#include <Arduino.h>

// #include "MenuItem.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Variable.h"

// extern MenuItem mainMenu[];
class Display_ManagerClass
{
protected:
private:
    LiquidCrystal_I2C *lcd;
    parameterStruct *Parameters;
    const byte *MAX_PARAM;

    // custom char
    byte FI_CHAR[8] = {
        B00100,
        B01110,
        B10101,
        B10101,
        B10101,
        B01110,
        B00100,
        B00000};
    // byte dash[] = {
    //     0x00,
    //     0x10,
    //     0x08,
    //     0x04,
    //     0x02,
    //     0x01,
    //     0x00,
    //     0x00};

    //Parametry menu i wyswietlacza:
    uint8_t LAST_DISPLAY_SCREEN;
    uint8_t DISPLAY_SCREEN;
    bool MENU_IS_ON;
    uint8_t QUICK_PARAM_NB;

    unsigned long timeold;
    unsigned long BUTTON_LAST_PRESS_TIME;
    unsigned int BACK_TO_MAIN_SCREEN_TIME;

    uint8_t *motorState;
    unsigned int *pause_before_time;
    unsigned int *soft_start_time;
    unsigned int *soft_stop_time;
    unsigned int *diameter;

public:
    int *TEMP_VAL_PRT;
    //void init();
    Display_ManagerClass();
    // void menuInit();

    void SetLcd(LiquidCrystal_I2C *lcd);
    void SetParam(parameterStruct *params, const byte *maxParam);
    void BindMotorState(uint8_t *MotorSt);
    // void setStruct(struct MenuItem *_p_menu);
    void setTempVal(int *_tempVal);
    void setCurrentMenuPos(byte *CURRENT_MENU_POS);
    void setLastMenuPos(byte *LAST_MENU_POS);
    void setCurrentMenuLevel(byte *CURRENT_MENU_LEVEL);
    void setLastMenuLevel(byte *LAST_MENU_LEVEL);
    void setMenuIsOn(bool *MENU_IS_ON);
    void setHomeScreenCursorPos(byte *HOME_SCREEN_CURSOR_POSITION);
    void Update_MOT_Status(uint8_t ms);
    void UpdateDiameterLcd(int diaVal);
    void UpdateReadLcd(float readVal);
    void UpdateSetLcd(float setVal);
    void drawMainScreen();
    void updateScreen();
    // void drawMainScreen(byte motorState, int motorStateValue, int diameter, int setSpeed, int readSpeed);
    void drawMenu();
    void Display_Manage();
    void cutText();
    void bindVariables(unsigned long *timeold);
    void UpdateTime();
    // void CreateCustomChar();
};

// extern Display_ManagerClass Display_Manager;

#endif
