#ifndef EnumTypes
#define EnumTypes

#include "Arduino.h"

enum MotorRunState
{
    MT_STOP,
    MT_PAUSE,
    MT_SOFT_START,
    MT_RUN_CW,
    MT_RUN_CCW,
    MT_SMOOTH_STOP,
    MT_TURN_IT_OFF
};

extern MotorRunState MotorState;

enum BPM
{
    NO_PRESS,
    TRIGER_SHORT_PRESS,
    TRIGER_LONG_PRESS,
    UP,
    DOWN,
    LEFT,
    RIGHT,
    ENC_SHORT_PRESS,
    ENC_LONG_PRESS
};

extern BPM BUTTON_PRESSED;

enum AppModeValues
{
    APP_NORMAL_MODE,
    APP_MENU_MODE,
    APP_PROCESS_MENU_CMD
};

extern AppModeValues APP_MODE;

#endif // !MOTORRUNSTATE
