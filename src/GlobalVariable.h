#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H
#include "Arduino.h"

extern byte BT_ST;

extern uint8_t AppMode;
extern uint8_t LastAppMode;

extern volatile int8_t master_count;
extern unsigned long ScreenSaverTime, ScreenRefreshTime;
extern bool ScreenSaverOn, ShowScreenSaver;
extern unsigned long CONTROL_ROTARY_SW_press_time;
extern unsigned long CONTROL_START_STOP_press_time;
extern bool CONTROL_START_STOP_LONG_PRESS_DETECT;
extern bool CONTROL_ROTARY_SW_LONG_PRESS_DETECT;
extern bool CONTROL_ROTARY_SW_prev;
extern bool CONTROL_START_STOP_prev;
extern bool WriteToEEprom_flag;
extern unsigned long LastEEpromWriteTime;
// status pracy silnika
extern uint8_t MOTOR_STATE, LAST_MOTOR_STATE;
//kierunek obrotow silnika
extern bool MOTOR_CCW_DIR;
extern bool focus;
// extern int pwm;
extern float ActualRPM;

/* PARAMETRY MENU */
// extern double paramLastRPM, paramRPS, paramMMSEC;
// // extern uint32_t paramDIA;
// extern uint8_t paramGEAR;
// extern uint32_t paramTimePause, paramTimeStart, paramTimeStop;
// extern double paramKp, paramKi, paramKd;
// extern uint8_t paramSpeedMethod;
// extern bool paramPidOnOff;
// extern uint16_t paramScreenSaver;

// pid parameters:-
extern double Setpoint, Input, Output;

extern bool PidOnOff;
extern uint16_t open_loop_pwm;

extern uint8_t LED_ACCEPT_BLINK_CNT;
extern unsigned long LedAcceptBlinkLastTime;
extern bool LedAcceptIsOn, LedAcceptFlag, LedAcceptMode;
extern uint8_t LedAcceptCnt;

// Variables will change:
extern uint8_t ledState;
extern unsigned long previousMillis;
extern long interval;
extern bool blinkStart;
extern uint8_t blinkCount;
extern uint8_t cursorFocus;

extern bool startFlag;

// extern int values[11];
// extern uint8_t enu[3];

//RPM
extern volatile uint8_t ENC_STATE;
extern uint32_t isrTime;
extern double RealRpm, SetRpm, RealMMsec, outPWM;
extern bool READ_RPM, MotorStartFlag, MotorRunFlag, MotorStopFlag, MotorPauseFlag, MotorSoftStartFlag, MotorSmoothStopFlag, softStartOn, smoothStopOn;
extern uint16_t setPWM, smoothStopPWM, softStartPWM;
extern unsigned int LastSoftStartTime, LastSmoothStopTime;

// tachometer
extern long lastUpdate;                  // for timing display updates
extern volatile long accumulator;        // sum of last 8 revolution times
extern volatile unsigned long startTime; // start of revolution in microseconds
extern volatile unsigned long revCount;  // number of revolutions since last display update
extern uint8_t averageCnt, averageCntMax;
extern unsigned long revCountCopy;      // number of revolutions since last display update
extern unsigned long revCountSmooth[5]; // number of revolutions since last display update

// MAX VARIABLE
extern uint16_t LAST_DIA;
extern uint16_t CURR_DIA;
extern uint16_t LAST_VC;
extern uint16_t CURR_VC;
extern uint16_t LAST_RPM;
extern uint16_t CURR_RPM;

extern struct Parametr parameters[13];

#endif // !GLOBAL_VARIABLE_H
