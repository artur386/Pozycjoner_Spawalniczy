#include <Arduino.h>
#include "BigFont.h"
#include "RTClib.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include "enums.h"
#include "myMenu.h"
#include "DisplayManager.h"
#include "Motor.h"
#include <TimerOne.h>
#include "Parametr.h"
#include "Funkcje.h"
#include "GlobalVariable.h"

/*  DEFINE PARAMETERS */
#define SET_DEFAULT 1

/***********************************************************
 *            PROTOTYPES                                   *
 * ********************************************************/

/*************************************/

byte BT_ST;

uint8_t AppMode;
uint8_t LastAppMode;

volatile int8_t master_count;
unsigned long ScreenSaverTime, ScreenRefreshTime;
bool ScreenSaverOn, ShowScreenSaver;
unsigned long CONTROL_ROTARY_SW_press_time;
unsigned long CONTROL_START_STOP_press_time;
bool CONTROL_START_STOP_LONG_PRESS_DETECT;
bool CONTROL_ROTARY_SW_LONG_PRESS_DETECT;
bool CONTROL_ROTARY_SW_prev;
bool CONTROL_START_STOP_prev;
bool WriteToEEprom_flag;
unsigned long LastEEpromWriteTime;
// status pracy silnika
uint8_t MOTOR_STATE, LAST_MOTOR_STATE;
//kierunek obrotow silnika
bool MOTOR_CCW_DIR;
bool focus;
// int pwm;
float ActualRPM;

/* PARAMETRY MENU */
// double paramRPM, paramLastRPM, paramRPS, paramMMSEC;
// uint32_t paramDIA;
// uint8_t paramGEAR;
// uint32_t paramTimePause, paramTimeStart, paramTimeStop;
// double paramKp, paramKi, paramKd;
// uint8_t paramSpeedMethod;
// bool paramPidOnOff;
// uint16_t paramScreenSaver;

// pid parameters:-
// double Setpoint, Input, Output;

bool PidOnOff;
uint16_t open_loop_pwm;

uint8_t LED_ACCEPT_BLINK_CNT;
unsigned long LedAcceptBlinkLastTime;
bool LedAcceptIsOn, LedAcceptFlag, LedAcceptMode;
uint8_t LedAcceptCnt;

// Variables will change:
uint8_t ledState;
unsigned long previousMillis;
long interval;
bool blinkStart;
uint8_t blinkCount;
uint8_t cursorFocus;

bool startFlag;

//RPM
volatile uint8_t ENC_STATE;
uint32_t isrTime;
double RealMMsec;
double RealRpm, SetRpm, outPWM;
bool READ_RPM, MotorStartFlag, MotorRunFlag, MotorStopFlag, MotorPauseFlag, MotorSoftStartFlag, MotorSmoothStopFlag, softStartOn, smoothStopOn;
uint16_t loadingBar;
uint16_t setPWM, smoothStopPWM, softStartPWM;
unsigned int LastSoftStartTime, LastSmoothStopTime;
// tachometer
long lastUpdate;                  // for timing display updates
volatile long accumulator;        // sum of last 8 revolution times
volatile unsigned long startTime; // start of revolution in microseconds
volatile unsigned long revCount;  // number of revolutions since last display update
uint8_t averageCnt, averageCntMax;
unsigned long revCountCopy;      // number of revolutions since last display update
unsigned long revCountSmooth[5]; // number of revolutions since last display update

// MAX VARIABLE
uint16_t LAST_DIA;
uint16_t CURR_DIA;
uint16_t LAST_VC;
uint16_t CURR_VC;
uint16_t LAST_RPM;
uint16_t CURR_RPM;

struct Parametr parameters[13];
byte PWMChar[] = {
    0x01,
    0x01,
    0x1F,
    0x10,
    0x10,
    0x10,
    0x1F,
    0x01};
// define object :-
// LiquidCrystal_I2C lcd(0x27, 20, 4);
// PID myPID(&Input, &Output, &Setpoint, paramKp, paramKi, paramKd, DIRECT);
// DisplayManager Display(&lcd, &MOTOR_STATE);
// Motor motor(RL_MOTOR_CW, RL_MOTOR_CCW, RL_MOTOR_GEAR, RL_MOTOR_START_STOP, PWM_DAC);
// myMenu menu(&lcd, &values[0], &enu[0], &BT_ST, &AppMode);
RTC_DS3231 rtc;
BigFont fontPrinter;

/***************************************************
 * functions
 * ************************************************/

/**************************************************
 * 
 *    SETUP:
 *      - INICJALIZACJA PARAMETROW.
 * 
 * ************************************************/
void setup()
{
  // REV = 0; //  START ALL THE VARIABLES FROM 0
  // Parametr parameters[13];
  BT_ST = 0x00000000;
  CONTROL_ROTARY_SW_press_time = millis();
  CONTROL_START_STOP_press_time = millis();
  CONTROL_START_STOP_LONG_PRESS_DETECT = false;
  CONTROL_ROTARY_SW_LONG_PRESS_DETECT = false;
  CONTROL_ROTARY_SW_prev = HIGH;
  CONTROL_START_STOP_prev = LOW;
  WriteToEEprom_flag = false;
  LastEEpromWriteTime = 0;
  MOTOR_CCW_DIR = false;
  focus = false;
  ledState = LOW;
  previousMillis = 0;
  interval = 1000;
  blinkStart = false;
  blinkCount = 0;
  cursorFocus = 0;
  MotorStartFlag = false;
  MotorRunFlag = false;
  MotorStopFlag = false;
  ENC_STATE = 0;
  isrTime = 500;

  READ_RPM = false;
  // tachometer
  lastUpdate = 0;  // for timing display updates
  accumulator = 0; // sum of last 8 revolution times
  startTime = 0;   // start of revolution in microseconds
  revCount = 0;    // number of revolutions since last display update
  averageCnt = 0;
  averageCntMax = 5;
  revCountCopy = 0;      // number of revolutions since last display update
  revCountSmooth[0] = 0; // number of revolutions since last display update
  revCountSmooth[1] = 0; // number of revolutions since last display update
  revCountSmooth[2] = 0; // number of revolutions since last display update
  revCountSmooth[3] = 0; // number of revolutions since last display update
  revCountSmooth[4] = 0; // number of revolutions since last display update
  Serial.begin(115200);
  Serial.println("start");
  ReadEEpromData();
  lcd.begin(20, 4);
  // lcd.clear(); // czyszczenie wyświetlacza lcd
  // lcd.backlight();
  // char testa[] = "test";
  // char *b = &testa[0];
  fontPrinter.BindLcd(&lcd);
  char const *p = " PROEKO ";
  fontPrinter.WriteBigString(p, 0, 1);

  // motor.BindMotorState(&MOTOR_STATE);
  // motor.SetTimes(&values[5], &values[6], &values[7]);
  // motor.SetGears(&enu[0]);
  // motor.BindCCWDir(&MOTOR_CCW_DIR);

  pinMode(MOTOR_ENCODER, INPUT);
  pinMode(ROT_ENC_CLK, INPUT_PULLUP);
  pinMode(ROT_ENC_DT, INPUT_PULLUP);
  pinMode(BTN_START_STOP_PANEL, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  AppMode = APP_IDLE_VIEW;
  LastAppMode = 100;
  MOTOR_STATE = MOTOR_STOP;
  LAST_MOTOR_STATE = 100;
  ENC_STATE = 0;
  READ_RPM = true;
  cursorFocus = 1;
  // pwm = 50;
  PidOnOff = !bool(parameters[12].val);
  //Initialize serial and wait for port to open:

  // pinMode(RS, OUTPUT);
  // pinMode(RW, OUTPUT);
  // pinMode(EN, OUTPUT);
  // pinMode(D0, OUTPUT);
  // pinMode(D1, OUTPUT);
  // pinMode(D2, OUTPUT);
  // pinMode(D3, OUTPUT);

  PWR_LED_ON;
  RED_LED_ON();
  GREEN_LED_OFF();
  // attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER), isr, CHANGE); //interrupt pin
  attachInterrupt(digitalPinToInterrupt(ROT_ENC_CLK), read_rotary, CHANGE);
  CONTROL_ROTARY_SW_press_time = millis();
  CONTROL_ROTARY_SW_prev = HIGH;

  Display.SetREAL_RPM_P(&RealRpm);
  Display.bindFocus(&focus);
  Display.BindParameters(&parameters[0]);
  Display.BindLoadingBar(&loadingBar);

  int a,
      b;
  for (;;)
  {
    a = 0;
    b = 100;
    for (int i = a; i < b; i++)
    {
      Display.DrawLoadingBar(i, 1);
      delay(100);
    }
    for (int i = b; i > a; i--)
    {
      Display.DrawLoadingBar(i, 1);
      delay(100);
    }
  }

  // Display.BindLoadBar(&paramRPM, &paramRPS, &paramMMSEC);
  // Display.BindDia(&paramDIA);
  // Display.BindSpeedMethod(&paramSpeedMethod);
  // Display.cutText();
  Timer1.initialize(128); // 40 us = 25 kHz
  Timer1.pwm(PWM_DAC, 0);
  motor.BindTimer1(&Timer1);
  // myPID.SetOutputLimits(MIN_PWM_VALUE, MAX_PWM_VALUE);
  startFlag = true;
  ScreenSaverOn = false;
  ShowScreenSaver = false;
}
/******************************************************************
 *
 *    LOOP:
 *      - PROGRAM FLOW.
 * 
 *****************************************************************/

void loop()
{

  LedAccept();
  Input = RealRpm * 100;
  controlBtn();

  switch (AppMode)
  {
  case APP_MOTOR_VIEW:
    appMotor();
    break;

  case APP_MENU_VIEW:

    APP_MENU_MODE();
    break;

  case APP_IDLE_VIEW:
    appIdle();

  default:
    break;
  }
}