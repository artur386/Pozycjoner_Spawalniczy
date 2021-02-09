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
#include <Wire.h>
#include "Parametr.h"
/***********************************************************
 *            PROTOTYPES                                   *
 * ********************************************************/
void WriteEEpromData();
void ReadEEpromData();
void controlBtn();
bool CheckAnyBtn();
void ClearButton(uint8_t btn);
bool ChceckBtn(uint8_t btn);
void SetButton(uint8_t btn);
void EncWrite(int8_t addVal);
void read_rotary();
void APP_MENU_MODE();
void getpidparam();
void appManage();
void APP_MOTOR_MODE();
void tach_interrupt();
void appIdle();
void FastParamChange(int mode);
void WriteParamToEEprom();
uint16_t CalcPWM(float rpm);
void CheckFastParameterChange();
/*************************************/

byte BT_ST = 0x00000000;

uint8_t AppMode;
uint8_t LastAppMode;

#define GET_INT_VARIABLE(X) values[x]
#define GET_DIV_VARIABLE(x) pgm_read_word(&(val_table[x][2]))
#define GET_FLOAT_VARIABLE(x) float(GET_INT_VARIABLE(x) / GET_DIV_VARIABLE(x))
volatile int8_t master_count;
unsigned long ScreenSaverTime, ScreenRefreshTime;
bool ScreenSaverOn, ShowScreenSaver;
unsigned long CONTROL_ROTARY_SW_press_time = millis();
unsigned long CONTROL_START_STOP_press_time = millis();
bool CONTROL_START_STOP_LONG_PRESS_DETECT = false;
bool CONTROL_ROTARY_SW_LONG_PRESS_DETECT = false;
bool CONTROL_ROTARY_SW_prev = HIGH;
bool CONTROL_START_STOP_prev = LOW;
bool WriteToEEprom_flag = false;
unsigned long LastEEpromWriteTime = 0;
// status pracy silnika
uint8_t MOTOR_STATE, LAST_MOTOR_STATE;
//kierunek obrotow silnika
bool MOTOR_CCW_DIR = false;
bool focus = false;
int pwm;
float ActualRPM;

// pid parameters:-
double Setpoint, Input, Output;

#define SAMPLE_TIME 100
bool PidOnOff;
uint16_t open_loop_pwm;

#define LED_ACCEPT_BLINK_TIME 100
uint8_t LED_ACCEPT_BLINK_CNT;
unsigned long LedAcceptBlinkLastTime;
bool LedAcceptIsOn, LedAcceptFlag, LedAcceptMode;
uint8_t LedAcceptCnt;

// Variables will change:
uint8_t ledState = LOW;
unsigned long previousMillis = 0;
const long interval = 1000;
bool blinkStart = false;
uint8_t blinkCount = 0;
uint8_t cursorFocus = 0;

bool startFlag;

int values[11];
uint8_t enu[3];

//RPM
volatile uint8_t ENC_STATE = 0;
uint32_t isrTime = 500;
double REAL_RPM, REAL_RPS;
bool READ_RPM = false;

// tachometer
long lastUpdate = 0;                  // for timing display updates
volatile long accumulator = 0;        // sum of last 8 revolution times
volatile unsigned long startTime = 0; // start of revolution in microseconds
volatile unsigned long revCount = 0;  // number of revolutions since last display update
uint8_t averageCnt = 0, averageCntMax = 5;
unsigned long revCountCopy = 0;                    // number of revolutions since last display update
unsigned long revCountSmooth[5] = {0, 0, 0, 0, 0}; // number of revolutions since last display update

// MAX VARIABLE
uint16_t LAST_DIA;
uint16_t CURR_DIA;
uint16_t LAST_VC;
uint16_t CURR_VC;
uint16_t LAST_RPM;
uint16_t CURR_RPM;

uint16_t setPWM, outPWM;
LiquidCrystal_I2C lcd(0x27, 20, 4);

#define testDT ((PIND >> ROT_ENC_DT) & 1)
#define BTN_START_STOP_TEST !((PIND >> BTN_START_STOP_PANEL) & 1)
#define BTN_SW_ROTATY ((PIND >> ROT_ENC_SW) & 1)

/* PARAMETRY MENU */
double paramRPM, paramLastRPM, paramRPS, paramMMSEC;
uint32_t paramDIA;
uint8_t paramGEAR;
uint32_t paramTimePause, paramTimeStart, paramTimeStop;
double paramKp, paramKi, paramKd;
uint8_t paramSpeedMethod;
bool paramPidOnOff;
uint16_t paramScreenSaver;

// define object :-
PID myPID(&Input, &Output, &Setpoint, paramKp, paramKi, paramKd, DIRECT);
DisplayManager Display(&lcd, &MOTOR_STATE);
Motor motor(RL_MOTOR_CW, RL_MOTOR_CCW, RL_MOTOR_GEAR, RL_MOTOR_START_STOP, PWM_DAC);
myMenu menu(&lcd, &values[0], &enu[0], &BT_ST, &AppMode);
RTC_DS3231 rtc;
BigFont fontPrinter;

/***************************************************
 * functions
 * ************************************************/
void GetParam()
{
}

void focusInc()
{
  focus = !focus;
  Display.updateScreen();
}

void tach_interrupt()
{
  // calculate the microseconds since the last interrupt
  long usNow = micros();
  long elapsed = usNow - startTime;
  startTime = usNow; // reset the clock

  // Accumulate the last 8 interrupt intervals
  accumulator -= (accumulator >> 3);
  accumulator += elapsed;
  revCount++;
}
void tach_count()
{
  revCount++;
}

uint16_t CalcPWM(float rpm)
{
  if (paramGEAR == 1)
  {
    return map(rpm * 1000, MinRPM_1stGear * 1000, MaxRPM_1stGear * 1000, MIN_PWM_VALUE, MAX_PWM_VALUE);
  }
  else if (paramGEAR == 2)
  {
    return map(rpm * 1000, MinRPM_2stGear * 1000, MaxRPM_2stGear * 1000, MIN_PWM_VALUE, MAX_PWM_VALUE);
  }
}

void LedAccept()
{
  if (LedAcceptFlag)
  {
    LedAcceptFlag = false;
    LedAcceptIsOn = true;
    LedAcceptBlinkLastTime = millis();
    LedAcceptCnt = 0;
    RED_LED_ON();
    GREEN_LED_OFF();
    if (LedAcceptMode == 0)
    {
      LED_ACCEPT_BLINK_CNT = 3;
    }
    else if (LedAcceptMode == 1)
    {
      LED_ACCEPT_BLINK_CNT = 6;
    }
  }
  if (LedAcceptIsOn)
  {
    if ((millis() - LedAcceptBlinkLastTime) >= LED_ACCEPT_BLINK_TIME)
    {
      /* code */
      RED_LED_OFF();
      digitalWrite(GLED, (LedAcceptCnt % 2));
      LedAcceptBlinkLastTime = millis();
      LedAcceptCnt++;
      if (LedAcceptCnt == LED_ACCEPT_BLINK_CNT)
      {
        RED_LED_ON();
        GREEN_LED_OFF();
        LedAcceptIsOn = false;
      }
    }
  }
}
void rpmCalc()
{
  switch (ENC_STATE)
  {
  case 0:
    detachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER));
    REAL_RPM = 0;
    break;

  case 1:
    lastUpdate = millis();
    ENC_STATE = 2;
    revCount = 0;
    if (TACHO_STYLE == 1)
    {
      attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER), tach_interrupt, CHANGE); //interrupt pin
    }
    if (TACHO_STYLE == 2)
    {
      averageCnt = 0;
      revCountSmooth[0] = 0;
      revCountSmooth[1] = 0;
      revCountSmooth[2] = 0;
      revCountSmooth[3] = 0;
      revCountSmooth[4] = 0;
      startTime = millis();
      attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER), tach_count, CHANGE); //interrupt pin
    }

    break;
  case 2:
    if (millis() - lastUpdate >= isrTime)
    {
      if (TACHO_STYLE == 1)
      {

        if (revCount > 0)
        {
          REAL_RPM = 40000.0 / (accumulator >> 3);
        }
        lastUpdate = millis();
        revCount = 0;
      }
      if (TACHO_STYLE == 2)
      {
        unsigned long timPeroid;
        float revCountCopyAvg;
        detachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER));
        timPeroid = micros() - startTime;
        revCountCopy = revCount;
        revCount = 0;
        startTime = micros();
        attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER), tach_count, CHANGE); //interrupt pin
        revCountSmooth[4] = revCountSmooth[3];
        revCountSmooth[3] = revCountSmooth[2];
        revCountSmooth[2] = revCountSmooth[1];
        revCountSmooth[1] = revCountSmooth[0];
        revCountSmooth[0] = revCountCopy;
        if (averageCnt < averageCntMax)
        {
          averageCnt++;
        }
        revCountCopyAvg = (revCountSmooth[0] + revCountSmooth[1] + revCountSmooth[2] + revCountSmooth[3] + revCountSmooth[4]) / float(averageCnt);
        REAL_RPM = ((revCountCopyAvg / 1500.0f) / timPeroid) * 60000000.0f;
        lastUpdate = millis();
      }
      Serial.println(REAL_RPM, 4);
    }
    break;

  default:
    break;
  }
}

void PID_ON_OFF()
{
  if (bool(enu[2]) != PidOnOff)
  {
    if (!bool(enu[2]))
    {
      DBG("AUTOMATIC PID");
      myPID.SetSampleTime(SAMPLE_TIME);
      myPID.SetMode(AUTOMATIC);
    }
    else
    {
      DBG("MANUAL PID");
      myPID.SetMode(MANUAL);
      // Output = open_loop_pwm;
    }
    PidOnOff = bool(enu[2]);
  }
}

void read_rotary()
{
  if (testDT)
    master_count++;
  else
    master_count--;
}

void EncWrite(int8_t addVal)
{
  detachInterrupt(digitalPinToInterrupt(ROT_ENC_CLK));
  master_count += addVal;
  attachInterrupt(digitalPinToInterrupt(ROT_ENC_CLK), read_rotary, CHANGE);
};

void SetButton(uint8_t btn)
{
  bitWrite(BT_ST, btn, HIGH);
}
bool ChceckBtn(uint8_t btn)
{
  if (bitRead(BT_ST, btn))
  {
    bitWrite(BT_ST, btn, LOW);
    return true;
  }
  else
  {
    return false;
  }
}
/*****************************************************************
 *  clear the specyfic button state after action runinng
 * ***************************************************************/

bool CheckAnyBtn()
{
  if (BT_ST > 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void controlBtn()
{
  if (master_count <= -2)
  {
    // check if the button is pressed and the encoder is rotated
    // the button is low active
    if (BTN_SW_ROTATY == LOW)
    {
      // button is pressed
      SetButton(BTN_RIGHT);
      // reset button press time for next detection
      CONTROL_ROTARY_SW_prev = HIGH;
    }
    else
    {
      SetButton(BTN_UP);
    }
    // init encoder for the next step
    EncWrite(2);
    // ENCODER.write(g_LCDML_CONTROL_Encoder_position + 4);
  }
  // check if encoder is rotated on direction B
  else if (master_count >= 2)
  {
    // check if the button is pressed and the encoder is rotated
    // the button is low active
    if (BTN_SW_ROTATY == LOW)
    {
      // button is pressed
      SetButton(BTN_LEFT);
      // reset button press time for next detection
      CONTROL_ROTARY_SW_prev = HIGH;
    }
    else
    {
      SetButton(BTN_DOWN);
    }
    // init encoder for the next step
    EncWrite(-2);
  }
  else if (BTN_SW_ROTATY == LOW && CONTROL_ROTARY_SW_prev == HIGH)
  {
    // check if the button was pressed for a shortly time or a long time
    //CHANGE edge, button pressed, no action
    CONTROL_ROTARY_SW_prev = LOW;
    CONTROL_ROTARY_SW_press_time = millis();
  }
  else if (BTN_SW_ROTATY == LOW && CONTROL_ROTARY_SW_prev == LOW)
  {
    if ((millis() - CONTROL_ROTARY_SW_press_time) >= CONTROL_button_long_press)
    {
      // long press detected
      if (!CONTROL_ROTARY_SW_LONG_PRESS_DETECT)
      {
        SetButton(BTN_SW_LONG);
      }
    }
  }
  // CHANGE edge, button not pressed, check how long was it pressed
  else if (BTN_SW_ROTATY == HIGH && CONTROL_ROTARY_SW_prev == LOW)
  {
    CONTROL_ROTARY_SW_prev = HIGH;
    CONTROL_ROTARY_SW_LONG_PRESS_DETECT = false;
    // check how long was the button pressed and detect a long press or a short press
    // check long press situation

    // check short press situation
    if ((millis() - CONTROL_ROTARY_SW_press_time) >= CONTROL_button_short_press && (millis() - CONTROL_ROTARY_SW_press_time) <= CONTROL_button_long_press)
    {
      // short press detected
      SetButton(BTN_SW_SHORT);
    }
  }
  else if (digitalRead(BTN_START_STOP_PANEL) == HIGH && CONTROL_START_STOP_prev == LOW)
  {
    CONTROL_START_STOP_prev = HIGH;
    CONTROL_START_STOP_press_time = millis();
  }
  else if (digitalRead(BTN_START_STOP_PANEL) == HIGH && CONTROL_START_STOP_prev == HIGH)
  {
    if ((millis() - CONTROL_START_STOP_press_time) >= CONTROL_button_long_press)
    {
      if (!CONTROL_START_STOP_LONG_PRESS_DETECT)
      {
        CONTROL_START_STOP_LONG_PRESS_DETECT = true;
        SetButton(BTN_START_STOP_LONG);
      }

      // long press detected
    }
  }
  // CHANGE edge, button not pressed, check how long was it pressed
  else if (digitalRead(BTN_START_STOP_PANEL) == LOW && CONTROL_START_STOP_prev == HIGH)
  {
    CONTROL_START_STOP_prev = LOW;
    CONTROL_START_STOP_LONG_PRESS_DETECT = false;
    // check how long was the button pressed and detect a long press or a short press
    // check long press situation
    // if ((millis() - CONTROL_START_STOP_press_time) >= CONTROL_button_long_press)
    // {
    //   // long press detected
    //   SetButton(BTN_START_STOP_LONG);
    // }
    // check short press situation
    if ((millis() - CONTROL_START_STOP_press_time) >= CONTROL_button_short_press && (millis() - CONTROL_START_STOP_press_time) <= CONTROL_button_long_press)
    {
      // short press detected
      SetButton(BTN_START_STOP_SHORT);
    }
  }
}

void WriteEEpromData()
{
  EEPROM.put(EEPROM_START_ADRESS, values);
  EEPROM.put(sizeof(values), enu);

  /*
  for (size_t i = 0; i < 11; i++)
  {
    Serial.print("Write EEPROM val - ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(values[i]);
  }
  for (size_t i = 0; i < 3; i++)
  {
    Serial.print("Write EEPROM enum - ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(enu[i]);
  }
  */
}
void ReadEEpromData()
{
  // Odczyt ustawień z pamięci EEPROM
  EEPROM.get(EEPROM_START_ADRESS, values);
  EEPROM.get(sizeof(values), enu);

  paramSpeedMethod = enu[1];
  paramDIA = values[3];
  paramTimePause = values[5];
  paramTimeStart = values[6];
  paramTimeStop = values[7];
  paramRPM = GET_FLOAT_VAL(0);
  paramRPS = GET_FLOAT_VAL(1);
  paramMMSEC = GET_FLOAT_VAL(2);
  paramGEAR = enu[0];
  paramPidOnOff = (bool)enu[2];
  paramKp = GET_FLOAT_VAL(8);
  paramKi = GET_FLOAT_VAL(9);
  paramKd = GET_FLOAT_VAL(10);
  paramScreenSaver = values[4] * 1000;
}
/********************************************************************************************
 * 
 * 
 *    SETUP:
 *      - INICJALIZACJA PARAMETROW.
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * ***************************************************************************************/
void setup()
{
  // REV = 0; //  START ALL THE VARIABLES FROM 0

  Serial.begin(115200);
  Serial.println("start");
  ReadEEpromData();
  lcd.init();
  lcd.clear(); // czyszczenie wyświetlacza lcd
  lcd.backlight();
  // char testa[] = "test";
  // char *b = &testa[0];
  fontPrinter.BindLcd(&lcd);
  fontPrinter.WriteBigString(" PROEKO ", 0, 1);

  motor.BindMotorState(&MOTOR_STATE);
  motor.SetTimes(&values[5], &values[6], &values[7]);
  motor.SetGears(&enu[0]);
  motor.BindCCWDir(&MOTOR_CCW_DIR);

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
  pwm = 50;
  PidOnOff = !bool(enu[2]);
  //Initialize serial and wait for port to open:

  pinMode(NCO_0, INPUT_PULLUP);
  pinMode(NCO_1, INPUT_PULLUP);
  pinMode(NCO_2, INPUT_PULLUP);
  pinMode(NCO_3, INPUT_PULLUP);
  pinMode(NCI_0, INPUT_PULLUP);
  pinMode(NCI_1, INPUT_PULLUP);
  pinMode(NCI_2, INPUT_PULLUP);
  pinMode(NCI_3, INPUT_PULLUP);
  pinMode(NCI_4, INPUT_PULLUP);

  PWR_LED_ON;
  RED_LED_ON();
  GREEN_LED_OFF();
  // attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER), isr, CHANGE); //interrupt pin
  attachInterrupt(digitalPinToInterrupt(ROT_ENC_CLK), read_rotary, CHANGE);
  CONTROL_ROTARY_SW_press_time = millis();
  CONTROL_ROTARY_SW_prev = HIGH;

  Display.SetREAL_RPM_P(&REAL_RPM);
  Display.bindFocus(&focus);
  Display.BindSpeed(&paramRPM, &paramRPS, &paramMMSEC);
  Display.BindDia(&paramDIA);
  Display.BindSpeedMethod(&paramSpeedMethod);
  // Display.cutText();
  Timer1.initialize(128); // 40 us = 25 kHz
  Timer1.pwm(PWM_DAC, 0);
  motor.BindTimer1(&Timer1);
  myPID.SetOutputLimits(MIN_PWM_VALUE, MAX_PWM_VALUE);
  startFlag = true;
  ScreenSaverOn = false;
  ShowScreenSaver = false;
}
/********************************************************************************************
 * 
 * 
 *    LOOP:
 *      - PROGRAM FLOW.
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * ***************************************************************************************/
void appMotor()
{
  if (LastAppMode != AppMode)
  {
    Display.drawMainScreen();
    Display.updateScreen();
    WriteEEpromData();
    LastAppMode = AppMode;
    myPID.SetTunings(paramKp, paramKi, paramKd);
    PID_ON_OFF();
    Output = 0;
    paramLastRPM = 0;
  }
  // DBG(ENC_STATE);

  rpmCalc();
  Setpoint = paramRPM * 100;
  if (ChceckBtn(BTN_START_STOP_SHORT))
  {
    if (MOTOR_STATE == MOTOR_STOP)
    {
      motor.motorStart();
      ENC_STATE = 1;
    }
    else if (MOTOR_STATE == MOTOR_OBR_PRAWO || MOTOR_STATE == MOTOR_OBR_LEWO)
    {
      MOTOR_STATE = MOTOR_SMOOTH_STOP;
    }
    else if (MOTOR_STATE == MOTOR_PAUSE)
    {
      MOTOR_STATE = MOTOR_PAUSE;
    }
    else if (MOTOR_STATE == MOTOR_SOFT_START)
    {
      MOTOR_STATE = MOTOR_SMOOTH_STOP;
    }
  }

  if (ChceckBtn(BTN_SW_LONG))
  {
    AppMode = APP_MENU_VIEW;
  }
  switch (MOTOR_STATE)
  {
  case MOTOR_STOP: //stop
    if (LAST_MOTOR_STATE != MOTOR_STATE)
    {
      LAST_MOTOR_STATE = MOTOR_STATE;
      AppMode = APP_IDLE_VIEW;
      Display.ForceRefresh();
    }
    break;

  case MOTOR_PAUSE: //pause
    if (LAST_MOTOR_STATE != MOTOR_STATE)
    {
      LAST_MOTOR_STATE = MOTOR_STATE;
      motor.DoPause();
      RED_LED_OFF();
      GREEN_LED_ON();
      Display.ForceRefresh();
    }
    break;

  case MOTOR_SOFT_START:
    if (LAST_MOTOR_STATE != MOTOR_STATE)
    {
      motor.relayStart();
      motor.DoSofrStart(Output);
      RED_LED_OFF();
      GREEN_LED_ON();
      LAST_MOTOR_STATE = MOTOR_STATE;
      Display.ForceRefresh();
    }
    break;

  case MOTOR_OBR_PRAWO:
    if (LAST_MOTOR_STATE != MOTOR_STATE)
    {
      LAST_MOTOR_STATE = MOTOR_STATE;
      motor.relayStart();
      Display.ForceRefresh();
    }
    if (ChceckBtn(BTN_START_STOP_SHORT))
    {
      MOTOR_STATE = MOTOR_SMOOTH_STOP;
    }
    myPID.Compute();
    motor.SET_PWM(Output);
    DBG(Output);
    RED_LED_OFF();
    GREEN_LED_ON();
    break;

  case MOTOR_OBR_LEWO:
    if (LAST_MOTOR_STATE != MOTOR_STATE)
    {
      LAST_MOTOR_STATE = MOTOR_STATE;
      motor.relayStart();

      Display.ForceRefresh();
      RED_LED_OFF();
      GREEN_LED_ON();
    }
    if (ChceckBtn(BTN_START_STOP_SHORT))
    {
      MOTOR_STATE = MOTOR_SMOOTH_STOP;
    }
    myPID.Compute();
    motor.SET_PWM(Output);
    DBG(Output);
    break;

  case MOTOR_SMOOTH_STOP:
    if (LAST_MOTOR_STATE != MOTOR_STATE)
    {
      LAST_MOTOR_STATE = MOTOR_STATE;
      motor.DoSmoothStop(Output);
      Display.ForceRefresh();
    }
    break;

  case MOTOR_GO_TO_STOP:
    if (LAST_MOTOR_STATE != MOTOR_STATE)
    {
      LAST_MOTOR_STATE = MOTOR_STATE;
      motor.motorStop();
      ENC_STATE = MOTOR_STOP;
      RED_LED_ON();
      GREEN_LED_OFF();
      Display.ForceRefresh();
    }

    break;

  default:
    break;
  }
  if (MOTOR_STATE == MOTOR_OBR_LEWO || MOTOR_STATE == MOTOR_OBR_PRAWO)
  {
    CheckFastParameterChange();
  }
  if (paramPidOnOff)
  {
    if (paramLastRPM != paramRPM)
    {
      motor.SET_PWM(CalcPWM(paramRPM));
    }
  }
  else
  {
    motor.SET_PWM(Output);
  }
  Display.updateScreen();
}

void APP_MENU_MODE()
{
  if (LastAppMode != AppMode)
  {
    WriteEEpromData();
    ReadEEpromData();

    LastAppMode = AppMode;
    menu.MenuStart();
    menu.GetMenu(0);
    menu.DrawMenu();
  }
  menu.UpdateMenu();
}

void appIdle()
{
  if (startFlag)
  {
    while (!CheckAnyBtn())
    {
      controlBtn();
      delay(10);
    }
    BT_ST = 0x00000000;
    startFlag = false;
    ScreenSaverOn = true;
    ScreenSaverTime = millis();
  }

  if (LastAppMode != AppMode)
  {
    Display.drawMainScreen();
    Display.ForceRefresh();
    Display.updateScreen();
    LastAppMode = AppMode;
  }
  if (CheckAnyBtn())
  {
    CheckFastParameterChange();

    if (ChceckBtn(BTN_START_STOP_SHORT))
    {
      AppMode = APP_MOTOR_VIEW;
      motor.motorStart();
      ENC_STATE = 1;
    }
    else if (ChceckBtn(BTN_SW_LONG))
    {
      LedAcceptFlag = true;
      LedAcceptMode = 1;
      AppMode = APP_MENU_VIEW;
    }
    else if (ChceckBtn(BTN_SW_SHORT))
    {
      LedAcceptFlag = true;
      focusInc();
    }

    else if (ChceckBtn(BTN_START_STOP_LONG))
    {
      LedAcceptFlag = true;
      MOTOR_CCW_DIR = !MOTOR_CCW_DIR;
    }
    Display.ForceRefresh();
    Display.updateScreen();
    ScreenSaverTime = millis();
    ShowScreenSaver = false;
  }
  if (ScreenSaverOn)
  {
    if (millis() - ScreenSaverTime >= paramScreenSaver && !ShowScreenSaver)
    {
      ShowScreenSaver = true;
      lcd.clear();
    }
  }
  if (ShowScreenSaver)
  {
    if (millis() - ScreenRefreshTime >= 1000)
    {
      DateTime now = rtc.now();
      char *timeStr = (char *)malloc(sizeof(char) * 8);
      sprintf(timeStr, "%d:%d:%d", now.hour(), now.minute(), now.second());
      fontPrinter.WriteBigString(timeStr, 0, 1);
      free(timeStr);
      ScreenRefreshTime = millis();
    }
  }
  // delay(500);
}

void CheckFastParameterChange()
{
  if (ChceckBtn(BTN_UP))
  {
    FastParamChange(0);
  }
  else if (ChceckBtn(BTN_DOWN))
  {
    FastParamChange(1);
  }
  else if (ChceckBtn(BTN_LEFT))
  {
    FastParamChange(3);
  }
  else if (ChceckBtn(BTN_RIGHT))
  {
    FastParamChange(2);
  }
}

void FastParamChange(int mode)
{
  float val;
  switch (mode)
  {
  case 0:
    val = -1;
    break;

  case 1:
    val = 1;
    break;

  case 2:
    val = -10;
    break;

  case 3:
    val = 10;
    break;

  default:
    break;
  }
  // DBG(val);
  // DBG(paramRPM);
  // DBG(paramSpeedMethod);
  if (paramSpeedMethod == 0)
  {
    val = val / VAL_TBL__DIV(0);
    paramRPM += val;
    paramRPS = paramRPM / 60.0f;
    paramMMSEC = RPM_TO_MMSEC(paramRPM, paramDIA);
    DBG(paramRPM);
    DBG(paramRPS);
    DBG(paramMMSEC);
  }
  else if (paramSpeedMethod == 1)
  {
    val = val / VAL_TBL__DIV(1);
    paramRPS += val;
    paramRPM = paramRPS * 60.0f;
    paramMMSEC = RPM_TO_MMSEC(paramRPM, paramDIA);
    // DBG(paramRPS);
  }
  else if (paramSpeedMethod == 2)
  {
    if (!focus)
    {
      paramDIA += val;
      paramRPM = MMSEC_TO_RPM(paramMMSEC, paramDIA);
      paramMMSEC = RPM_TO_MMSEC(paramRPM, paramDIA);
      paramRPS = paramRPM / 60.0f;
    }
    else
    {
      val = val / VAL_TBL__DIV(2);
      paramMMSEC += val;
      paramRPM = MMSEC_TO_RPM(paramMMSEC, paramDIA);
      paramRPS = paramRPM / 60.0f;
    }
  }
  values[0] = int(paramRPM * VAL_TBL__DIV(0));
  values[1] = int(paramRPS * VAL_TBL__DIV(1));
  values[2] = int(paramMMSEC * VAL_TBL__DIV(2));
  values[3] = int(paramDIA * VAL_TBL__DIV(3));
  Display.updateScreen();
  LastEEpromWriteTime = millis();
  WriteToEEprom_flag = true;
  WriteParamToEEprom();
}
void WriteParamToEEprom()
{
  if (WriteToEEprom_flag == true)
  {
    WriteToEEprom_flag = false;
    LastEEpromWriteTime = millis();
    WriteEEpromData();
  }
}
void loop()
{

  LedAccept();
  Input = REAL_RPM * 100;
  WriteParamToEEprom();
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