#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Variable.h"
#include "Display_Manager.h"
#include "Buttons.h"
#include "Motor.h"
#include "EnumTypes.h"
#include <EEPROM.h>

uint8_t AppMode;
uint8_t LastAppMode;

uint8_t MOTOR_STATE;
uint8_t LAST_MOTOR_STATE;
bool MOTOR_CCW_DIR = false;

byte BTN_ROT_SW = NO_PRESS;
byte BTN_START_STOP = NO_PRESS;
unsigned long g_LCDML_CONTROL_button_press_time;
bool g_LCDML_CONTROL_button_prev;
// lcd
#define LCD_ADRESS 0x27
#define LCD_ROWS 4
#define LCD_COLS 20

// relay motor pins:
#define RL_MOTOR_CW 11
#define RL_MOTOR_CCW 8
#define RL_MOTOR_GEAR 10
#define RL_MOTOR_START_STOP A0

// pwm pin
#define PWM_DAC 9

// rotary encoder
#define ROT_ENC_CLK 3
#define ROT_ENC_DT 4
#define ROT_ENC_SW 6

// PRZYCISK START/STOP
#define BTN_START_STOP 7

// MOTOR INC ENCODER
#define MOTOR_ENCODER 2
#define LED 13

// Variables will change:
uint8_t ledState = LOW;
unsigned long previousMillis = 0;
const long interval = 1000;
bool blinkStart = false;
uint8_t blinkCount = 0;

float value = 0;
float rev = 0;
int rpm;
int oldtime = 0;
int time;
void isr()
{
  rev++;
}

uint8_t id[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
char *nazwa[10] = {"Srednica", "Predkosc", "Przerwa", "SoftStart", "SmoothStop", "PID", "Kp", "Ki", "Kd", "Back"};
uint16_t minVal[10] = {
    10,
    1,
    0,
    0,
    0,
    0,
    1,
    1,
    1,
    0};
uint16_t maxVal[10] = {
    300,
    25,
    5000,
    5000,
    5000,
    1,
    20,
    20,
    20,
    0};
uint16_t curVal[10] = {
    10,
    2,
    0,
    0,
    0,
    0,
    1,
    1,
    1,
    0};
uint16_t incVal[10] = {
    1,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    0};
uint16_t fracVal[10] = {
    1,
    10,
    100,
    100,
    100,
    1,
    1,
    1,
    1,
    0};
// void (*handle)()[10] = {
//     printMM,
//     printVC,
//     printSEC,
//     printSEC,
//     printSEC,
//     ON_OFF,
//     NULL,
//     NULL,
//     NULL,
//     goBACK}

// MAX VARIABLE
uint16_t LAST_DIA;
uint16_t LAST_VC;
uint16_t LAST_RPM;
uint16_t REAL_RPM;

#define IsDiaChanged (LAST_DIA != curVal[0])
#define IsVcChanged (LAST_VC != curVal[1])
#define IsReadRpmChanged (LAST_RPM != REAL_RPM)

LiquidCrystal_I2C lcd(LCD_ADRESS, LCD_COLS, LCD_ROWS);

Buttons BT_START_STOP(BTN_START_STOP);
Buttons BT_ROT_ENC_SW(ROT_ENC_SW);
// #define _DEBUG

void printMM();
void printVC();
void printSEC();
void ON_OFF();
void goBACK();
void encoderISR();
void counter();
void buttonLongAction();
void buttonShortAction();
void ButtonSW();
void DecRotEnc(int decVal);
void IncRotEnc(int incVal);
void formatUlamki();
void formatDir();
void formatInt();
void triger();
void formatPWM();
// void drawMainScreen();
void drawMenu();
void handleCW();
void handleCCW();
void handleOK();
int getEncPos();
int readRotEnc();
uint8_t readButtons();
void blinkLed(uint8_t times);
void SaveToEEPROM();

// DEKLARACJA ZMIENNYCH przerwań
volatile byte readPulses; // liczba pulsów enkodera prędkości
byte copyPulses;
volatile char master_count;
unsigned long timeold; // ostatni czas odczytu prędkości
unsigned long timenow; //  czas odczytu prędkości

const byte PPR = 50;                     // liczba rowków na tarczy enkodera prędkości
const unsigned int rpmUpdateTime = 1000; // czas odświeżania obrotów

#define testDT ((PIND >> ROT_ENC_DT) & 1)
#define testSTART_STOP ((PIND >> BTN_START_STOP) & 1)
#define testSW ((PIND >> ROT_ENC_SW) & 1)

//Parametry:

const byte MAX_PARAM = 5;
parameterStruct Parameters[MAX_PARAM];

void counter()
{
  //Update count
  readPulses++;
}
void read_rotary()
{
  if (testDT)
    master_count++;
  else
    master_count--;
};
char EncRead()
{
  return master_count;
};
void EncWrite(char addVal)
{
  detachInterrupt(digitalPinToInterrupt(ROT_ENC_CLK));
  master_count = addVal;
  attachInterrupt(digitalPinToInterrupt(ROT_ENC_CLK), read_rotary, CHANGE);
};
void blinkLed(uint8_t times)
{
  // if (!blinkStart)
  // {
  //   blinkStart = true;
  //   blinkCount = times;
  // }
  // if (blinkStart)
  // {
  //   unsigned long currentMillis = millis();
  //   if (currentMillis - previousMillis >= interval)
  //   {
  //     // save the last time you blinked the LED
  //     previousMillis = currentMillis;

  //     // if the LED is off turn it on and vice-versa:
  // if (ledState == LOW)
  // {
  //   ledState = HIGH;
  // }
  // else
  // {
  //   ledState = LOW;
  // }

  // set the LED with the ledState of the variable:
  digitalWrite(LED, HIGH);
  delay(800);
  digitalWrite(LED, LOW);
  delay(800);
  digitalWrite(LED, HIGH);
  delay(800);
  digitalWrite(LED, LOW);
  delay(800);
  // blinkCount--;
  // }
  // }
  // if (!blinkCount)
  // {
  //   blinkStart = false;
  // }
}
void formatUlamki()
{
}
void formatDir()
{
}
void formatInt()
{
}
void triger()
{
}
void formatPWM()
{
}

// void SaveToEEPROM()
// {
//   uint16_t AdressStart = 0;
//   /*
//   char *name[];
//   uint8_t Id;
//   uint8_t minVal;
//   uint16_t maxVal;
//   uint16_t curVal;
//   uint8_t incVal;
//   uint8_t frac;
//   void (*handler)();
// */

//   struct Config1
//   { //structure that we want to store
//     char name[20] = "Srednica";
//     uint8_t id = 0;
//     uint16_t minVal = 10;
//     uint16_t maxVal = 300;
//     uint16_t curVal = 100;
//     uint16_t incVal = 1;
//     uint16_t frac = 1;
//     void (*handler)() = printMM;
//   } Config1;
//   struct Config2
//   { //structure that we want to store
//     char name[20] = "Predkoc";
//     uint8_t id = 1;
//     uint16_t minVal = 10;
//     uint16_t maxVal = 300;
//     uint16_t curVal = 20;
//     uint16_t incVal = 1;
//     uint16_t frac = 10;
//     void (*handler)() = printVC;
//   } Config2;

//   struct Config3
//   { //structure that we want to store
//     char name[20] = "Pauza";
//     uint8_t id = 0;
//     uint16_t minVal = 0;
//     uint16_t maxVal = 5000;
//     uint16_t curVal = 0;
//     uint16_t incVal = 1;
//     uint16_t frac = 1;
//     void (*handler)() = printMM;
//   } Config3;
//   struct Config4
//   { //structure that we want to store
//     char name[20] = "SoftStart";
//     uint8_t id = 0;
//     uint16_t minVal = 0;
//     uint16_t maxVal = 5000;
//     uint16_t curVal = 0;
//     uint16_t incVal = 1;
//     uint16_t frac = 1;
//     void (*handler)() = printMM;
//   } Config4;
//   struct Config5
//   { //structure that we want to store
//     char name[20] = "SmoothStop";
//     uint8_t id = 0;
//     uint16_t minVal = 0;
//     uint16_t maxVal = 5000;
//     uint16_t curVal = 0;
//     uint16_t incVal = 1;
//     uint16_t frac = 1;
//     void (*handler)() = printMM;
//   } Config5;
//   struct Config6
//   { //structure that we want to store
//     char name[20] = "PID";
//     uint8_t id = 0;
//     uint16_t minVal = 0;
//     uint16_t maxVal = 1;
//     uint16_t curVal = 0;
//     uint16_t incVal = 1;
//     uint16_t frac = 1;
//     void (*handler)() = ON_OFF;
//   } Config6;
//   struct Config7
//   { //structure that we want to store
//     char name[20] = "Kp";
//     uint8_t id = 0;
//     uint16_t minVal = 0;
//     uint16_t maxVal = 1000;
//     uint16_t curVal = 1;
//     uint16_t incVal = 100;
//     uint16_t frac = 100;
//     void (*handler)() = NULL;
//   } Config7;
//   struct Config8
//   { //structure that we want to store
//     char name[20] = "Ki";
//     uint8_t id = 0;
//     uint16_t minVal = 0;
//     uint16_t maxVal = 1000;
//     uint16_t curVal = 1;
//     uint16_t incVal = 100;
//     uint16_t frac = 100;
//     void (*handler)() = NULL;
//   } Config8;
//   struct Config9
//   { //structure that we want to store
//     char name[20] = "Kd";
//     uint8_t id = 0;
//     uint16_t minVal = 0;
//     uint16_t maxVal = 1000;
//     uint16_t curVal = 1;
//     uint16_t incVal = 100;
//     uint16_t frac = 100;
//     void (*handler)() = NULL;
//   } Config9;

//   EEPROM.put(AdressStart, Config1);
//   AdressStart += sizeof(Config1);
//   EEPROM.put(AdressStart, Config2);
//   AdressStart += sizeof(Config2);
//   EEPROM.put(AdressStart, Config3);
//   AdressStart += sizeof(Config1);
//   EEPROM.put(AdressStart, Config4);
//   AdressStart += sizeof(Config1);
//   EEPROM.put(AdressStart, Config5);
//   AdressStart += sizeof(Config1);
//   EEPROM.put(AdressStart, Config6);
//   AdressStart += sizeof(Config1);
//   EEPROM.put(AdressStart, Config7);
//   AdressStart += sizeof(Config1);
//   EEPROM.put(AdressStart, Config8);
//   AdressStart += sizeof(Config1);
//   EEPROM.put(AdressStart, Config9);
//   AdressStart += sizeof(Config1);
// }

uint8_t
readButtons()
{
  uint8_t btn = NO_PRESS;
#define g_LCDML_CONTROL_button_long_press 800  // ms
#define g_LCDML_CONTROL_button_short_press 120 // ms
  // declare variable for this function
  int32_t g_LCDML_CONTROL_Encoder_position = EncRead();
  // bool g_LCDML_button = testSW;
  btn = BT_START_STOP.getButton();
  if ((bool)btn)
  {
    return btn;
  }
  // check if encoder is rotated on direction A
  if (g_LCDML_CONTROL_Encoder_position <= -1)
  {
    // check if the button is pressed and the encoder is rotated
    // the button is low active
    if (testSW == LOW)
    {
      // button is pressed
      btn = LEFT;
      // reset button press time for next detection
      g_LCDML_CONTROL_button_prev = HIGH;
    }
    else
    {
      btn = DOWN;
    }
    // init encoder for the next step
    EncWrite(master_count + 2);
    // ENCODER.write(g_LCDML_CONTROL_Encoder_position + 4);
  }
  // check if encoder is rotated on direction B
  else if (g_LCDML_CONTROL_Encoder_position >= 1)
  {
    // check if the button is pressed and the encoder is rotated
    // the button is low active
    if (testSW == LOW)
    {
      // button is pressed
      btn = RIGHT;
      // reset button press time for next detection
      g_LCDML_CONTROL_button_prev = HIGH;
    }
    else
    {
      btn = UP;
    }
    // init encoder for the next step
    EncWrite(master_count - 2);
  }
  else
  {
    // check if the button was pressed for a shortly time or a long time
    //falling edge, button pressed, no action
    if (testSW == LOW && g_LCDML_CONTROL_button_prev == HIGH)
    {
      g_LCDML_CONTROL_button_prev = LOW;
      g_LCDML_CONTROL_button_press_time = millis();
    }
    // rising edge, button not pressed, check how long was it pressed
    else if (testSW == HIGH && g_LCDML_CONTROL_button_prev == LOW)
    {
      g_LCDML_CONTROL_button_prev = HIGH;
      // check how long was the button pressed and detect a long press or a short press
      // check long press situation
      if ((millis() - g_LCDML_CONTROL_button_press_time) >= g_LCDML_CONTROL_button_long_press)
      {
        // long press detected
        btn = ENC_LONG_PRESS;
      }
      // check short press situation
      else if ((millis() - g_LCDML_CONTROL_button_press_time) >= g_LCDML_CONTROL_button_short_press)
      {
        // short press detected
        btn = ENC_SHORT_PRESS;
      }
    }
    else
    {
      // do nothing
    }
  }
  return btn;
}

Display_ManagerClass Display;
Motor motor(RL_MOTOR_CW, RL_MOTOR_CCW, RL_MOTOR_GEAR, RL_MOTOR_START_STOP, PWM_DAC);
void setup()
{
  Display.SetLcd(&lcd);
  Display.SetParam(&Parameters[0], &MAX_PARAM);
  motor.BindMotorState(&MOTOR_STATE);
  Display.BindMotorState(&MOTOR_STATE);

  // Ustawienie pinów IO
  pinMode(MOTOR_ENCODER, INPUT);
  pinMode(ROT_ENC_CLK, INPUT);
  pinMode(ROT_ENC_DT, INPUT);
  AppMode = APP_NORMAL_MODE;
  LastAppMode = 100;
  MOTOR_STATE = MT_STOP;
  LAST_MOTOR_STATE = 100;
  pinMode(LED, OUTPUT);

  Serial.begin(115200);
  //Initialize serial and wait for port to open:
  // Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB
  }
  // SaveToEEPROM();
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER), isr, RISING); //interrupt pin
  attachInterrupt(digitalPinToInterrupt(ROT_ENC_CLK), read_rotary, CHANGE);
  g_LCDML_CONTROL_button_press_time = millis();
  g_LCDML_CONTROL_button_prev = HIGH;
  // Display_Manager.setStruct(&menu);
  // Display_Manager.drawMainScreen();
  // MENU_SIZE = sizeof(menu) / sizeof(menu[0]);

  // void setCurrentMenuPos(byte & CURRENT_MENU_POS);
  // void setLastMenuPos(byte & CURRENT_MENU_POS);
  // void setCurrentMenuLevel(byte & CURRENT_MENU_LEVEL);
  // void setLastMenuLevel(byte & LAST_MENU_LEVEL);
  // void setMenuIsOn(bool &MENU_IS_ON);
  // void setHomeScreenCursorPos(byte & HOME_SCREEN_CURSOR_POSITION);
  // Display_Manager.drawMainScreen(1, 2, 3, 4, 5);
  // Display_Manager.bindVariables(&timeold);
}

void loop()
{
  MOTOR_STATE = motor.GetState();
  uint8_t btn = 0;
  switch (AppMode)
  {
  case APP_NORMAL_MODE:
    if (LastAppMode != AppMode)
    {
      Display.drawMainScreen();
      LastAppMode = AppMode;
    }
    btn = readButtons();
    if (MOTOR_STATE == 0)
    {
      if (btn == 1)
      {
        motor.SET_PWM(200);
        motor.motorStart(MOTOR_CCW_DIR);
      }
      else if (btn == 2)
      {
        MOTOR_CCW_DIR = !MOTOR_CCW_DIR;
        motor.motorStart(MOTOR_CCW_DIR);
      }
    }
    else if (MOTOR_STATE == 3 || MOTOR_STATE == 4)
    {
      if (btn == 1)
      {
        motor.motorStop();
      }
    }

    motor.motorManage();
    if (MOTOR_STATE != LAST_MOTOR_STATE)
    {
      Display.Update_MOT_Status(MOTOR_STATE);
      LAST_MOTOR_STATE = MOTOR_STATE;
    }
    if (IsDiaChanged)
    {
      LAST_DIA = curVal[0];
      Display.UpdateDiameterLcd(curVal[0]);
    }
    if (IsVcChanged)
    {
      LAST_VC = curVal[1];
      Display.UpdateSetLcd((float)curVal[1]);
    }
    if (IsReadRpmChanged)
    {
      LAST_RPM = REAL_RPM;
      Display.UpdateDiameterLcd(curVal[0]);
    }
#ifdef DEBUG
    Serial.print("MOTOR_STATE: ");
    Serial.println(MOTOR_STATE);
#endif // DEBUG
    break;
  case APP_MENU_MODE:
    /* code */
    break;

  case APP_PROCESS_MENU_CMD:
    /* code */
    break;

  default:
    break;
  }
  // Serial.println(readButtons());
  // Serial.print("master_count ");
  // Serial.println(testSW);
  // lcd.setCursor(0, 0);
  // lcd.print("test");
  // delay(200);
  // Display_Manager.cutText();
}
