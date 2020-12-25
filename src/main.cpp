#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Variable.h"
#include "Display_Manager.h"
#include "Buttons.h"

enum AppModeValues
{
  APP_NORMAL_MODE,
  APP_MENU_MODE,
  APP_PROCESS_MENU_CMD
};
enum ButtonPressMode
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

byte appMode = APP_NORMAL_MODE;
byte BTN_ROT_SW = NO_PRESS;
byte BTN_START_STOP = NO_PRESS;

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

// MAX VARIABLE
#define MAX_VARIABLES 6

LiquidCrystal_I2C lcd(LCD_ADRESS, LCD_ROWS, LCD_COLS);

Buttons BT_START_STOP(BTN_START_STOP);
Buttons BT_ROT_ENC_SW(ROT_ENC_SW);
// #define _DEBUG

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

// DEKLARACJA ZMIENNYCH przerwań
volatile byte readPulses; // liczba pulsów enkodera prędkości
byte copyPulses;
volatile uint8_t master_count;
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
uint8_t EncRead()
{
  return master_count;
};
void EncWrite(uint8_t addVal)
{
  noInterrupts();
  master_count += addVal;
  interrupts();
};
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
Display_ManagerClass Display(&lcd, &Parameters[0], &MAX_PARAM);

void setup()
{
  // Ustawienie pinów IO
  pinMode(MOTOR_ENCODER, INPUT);
  pinMode(ROT_ENC_CLK, INPUT);
  pinMode(ROT_ENC_DT, INPUT);

  // pinMode(BTN_START_STOP, INPUT);
  // pinMode(ROT_ENC_SW, INPUT);
  // pinMode(PWM_DAC, OUTPUT);
  // pinMode(RL_MOTOR_GEAR, OUTPUT);
  // pinMode(RL_MOTOR_CW, OUTPUT);
  // pinMode(RL_MOTOR_CCW, OUTPUT);
  // pinMode(RL_MOTOR_START_STOP, OUTPUT);
  pinMode(LED, OUTPUT);

  Serial.begin(115200);

  attachInterrupt(0, counter, FALLING);
  attachInterrupt(1, read_rotary, CHANGE);

  // Display_Manager.setStruct(&menu);
  // Display_Manager.drawMainScreen();
  // MENU_SIZE = sizeof(menu) / sizeof(menu[0]);

  void setCurrentMenuPos(byte & CURRENT_MENU_POS);
  void setLastMenuPos(byte & CURRENT_MENU_POS);
  void setCurrentMenuLevel(byte & CURRENT_MENU_LEVEL);
  void setLastMenuLevel(byte & LAST_MENU_LEVEL);
  void setMenuIsOn(bool &MENU_IS_ON);
  void setHomeScreenCursorPos(byte & HOME_SCREEN_CURSOR_POSITION);
  // Display_Manager.drawMainScreen(1, 2, 3, 4, 5);
  // Display_Manager.bindVariables(&timeold);
}

void loop()
{

  // Display_Manager.cutText();
}