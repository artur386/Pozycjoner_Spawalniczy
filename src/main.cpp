#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Display_Manager.h"

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
void drawMainScreen();
void drawMenu();
void handleCW();
void handleCCW();
void handleOK();
int getEncPos();
int readRotEnc();

LiquidCrystal_I2C lcd(0x27, 20, 4);

// DEKLARACJA PINÓW I/O
const byte encoderPin = 2; // pin kontrolera prędkości
const byte CLK = 3;        // pin zegarowy enkodera obrotowego
const byte DT = 4;         // pin kierunku enkodera obrotowego
const byte dacPin = 9;
const byte SW = 6;             // pin przycisku na enkoderze obrotowym
const byte btn_START_STOP = 7; // pin przycisku start/stop na panelu przednim
const byte obrLEWE = 8;        // pin przekaźnika dla załączenia obrotów lewych
const byte obrPRAWE = 11;      // pin przekaźnika dla załączenia obrotów prawych
const byte GEAR = 10;          // pin przekaźnika dla załączenia 2 biegu silnika
#define RELAY_START_STOP A0

// DEKLARACJA ZMIENNYCH przerwań
volatile byte readPulses; // liczba pulsów enkodera prędkości
byte copyPulses;
volatile char rotEncPos;      // pozycja enkodera obrotowego
volatile bool rotEncIsChange; // pozycja enkodera obrotowego
char copyRotEncPos;

unsigned long timeold; // ostatni czas odczytu prędkości
unsigned long timenow; //  czas odczytu prędkości

const byte PPR = 50;                     // liczba rowków na tarczy enkodera prędkości
const unsigned int rpmUpdateTime = 1000; // czas odświeżania obrotów

//przyciski
unsigned long buttonTimer = 0;
unsigned int longPressTime = 250;
boolean TurnDetected = false;

boolean buttonActive = false;
boolean longPressActive = false;

#define testDT ((PIND >> DT) & 1)
#define testSTART_STOP ((PIND >> btn_START_STOP) & 1)
#define testSW ((PIND >> SW) & 1)

//Parametry menu:
byte CURRENT_MENU_POS;
byte LAST_MENU_POS;
byte CURRENT_MENU_LEVEL;
byte LAST_MENU_LEVEL;

bool MENU_IS_ON;
byte HOME_SCREEN_CURSOR_POSITION;

unsigned long BUTTON_LAST_PRESS_TIME;
unsigned int BACK_TO_MAIN_SCREEN_TIME;

int MENU_SIZE;
bool MENU_IN_LOWER_LEVEL;
int TEMP_VAL;

struct MOTOR_STATE
{
  String label;
  int digitData;
  int dataPos;
  bool printData;
};

struct PARAMETER
{
  String label;
  int minVal;
  int maxVal;
  int currentVal;
  int incVal;
  void (*handler)();
};

struct MOTOR_SETTINGS
{
  String label;
  int minVal;
  int maxVal;
  int currentVal;
  int incVal;
  void (*handler)();
};

typedef enum
{
  CW,
  CCW,
  OK,
  NONE
} ENUM_BUTTON;

ENUM_BUTTON pressedButton;

int readRotEnc()
{
  Serial.print("read rot enc");
  Serial.println();

  detachInterrupt(1);
  copyRotEncPos = rotEncPos;
  rotEncPos = 0;
  attachInterrupt(1, encoderISR, FALLING);
  int returnData;
  Serial.print("copy rot enc");
  Serial.print((int)copyRotEncPos);
  Serial.println();
  if ((bool)copyRotEncPos)
  {
    if ((int)copyRotEncPos > 0)
    {
      returnData = 0;
      // IncRotEnc(copyRotEncPos);
    }
    else if ((int)copyRotEncPos < 0)
    {
      returnData = 1;
      // DecRotEnc((copyRotEncPos * (-1)));
    }
  }
  else
  {
    returnData = 3;
  }

  return returnData;
}

void IncRotEnc(int incVal)
{

  Serial.print("inc rot enc");
  Serial.println();

  if (CURRENT_MENU_LEVEL == 0)
  {
  }
  else if (CURRENT_MENU_LEVEL == 1)
  {
    for (int i = 0; i < incVal; i++)
    {
      handleCW();
    }
    copyRotEncPos = 0;
  }
}
void DecRotEnc(int decVal)
{
  Serial.print("dec rot enc");
  Serial.println();

  if (CURRENT_MENU_LEVEL == 0)
  {
  }
  else if (CURRENT_MENU_LEVEL == 1)
  {
    for (size_t i = 0; i < decVal; i++)
    {
      handleCCW();
    }
    copyRotEncPos = 0;
  }
}

void Display_Manage()
{
  Serial.print("display manage");
  Serial.println();

  if (LAST_MENU_LEVEL != CURRENT_MENU_LEVEL)
  {
    if (CURRENT_MENU_LEVEL == 0)
    {
      drawMainScreen();
    }
    LAST_MENU_LEVEL = CURRENT_MENU_LEVEL;
  }
  if (CURRENT_MENU_LEVEL == 1)
  {
    drawMenu();
  }
  Serial.print("LAST_MENU_POS");
  Serial.print(LAST_MENU_POS);
  Serial.print("currentScreen");
  Serial.print(CURRENT_MENU_LEVEL);
  Serial.println();
}
void drawMainScreen()
{
  lcd.clear();
  lcd.print("menu glowne");
}
void drawMenu()
{
  //  switch (pressedButton)
  //   {
  //   case 0:
  //     IncRotEnc(copyRotEncPos);
  //     break;
  //   case 1:
  //     DecRotEnc(copyRotEncPos);
  //     break;
  //   case 3:
  //     return;
  //     break;
  //   }

  lcd.clear();
  if (MENU_IN_LOWER_LEVEL)
  {
    lcd.print(SETTINGS[CURRENT_MENU_POS].label);
    lcd.setCursor(0, 1);
    lcd.print("> ");

    if (SETTINGS[CURRENT_MENU_POS].handler != NULL)
    {
      (*(SETTINGS[CURRENT_MENU_POS].handler))();
    }
    else
    {
      lcd.print(TEMP_VAL);
    }
  }
  else
  {
    lcd.print("> Ustawienia --");
    lcd.setCursor(0, 1);
    lcd.print("> ");

    lcd.print(SETTINGS[CURRENT_MENU_POS].label);
  }
}

void ButtonSW()
{
  Serial.print("button sw");
  Serial.println();

  if (digitalRead(SW) == HIGH)
  {
    Serial.print("button sw - high");
    Serial.println();
    if (buttonActive == false)
    {

      buttonActive = true;
      buttonTimer = millis();
    }

    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false))
    {

      longPressActive = true;
      Serial.print("go to button long action");
      Serial.println();

      buttonLongAction();
    }
  }
  else
  {
    if (buttonActive == true)
    {
      if (longPressActive == true)
      {
        longPressActive = false;
      }
      else
      {
        Serial.print("go to button short action");
        Serial.println();
        buttonShortAction();
      }
      buttonActive = false;
    }
  }
}

void buttonShortAction()
{
  Serial.print("btn short action");
  Serial.println();
  if (CURRENT_MENU_LEVEL == 0)
  {
    HOME_SCREEN_CURSOR_POSITION++;
    if (HOME_SCREEN_CURSOR_POSITION > 1)
    {
      HOME_SCREEN_CURSOR_POSITION = 0;
    }
  }
  else if (CURRENT_MENU_LEVEL == 1)
  {
    handleOK();
  }
}

void handleCW()
{
  Serial.print("CW");
  Serial.println();
  if (MENU_IN_LOWER_LEVEL)
  {
    TEMP_VAL++;
    delay(50);
    if (TEMP_VAL > SETTINGS[CURRENT_MENU_POS].maxVal)
      TEMP_VAL = SETTINGS[CURRENT_MENU_POS].maxVal;
  }
  else
  {
    CURRENT_MENU_POS = (CURRENT_MENU_POS + 1) % MENU_SIZE;
  }
}

void handleCCW()
{
  Serial.print("handle ccw");
  Serial.println();
  if (MENU_IN_LOWER_LEVEL)
  {
    TEMP_VAL--;
    delay(50);
    if (TEMP_VAL < SETTINGS[CURRENT_MENU_POS].minVal)
      TEMP_VAL = SETTINGS[CURRENT_MENU_POS].minVal;
  }
  else
  {
    CURRENT_MENU_POS--;
    if (CURRENT_MENU_POS < 0)
      CURRENT_MENU_POS = MENU_SIZE - 1;
  }
}

void handleBack()
{
  //    if (MENU_IN_LOWER_LEVEL)
  //    {
  //        MENU_IN_LOWER_LEVEL = false;
  //    }
}

void handleOK()
{
  Serial.print("handle ok");
  Serial.println();
  if (SETTINGS[CURRENT_MENU_POS].handler != NULL && SETTINGS[CURRENT_MENU_POS].maxVal <= SETTINGS[CURRENT_MENU_POS].minVal)
  {
    (*(SETTINGS[CURRENT_MENU_POS].handler))();
    return;
  }
  if (MENU_IN_LOWER_LEVEL)
  {
    SETTINGS[CURRENT_MENU_POS].currentVal = TEMP_VAL;
    MENU_IN_LOWER_LEVEL = false;
  }
  else
  {
    TEMP_VAL = SETTINGS[CURRENT_MENU_POS].currentVal;
    MENU_IN_LOWER_LEVEL = true;
  }
}

void buttonLongAction()
{
  Serial.print("buttonLongAction");
  Serial.println();
  CURRENT_MENU_LEVEL++;
  if (CURRENT_MENU_LEVEL > 1)
  {
    CURRENT_MENU_LEVEL = 0;
  }
}

void counter()
{
  //Update count
  readPulses++;
}
void encoderISR()
{
  if (testDT)
  {
    rotEncPos--;
  }
  else
  {
    rotEncPos++;
  }
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

void setup()
{
  MOTOR_STATE MOTOR_STATE_DATA[6];
  MOTOR_STATE *MOTOR_STATE_DATA_ptrs[6];
  PARAMETER SETTINGS[7];
  PARAMETER *SETTINGS_ptrs[7];
  MOTOR_SETTINGS MOTOR_PARAMETER[3];
  MOTOR_SETTINGS *MOTOR_PARAMETER_ptrs[3];

  MOTOR_STATE_DATA[0] = {"MOTOR STOP          ", 0, 0, 0};
  MOTOR_STATE_DATA[1] = {"PAUSE BEFORE        ", 0, 0, 0};
  MOTOR_STATE_DATA[2] = {"FADE IN             ", 0, 0, 0};
  MOTOR_STATE_DATA[3] = {"OBROTY W PRAWO      ", 0, 0, 0};
  MOTOR_STATE_DATA[4] = {"OBROTY W LEWO       ", 0, 0, 0};
  MOTOR_STATE_DATA[5] = {"FADE OUT            ", 0, 0, 0};

  SETTINGS[0] = {"PREDKOSC", 0, 1000, 1, 1, formatUlamki};
  SETTINGS[1] = {"SREDNICA", 1, 250, 1, 1, formatInt};
  SETTINGS[2] = {"KIERUNEK OBR", 0, 1, 1, 1, formatDir};
  SETTINGS[3] = {"PAUZA PRZED", 0, 5000, 10, 1, formatInt};
  SETTINGS[4] = {"CZAS ROZPEDU", 0, 5000, 1, 1, formatInt};
  SETTINGS[5] = {"CZAS ZWALNIANIA", 0, 5000, 1, 1, formatInt};
  SETTINGS[6] = {"WYZWALACZ", 0, 1, 1, 1, triger};

  for (int i = 0; i < 7; i++)
  {
    SETTINGS_ptrs[i] = &SETTINGS[i];
  }

  MOTOR_PARAMETER[0] = {"BIEG-1", 0, 1000, 0, 1, formatUlamki};
  MOTOR_PARAMETER[1] = {"BIEG-2", 0, 1000, 0, 1, formatUlamki};
  MOTOR_PARAMETER[2] = {"PWM", 0, 1023, 0, 1, formatInt};
  for (int i = 0; i < 3; i++)
  {
    MOTOR_PARAMETER_ptrs[i] = &MOTOR_PARAMETER[i];
  }
  Serial.begin(115200);

  attachInterrupt(0, counter, FALLING);
  attachInterrupt(1, encoderISR, FALLING);
  Display_Manager.init(&lcd);
  // Display_Manager.drawMainScreen();
  MENU_SIZE = sizeof(SETTINGS) / sizeof(PARAMETER);

  CURRENT_MENU_POS = 0;
  LAST_MENU_POS = -1;

  CURRENT_MENU_LEVEL = 0;
  LAST_MENU_LEVEL = -1;

  MENU_IS_ON = false;
  HOME_SCREEN_CURSOR_POSITION = 0;

  // put your setup code here, to run once:
}

void loop()
{
  // Serial.println("loop");
  // Serial.println();
  // Serial.println("-------------------------------");
  // Serial.println();

  // delay(100);
  // readRotEnc();
  // ButtonSW();
  // Display_Manage();
}