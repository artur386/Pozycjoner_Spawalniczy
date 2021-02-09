#ifndef ENUMS_H
#define ENUMS_H
#include <avr/pgmspace.h>
// relay motor pins:
#define RL_MOTOR_CW 11
#define RL_MOTOR_CCW 8
#define RL_MOTOR_GEAR 10
#define RL_MOTOR_START_STOP A0

// pwm pin
#define PWM_DAC 9

// MOTOR INC ENCODER
#define MOTOR_ENCODER 2

// led
#define LED 13
#define RLED A2
#define GLED A1

/*inputs*/
// rotary encoder
#define ROT_ENC_CLK 3 // PIN A
#define ROT_ENC_DT 4  // Pin B
#define ROT_ENC_SW 6  // Switch

// PRZYCISK START/STOP
#define BTN_START_STOP_PANEL 7 //switch on front panel

// LEDS

#define NCO_0 0
#define NCO_1 1
#define NCO_2 5
#define NCO_3 12
#define NCI_0 A3
#define NCI_1 A4
#define NCI_2 A5
#define NCI_3 A6
#define NCI_4 A7
/**************************************
 * declaration enum etc:
 * ************************************/
enum BT_Modes
{
    BTN_UP = 0,
    BTN_DOWN,
    BTN_LEFT,
    BTN_RIGHT,
    BTN_SW_SHORT,
    BTN_SW_LONG,
    BTN_START_STOP_SHORT,
    BTN_START_STOP_LONG
};
enum AppEnumModes
{
    APP_MOTOR_VIEW = 0,
    APP_MENU_VIEW,
    APP_IDLE_VIEW
};
enum MotorStateEnum
{
    MOTOR_STOP = 0,
    MOTOR_PAUSE,
    MOTOR_SOFT_START,
    MOTOR_OBR_PRAWO,
    MOTOR_OBR_LEWO,
    MOTOR_SMOOTH_STOP,
    MOTOR_GO_TO_STOP
};
#define DEBUG 1
#ifdef DEBUG
#define DBG(x)          \
    Serial.print(#x);   \
    Serial.print(": "); \
    Serial.println(x)

#else
#define DBG(NULL)
#endif // DEBUG
// lcd
#define LCD_ADRESS 0x27
#define LCD_ROWS 4
#define LCD_COLS 20

// PWM DATA:
#define MAX_PWM_VALUE 1015
#define MIN_PWM_VALUE 85

// MOTOR SPEED:
#define MinRPM_1stGear 0.0800
#define MaxRPM_1stGear 1.8000
#define MinRPM_2stGear 0.0800
#define MaxRPM_2stGear 3.1400

// TACHOMETER
#define TACHO_STYLE 2 // 1,2

// EEPROM VARIABLES
#define EEPROM_START_ADRESS 0
#define EEpromWriteTime 10000

#define CONTROL_button_long_press 750  // ms
#define CONTROL_button_short_press 100 // ms

#define MM_PER_SEC_to_M_PER_MIN(vc) (vc * 0.0166)
#define M_PER_MIN_to_MM_SEC(vc) (vc * 16.6666)
#define VC_TO_RPM(vc) (vc * 1000) / (PI * paramDIA)
#define RPM_TO_VC(rpm) (PI * (*paramDIA) * rpm) / 1000.0f

#define RPM_TO_MMSEC(rpm, dia) (((PI * dia * rpm) / 1000.0f) * 16.6666f)
#define MMSEC_TO_RPM(mmsec, dia) (((mmsec * 0.0166) * 1000.0f) / (PI * dia))

#define RED_LED_ON() digitalWrite(RLED, HIGH)
#define RED_LED_OFF() digitalWrite(RLED, LOW)
#define RED_LED_TOGGLE() digitalWrite(RLED, !digitalRead(RLED);
#define GREEN_LED_ON() digitalWrite(GLED, HIGH)
#define GREEN_LED_OFF() digitalWrite(GLED, LOW)
#define GREEN_LED_TOGGLE() digitalWrite(GLED, !(bool)digitalRead(GLED))
#define PWR_LED_ON digitalWrite(LED, HIGH)
#define PWR_LED_OFF digitalWrite(LED, LOW)
#define PWR_LED_CHANGE digitalWrite(LED, !digitalRead(LED))

#define MOTOR_CW_START()             \
    digitalWrite(RL_MOTOR_CCW, LOW); \
    delay(10);                       \
    digitalWrite(RL_MOTOR_CW, HIGH); \
    delay(10);                       \
    digitalWrite(RL_MOTOR_START_STOP, HIGH);
#define MOTOR_CCW_START()             \
    digitalWrite(RL_MOTOR_CW, LOW);   \
    delay(10);                        \
    digitalWrite(RL_MOTOR_CCW, HIGH); \
    delay(10);                        \
    digitalWrite(RL_MOTOR_START_STOP, HIGH);
#define MOTOR_STOP()                        \
    digitalWrite(RL_MOTOR_START_STOP, LOW); \
    delay(100);                             \
    digitalWrite(RL_MOTOR_CCW, LOW);        \
    delay(10);                              \
    digitalWrite(RL_MOTOR_CW, LOW);

#define VAL_TBL__MAX(id) pgm_read_word(&(val_table[id][0]))
#define VAL_TBL__INC(id) pgm_read_word(&(val_table[id][1]))
#define VAL_TBL__DIV(id) pgm_read_word(&(val_table[id][2]))
#define VAL_TBL__UNIT(id) pgm_read_word(&(val_table[id][3]))

#define GET_FLOAT_VAL(id) (values[id] / float(VAL_TBL__DIV(id)))

#define MENU_DATA_ID(menuNb) pgm_read_byte(&(menud[menuNb][0]))
#define MENU_DATA_TYP(menuNb) pgm_read_byte(&(menud[menuNb][1]))
#define MENU_DATA_LVL(menuNb) pgm_read_byte(&(menud[menuNb][2]))
#define MENU_DATA_CALL(menuNb) pgm_read_byte(&(menud[menuNb][3]))

#define ENUM_TABLE__MAX(id) pgm_read_byte(&(enu_table[id][0]))
#define ENUM_TABLE__TXT(id) pgm_read_byte(&(enu_table[id][1]))
#define ENU_VAL_CONTECT(id) pgm_read_byte(&(enu_table[id][2]))
#define SET_ENUM_P(id) \
    e_p = enu_p + id

#define ENUM_LABEL(buf, call) \
    SET_ENUM_P(call);         \
    GET_TEXT_ID(buf, *e_p + ENUM_TABLE__TXT(call))

#define SET_VALUE_P(id) \
    v_p = value_p + id

#define GET_TEXT_ID(buf, id) \
    strcpy_P(buf, (char *)pgm_read_word(&(id_table[id])))

#define MENU_NUMBER(_cursor) (this->menuOffset + this->scroll + _cursor)

const char ID_00[] PROGMEM = "Predkosc:";
const char ID_01[] PROGMEM = "Srednica:";
const char ID_02[] PROGMEM = "Ustawienia";
const char ID_03[] PROGMEM = "Wyjscie";
const char ID_04[] PROGMEM = "Ust. CZASU";
const char ID_05[] PROGMEM = "Jednostka Obrotow";
const char ID_06[] PROGMEM = "Reg PID";
const char ID_07[] PROGMEM = "Bieg Silnika:";
const char ID_08[] PROGMEM = "1";
const char ID_09[] PROGMEM = "2";
const char ID_10[] PROGMEM = "AUTO";
const char ID_11[] PROGMEM = "Wygaszacz:";
const char ID_12[] PROGMEM = "Powrot";
const char ID_13[] PROGMEM = "Pauza Start:";
const char ID_14[] PROGMEM = "Soft Start:";
const char ID_15[] PROGMEM = "Smooth Stop:";
const char ID_16[] PROGMEM = "Powrot";
const char ID_17[] PROGMEM = "UNIT:";
const char ID_18[] PROGMEM = "RPM";
const char ID_19[] PROGMEM = "RPS";
const char ID_20[] PROGMEM = "MM/S";
const char ID_21[] PROGMEM = "Powrot";
const char ID_22[] PROGMEM = "PID";
const char ID_23[] PROGMEM = "ON";
const char ID_24[] PROGMEM = "OFF";
const char ID_25[] PROGMEM = "Kp";
const char ID_26[] PROGMEM = "Ki";
const char ID_27[] PROGMEM = "Kd";
const char ID_28[] PROGMEM = "Powrot";
const char ID_29[] PROGMEM = "mm";
const char ID_30[] PROGMEM = "sec";
const char ID_31[] PROGMEM = "ms";
const char ID_32[] PROGMEM = "";

const char *const id_table[] PROGMEM = {ID_00, ID_01, ID_02, ID_03, ID_04, ID_05, ID_06, ID_07, ID_08, ID_09, ID_10, ID_11, ID_12, ID_13, ID_14, ID_15, ID_16, ID_17, ID_18, ID_19, ID_20, ID_21, ID_22, ID_23, ID_24, ID_25, ID_26, ID_27, ID_28, ID_29, ID_30, ID_31, ID_32};

const uint16_t val_table[][4] PROGMEM =
    {{300, 1, 100, 18},
     {50, 1, 1000, 19},
     {200, 1, 100, 20},
     {350, 1, 1, 29},
     {120, 1, 1, 30},
     {5000, 100, 1, 31},
     {5000, 100, 1, 31},
     {5000, 100, 1, 31},
     {500, 1, 100, 32},
     {500, 1, 100, 32},
     {500, 1, 100, 32}};

const uint8_t enu_table[][3] PROGMEM = {{2, 8, 0}, {2, 18, 0}, {1, 23, 0}};

const uint8_t menud[][4] PROGMEM = {{0, 4, 0, 1},
                                    {1, 1, 0, 3},
                                    {2, 0, 0, 1},
                                    {3, 3, 0, 0},
                                    {4, 0, 1, 2},
                                    {5, 0, 1, 3},
                                    {6, 0, 1, 4},
                                    {7, 2, 1, 0},
                                    {11, 1, 1, 4},
                                    {12, 0, 1, 0},
                                    {13, 1, 2, 5},
                                    {14, 1, 2, 6},
                                    {15, 1, 2, 7},
                                    {16, 0, 2, 1},
                                    {17, 2, 2, 1},
                                    {21, 0, 2, 1},
                                    {22, 2, 2, 2},
                                    {25, 1, 2, 8},
                                    {26, 1, 2, 9},
                                    {27, 1, 2, 10},
                                    {28, 0, 2, 1}};

const uint8_t menuSc[][2] PROGMEM = {
    {0, 4},
    {4, 6},
    {10, 4},
    {14, 2},
    {16, 5},
};

#endif // !1