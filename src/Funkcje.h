#ifndef FUNKCJE_H
#define FUNKCJE_H
#include "Arduino.h"
#include "enums.h"
#include "GlobalVariable.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "RTClib.h"
#include <PID_v1.h>
#include <EEPROM.h>
#include "BigFont.h"
#include "AppIdle.h"
#include "AppMenu.h"
#include "AppMotor.h"

void SetButton(uint8_t btn);
void EncWrite(int8_t addVal);
void WriteEEpromData(void);
void ReadEEpromData(void);
void controlBtn(void);
bool CheckAnyBtn(void);
void ClearButton(uint8_t btn);
bool ChceckBtn(uint8_t btn);
void read_rotary(void);
void APP_MENU_MODE(void);
void getpidparam(void);
void appManage(void);
void APP_MOTOR_MODE(void);
void tach_interrupt(void);
void appIdle(void);
void FastParamChange(int mode);
void WriteParamToEEprom(void);
uint16_t CalcPWM(float rpm);
bool CheckFastParameterChange(void);
// void BindParameters(void);
// float RpmToMmin(float rpm, int dia);
float RpmToMMsec(float rpm, int dia);
float MMsecToRpm(float mmSec, int dia);
double GetCalcParam(uint8_t par);
// float MMsecToRpm(float mmSec, int dia);

LiquidCrystal_I2C lcd(0x27, 20, 4);
// PID myPID(&RealRpm, &outPWM, &SetRpm, GetCalcParam(7), GetCalcParam(8), GetCalcParam(9), 0);
DisplayManager Display(&lcd, &MOTOR_STATE);
Motor motor(RL_MOTOR_CW, RL_MOTOR_CCW, RL_MOTOR_GEAR, RL_MOTOR_START_STOP, PWM_DAC);
myMenu menu(&lcd, &parameters[p_RPM], &BT_ST, &AppMode);
extern RTC_DS3231 rtc;
extern BigFont fontPrinter;

/*Funkcje*/

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
double GetCalcParam(uint8_t par)
{
    return ((parameters[par].val + parameters[par].offset) / pow(10, parameters[par].precision));
}
void focusInc()
{
    focus = !focus;
    WriteEEpromData();
    Display.updateScreen();
}
float RpmToMMsec(float rpm, int dia)
{
    return (PI * dia * rpm) / 60.0f;
}

float MMsecToRpm(float mmSec, int dia)
{
    return (60.0f * mmSec) / (PI * float(dia));
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
    DBG(parameters[p_BiegSilnk].val);
    if (parameters[p_BiegSilnk].val == 0)
    {
        DBGF("CALC PWM G1");
        return map(parameters[p_RPM].val, (MinRPM_1stGear * 100), (MaxRPM_1stGear * 100), MIN_PWM_VALUE, MAX_PWM_VALUE);
    }
    else if (parameters[p_BiegSilnk].val == 1)
    {
        DBGF("CALC PWM G2");
        return map(parameters[p_RPM].val, (MinRPM_2stGear * 100), (MaxRPM_2stGear * 100), MIN_PWM_VALUE, MAX_PWM_VALUE);
    }
    return 0;
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
        RealRpm = 0;
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
                    RealRpm = 40000.0 / (accumulator >> 3);
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
                RealRpm = ((revCountCopyAvg / 1500.0f) / timPeroid) * 60000000.0f;
                RealMMsec = RpmToMMsec(RealRpm, parameters[p_DIA].val);
                lastUpdate = millis();
            }
            Serial.println(RealRpm, 4);
        }
        break;

    default:
        break;
    }
}

void PID_ON_OFF()
{
    if (bool(parameters[p_PidOnOff].val) != PidOnOff)
    {
        if (!bool(parameters[p_PidOnOff].val))
        {
            DBG("AUTOMATIC PID");
            // myPID.SetSampleTime(SAMPLE_TIME);
            // myPID.SetTunings(GetCalcParam(7), GetCalcParam(8), GetCalcParam(9));
            // myPID.SetMode(AUTOMATIC);
        }
        else
        {
            DBG("MANUAL PID");
            // myPID.SetMode(MANUAL);
            // Output = open_loop_pwm;
        }
        PidOnOff = bool(parameters[p_PidOnOff].val);
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

void WriteEEpromData()
{
    EEPROM.put(EEPROM_START_ADRESS, parameters);
    // BindParameters();
    // EEPROM.put(sizeof(values), enu);

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

// void BindParameters()
// {
//     paramSpeedMethod = (parameters[p_UNIT].val);
//     // paramDIA = (parameters[p_DIA].val);
//     paramTimePause = (parameters[p_SCR_SAVE].val);
//     paramTimeStart = (parameters[p_START_T].val);
//     paramTimeStop = (parameters[p_STOP_T].val);
//     // paramRPM = (parameters[p_RPM].val / float(parameters[p_RPM].precision ^ 10));
//     // paramRPS = (parameters[p_DIA].val / float(parameters[p_DIA].precision ^ 10));
//     paramMMSEC = (parameters[p_MMSEC].val / pow(10, parameters[p_MMSEC].precision));
//     paramGEAR = (parameters[p_BiegSilnk].val);
//     paramPidOnOff = (bool)(parameters[p_PidOnOff].val);
//     paramKp = (parameters[p_KP].val / pow(10, parameters[p_KP].precision));
//     paramKi = (parameters[p_KI].val / pow(10, parameters[p_KI].precision));
//     paramKd = (parameters[p_KD].val / pow(10, parameters[p_KD].precision));
//     paramScreenSaver = parameters[p_DIA].val * 1000;
// }
void ReadEEpromData()
{
    // Odczyt ustawień z pamięci EEPROM
    EEPROM.get(EEPROM_START_ADRESS, parameters);
    // EEPROM.get(sizeof(values), enu);
    // BindParameters();
    // for (size_t i = 0; i < 14; i++)
    // {
    //     Serial.print("param: ");
    //     Serial.print(i);
    //     Serial.print(": val: ");
    //     Serial.println(parameters[i].val);

    //     Serial.print("param: ");
    //     Serial.print(i);
    //     Serial.print(": inc: ");
    //     Serial.println(parameters[i].inc);

    //     Serial.print("param: ");
    //     Serial.print(i);
    //     Serial.print(": max: ");
    //     Serial.println(parameters[i].maxVal);

    //     Serial.print("param: ");
    //     Serial.print(i);
    //     Serial.print(": precision: ");
    //     Serial.println(parameters[i].precision);

    //     Serial.print("param: ");
    //     Serial.print(i);
    //     Serial.print(": unitNum: ");
    //     Serial.println(parameters[i].unitNum);
    // }
}

bool CheckFastParameterChange()
{
    if (ChceckBtn(BTN_UP))
    {
        FastParamChange(0);
        return true;
    }
    else if (ChceckBtn(BTN_DOWN))
    {
        FastParamChange(1);
        return true;
    }
    else if (ChceckBtn(BTN_LEFT))
    {
        FastParamChange(3);
        return true;
    }
    else if (ChceckBtn(BTN_RIGHT))
    {
        FastParamChange(2);
        return true;
    }
    return false;
}

void FastParamChange(int mode)
{
    DBG("FAST PARAM CHANGE");
    int8_t val;
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
    if (parameters[p_UNIT].val == 0)
    {
        // val = val / VAL_TBL__DIV(0);
        parameters[p_RPM].val += val;
        if (parameters[p_RPM].val > int(parameters[p_RPM].maxVal))
        {
            parameters[p_RPM].val = parameters[p_RPM].maxVal;
        }
        else if (parameters[p_RPM].val < 20)
        {
            parameters[p_RPM].val = 20;
        }

        parameters[p_MMSEC].val = int(RPM_TO_MMSEC(GetCalcParam(0), GetCalcParam(2)) * pow(10, parameters[p_MMSEC].precision));
    }
    else if (parameters[p_UNIT].val == 1)
    {
        if (!focus)
        {
            //srednica
            parameters[p_DIA].val += val;
            if (parameters[p_DIA].val > int(parameters[p_DIA].maxVal))
            {
                parameters[p_DIA].val = parameters[p_DIA].maxVal;
            }
            else if (parameters[p_DIA].val < parameters[p_DIA].offset)
            {
                parameters[p_DIA].val = parameters[p_DIA].offset;
            }
            parameters[p_RPM].val = (int)(MMsecToRpm(GetCalcParam(1), GetCalcParam(2)) * pow(10, parameters[p_RPM].precision));
        }
        else
        {
            parameters[p_MMSEC].val += val;
            if (parameters[p_MMSEC].val > int(parameters[p_MMSEC].maxVal))
            {
                parameters[p_MMSEC].val = parameters[p_MMSEC].maxVal;
            }
            else if (parameters[p_MMSEC].val < 20)
            {
                parameters[p_MMSEC].val = 20;
            }
            parameters[p_RPM].val = (int)(MMsecToRpm(GetCalcParam(1), GetCalcParam(2)) * pow(10, parameters[p_RPM].precision));
        }
    }

    Display.updateScreen();
    LastEEpromWriteTime = millis();
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

void appMotor()
{
    rpmCalc();
    if (LastAppMode != AppMode)
    {
        DBGF("MOTOR INIT");
        Display.drawMainScreen();
        Display.updateScreen();
        WriteToEEprom_flag = true;
        WriteParamToEEprom();
        LastAppMode = AppMode;
        PID_ON_OFF();
        setPWM = 0;
        outPWM = 0;
        MotorPauseFlag = true;
        MotorSoftStartFlag = true;
        MotorSmoothStopFlag = true;
        softStartOn = false;
        Timer1.pwm(PWM_DAC, outPWM);
        if (MOTOR_CCW_DIR)
        {
            DBGF("MOTOR CCW");
            digitalWrite(RL_MOTOR_CW, LOW);
            delay(50);
            digitalWrite(RL_MOTOR_CCW, HIGH);
            delay(50);
            digitalWrite(RL_MOTOR_START_STOP, HIGH);
        }
        else
        {
            DBGF("MOTOR CW");
            digitalWrite(RL_MOTOR_CCW, LOW);
            delay(50);
            digitalWrite(RL_MOTOR_CW, HIGH);
            delay(50);
            digitalWrite(RL_MOTOR_START_STOP, HIGH);
        }
        if (parameters[p_BiegSilnk].val == 1)
        {
            DBGF("MOTOR GEAR 2");
            digitalWrite(RL_MOTOR_GEAR, HIGH);
        }
        else if (parameters[p_BiegSilnk].val == 0)
        {
            DBGF("MOTOR GEAR 1");
            digitalWrite(RL_MOTOR_GEAR, LOW);
        }
        setPWM = CalcPWM(GetCalcParam(0));
    }
    // DBG(ENC_STATE);

    if (MotorStartFlag && !MotorRunFlag)
    {
        if (MotorPauseFlag && parameters[p_SCR_SAVE].val != 0)
        {
            DBGF("MOTOR PAUSE");
            MOTOR_STATE = MOTOR_PAUSE;
            Display.updateScreen();
            delay(parameters[p_SCR_SAVE].val);
            MotorPauseFlag = false;
        }
        else
        {
            MotorPauseFlag = false;
        }
        if (MotorSoftStartFlag && parameters[p_START_T].val != 0)
        {
            if (!softStartOn)
            {
                DBGF("MOTOR SOFT START INIT");
                MOTOR_STATE = MOTOR_SOFT_START;
                LastSoftStartTime = millis();
                softStartPWM = outPWM;
                softStartOn = true;
                ENC_STATE = 1;
            }
            if (softStartOn)
            {
                unsigned int elapsedTime = millis() - LastSmoothStopTime;

                if (elapsedTime < uint32_t(parameters[p_START_T].val))
                {
                    outPWM = map(elapsedTime, 0, parameters[p_START_T].val, softStartPWM, setPWM);
                }
                else
                {
                    outPWM = setPWM;
                    MotorSoftStartFlag = false;
                    MotorRunFlag = true;
                    softStartOn = false;
                }
                Timer1.pwm(PWM_DAC, outPWM);
            }
        }
        else
        {
            MotorRunFlag = true;
        }
    }
    if (MotorRunFlag && MotorStartFlag)
    {
        if (MOTOR_CCW_DIR)
        {
            DBGF("MOTOR LEWO");
            MOTOR_STATE = MOTOR_OBR_LEWO;
        }
        else
        {
            DBGF("MOTOR PRAWO");
            MOTOR_STATE = MOTOR_OBR_PRAWO;
        }
        MotorStartFlag = false;
    }
    if (MotorRunFlag && !MotorStartFlag)
    {
        // DBG();
        // myPID.Compute();
        Timer1.pwm(PWM_DAC, outPWM);
        if (ChceckBtn(BTN_START_STOP_SHORT))
        {
            MotorStopFlag = true;
        }
    }
    if (MotorRunFlag && MotorStopFlag)
    {
        DBGF("MOTOR STOP");
        if (parameters[p_STOP_T].val != 0)
        {
            if (!smoothStopOn)
            {
                MOTOR_STATE = MOTOR_SMOOTH_STOP;
                LastSmoothStopTime = millis();
                smoothStopOn = true;
                smoothStopPWM = outPWM;
            }
            if (smoothStopOn)
            {
                unsigned int elapsedTime = millis() - LastSmoothStopTime;

                if (elapsedTime < uint32_t(parameters[p_STOP_T].val))
                {
                    outPWM = map(elapsedTime, 0, parameters[p_STOP_T].val, smoothStopPWM, 0);
                    Timer1.pwm(PWM_DAC, outPWM);
                    delay(25);
                }
                else
                {
                    outPWM = 0;
                    Timer1.pwm(PWM_DAC, outPWM);
                    smoothStopOn = false;
                    MotorSoftStartFlag = false;
                    MOTOR_STATE = MOTOR_STOP;
                }
            }
        }
    }
    if (MOTOR_STATE == MOTOR_STOP)
    {
        digitalWrite(RL_MOTOR_START_STOP, LOW);
        delay(100);
        digitalWrite(RL_MOTOR_CCW, LOW);
        delay(10);
        digitalWrite(RL_MOTOR_CW, LOW);
        delay(10);
        digitalWrite(RL_MOTOR_GEAR, LOW);
        AppMode = APP_IDLE_VIEW;
        MotorRunFlag = false;
    }

    Display.updateScreen();
}

// // Setpoint = GetCalcParam(0);
// if (ChceckBtn(BTN_START_STOP_SHORT))
// {
//     if (MOTOR_STATE == MOTOR_STOP)
//     {
//         motor.motorStart();
//     }
//     else if (MOTOR_STATE == MOTOR_OBR_PRAWO || MOTOR_STATE == MOTOR_OBR_LEWO)
//     {
//         MOTOR_STATE = MOTOR_SMOOTH_STOP;
//     }
//     else if (MOTOR_STATE == MOTOR_PAUSE)
//     {
//         MOTOR_STATE = MOTOR_PAUSE;
//     }
//     else if (MOTOR_STATE == MOTOR_SOFT_START)
//     {
//         MOTOR_STATE = MOTOR_SMOOTH_STOP;
//     }
// }

// if (ChceckBtn(BTN_SW_LONG))
// {
//     AppMode = APP_MENU_VIEW;
// }
// switch (MOTOR_STATE)
// {
// case MOTOR_STOP: //stop
//     if (LAST_MOTOR_STATE != MOTOR_STATE)
//     {
//         LAST_MOTOR_STATE = MOTOR_STATE;
//         AppMode = APP_IDLE_VIEW;
//         Display.ForceRefresh();
//     }
//     break;

// case MOTOR_PAUSE: //pause
//     if (LAST_MOTOR_STATE != MOTOR_STATE)
//     {
//         LAST_MOTOR_STATE = MOTOR_STATE;
//         motor.DoPause();
//         RED_LED_OFF();
//         GREEN_LED_ON();
//         Display.ForceRefresh();
//     }
//     break;

// case MOTOR_SOFT_START:
//     if (LAST_MOTOR_STATE != MOTOR_STATE)
//     {
//         motor.relayStart();
//         motor.DoSofrStart(Output);
//         RED_LED_OFF();
//         GREEN_LED_ON();
//         LAST_MOTOR_STATE = MOTOR_STATE;
//         Display.ForceRefresh();
//     }
//     break;

// case MOTOR_OBR_PRAWO:
//     if (LAST_MOTOR_STATE != MOTOR_STATE)
//     {
//         LAST_MOTOR_STATE = MOTOR_STATE;
//         motor.relayStart();
//         Display.ForceRefresh();
//     }
//     if (ChceckBtn(BTN_START_STOP_SHORT))
//     {
//         MOTOR_STATE = MOTOR_SMOOTH_STOP;
//     }
//     myPID.Compute();
//     motor.SET_PWM(Output);
//     DBG(Output);
//     RED_LED_OFF();
//     GREEN_LED_ON();
//     break;

// case MOTOR_OBR_LEWO:
//     if (LAST_MOTOR_STATE != MOTOR_STATE)
//     {
//         LAST_MOTOR_STATE = MOTOR_STATE;
//         motor.relayStart();

//         Display.ForceRefresh();
//         RED_LED_OFF();
//         GREEN_LED_ON();
//     }
//     if (ChceckBtn(BTN_START_STOP_SHORT))
//     {
//         MOTOR_STATE = MOTOR_SMOOTH_STOP;
//     }

//     break;

// case MOTOR_SMOOTH_STOP:
//     if (LAST_MOTOR_STATE != MOTOR_STATE)
//     {
//         LAST_MOTOR_STATE = MOTOR_STATE;
//         motor.DoSmoothStop(Output);
//         Display.ForceRefresh();
//     }
//     break;

// case MOTOR_GO_TO_STOP:
//     if (LAST_MOTOR_STATE != MOTOR_STATE)
//     {
//         LAST_MOTOR_STATE = MOTOR_STATE;
//         motor.motorStop();
//         MOTOR_STOP();
//         ENC_STATE = MOTOR_STOP;
//         RED_LED_ON();
//         GREEN_LED_OFF();
//         Display.ForceRefresh();
//     }

//     break;

// default:
//     break;
// }
// if (MOTOR_STATE == MOTOR_OBR_LEWO || MOTOR_STATE == MOTOR_OBR_PRAWO)
// {
//     CheckFastParameterChange();
//     if (bool(parameters[p_PidOnOff].val))
//     {
//         myPID.Compute();
//     }
//     else
//     {
//         Output = CalcPWM(GetCalcParam(0));
//     }
//     motor.SET_PWM(Output);
// }
// if (paramPidOnOff)
// {
//     if (paramLastRPM != (parameters[p_RPM].val / pow(10, parameters[p_RPM].precision)))
//     {
//         motor.SET_PWM(CalcPWM(parameters[p_RPM].val / pow(10, parameters[p_RPM].precision)));
//     }
// }
// else
// {
//     motor.SET_PWM(Output);
// }
// Display.updateScreen();
// }

/******************************
 * app menu
 * ***************************/

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

/*************************
 *  APP IDLE
 * ************************/

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
    }

    if (LastAppMode != AppMode)
    {
        if (LastAppMode == APP_MENU_VIEW)
        {
            WriteEEpromData();
        }
        ScreenSaverOn = true;
        ScreenSaverTime = millis();

        Display.drawMainScreen();
        Display.ForceRefresh();
        Display.updateScreen();
        LastAppMode = AppMode;
    }
    if (CheckAnyBtn())
    {
        if (ShowScreenSaver)
        {
            ShowScreenSaver = false;
            Display.drawMainScreen();
            BT_ST = 0x00000000;
        }
        else if (CheckFastParameterChange())
        {
            // Display.ForceRefresh();
            // Display.updateScreen();
        }
        else if (ChceckBtn(BTN_START_STOP_SHORT))
        {
            AppMode = APP_MOTOR_VIEW;
            // motor.motorStart();
            MotorStartFlag = true;
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
            LedAcceptMode = 0;
            focusInc();
        }

        else if (ChceckBtn(BTN_START_STOP_LONG))
        {
            LedAcceptFlag = true;
            LedAcceptMode = 1;
            MOTOR_CCW_DIR = !MOTOR_CCW_DIR;
        }

        Display.ForceRefresh();
        Display.updateScreen();
        ScreenSaverTime = millis();
    }
    if (ScreenSaverOn)
    {
        if ((millis() - ScreenSaverTime) >= (parameters[p_DIA].val * 1000) && !ShowScreenSaver)
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
            float temp = rtc.getTemperature();
            char *timeStr = (char *)malloc(sizeof(char) * 9);
            char *hours = (char *)malloc(sizeof(char) * 5);
            char *minutes = (char *)malloc(sizeof(char) * 5);
            char *seconds = (char *)malloc(sizeof(char) * 5);
            char *tempStrf = (char *)malloc(sizeof(char) * 10);

            dtostrf(temp, 5, 1, tempStrf);
            //hour
            if (now.hour() > 9)
            {
                sprintf(hours, "%d", now.hour());
            }
            else
            {
                sprintf(hours, "   %d", now.hour());
            }

            //minutes
            if (now.minute() > 9)
            {
                sprintf(minutes, "%d", now.minute());
            }
            else
            {
                sprintf(minutes, "   %d", now.minute());
            }

            //second
            if (now.second() > 9)
            {
                sprintf(seconds, "%d", now.second());
            }
            else
            {
                sprintf(seconds, "   %d", now.second());
            }

            sprintf(timeStr, "%s:%s:%s", hours, minutes, seconds);
            free(hours);
            free(minutes);
            free(seconds);
            fontPrinter.WriteBigString(timeStr, 0, 1);
            free(timeStr);
            char *tempStr = (char *)malloc(sizeof(char) * 20);
            sprintf(tempStr, "temp: %s", tempStrf);
            free(tempStrf);
            lcd.setCursor(0, 3);
            lcd.print(tempStr);
            free(tempStr);
            ScreenRefreshTime = millis();
        }
    }
    // delay(500);
}

#endif // !FUNKCJE_H