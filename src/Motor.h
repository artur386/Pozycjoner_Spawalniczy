#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <Arduino.h>
#include <Wire.h>
#include <FastPWMdac.h>
#include "EnumTypes.h"

#define MAX_PWM_VALUE 1020
#define MIN_PWM_VALUE 0

class Motor
{
private:
    // relays pins
    byte rl_cw;
    byte rl_ccw;
    byte rl_gear;
    byte rl_start_stop;

    //pwm pin
    byte pwm_pin;
    FastPWMdac _fastPWMdac;

    uint16_t PWM_10BitSET; //
    uint16_t PWM_10BitOUT; //
    uint16_t PWM_10BitLst; //
    uint16_t passedTime;
    uint8_t motorGear;

    unsigned long timeStart;
    unsigned long timeLeft;

    // dane odnośnie predkości i obrotów
    uint16_t weldingDiameter; // średnica spawanego elementu [mm]
    float VC;                 // predkosc spawania [m/min]
    float rpm;                // ilość obrotów na minutę [rpm]

    uint8_t *Motor_Status;

    bool pauseBeforeOn;

    // bool pauseBeforeOff;
    bool softStartOn;
    // bool softStartOff;
    bool smoothStopOn;
    // bool smoothStopOff;
    bool motorStartOn;
    bool motorRunOn;
    bool motorRelayOn;
    bool MOTOR_CCW_DIR;

    uint16_t softStartTIME;   // ms
    uint16_t smoothStopTIME;  // ms
    uint16_t pauseBeforeTIME; // ms

    // uint8_t motorState; // 0:stop  1:pause; 2:fadeUP; 3:CW; 4: CCW; 5:fadeDOWN; 6:to stop
    bool PwmIsUpdated;

    void relayStart();
    void relayStop();
    void StopMotion();
    void motorStartFunc();
    void motorRunFunc();
    void motorGoStopFunc();
    void OutputPWM_Signal(uint16_t _pwm);

public:
    // Motor(FastPWMdac &fastPWMdac) : _fastPWMdac(fastPWMdac)
    Motor(byte _rl_cw, byte _rl_ccw, byte _rl_gear, byte _rl_start_stop, byte _pwm_pin);

    void SetSoftStartTime(int time);
    void BindMotorState(uint8_t *MotorSt);
    void SetSmoothStopTime(int time);
    void SetPauseTime(int time);
    uint8_t GetState();
    float RPM_TO_VC(float _rpm);
    float VC_TO_RPM(float _vc);
    float M_PER_MIN_to_MM_SEC(float _vc);
    float MM_PER_SEC_to_M_PER_MIN(float _vc);

    void motorStart(bool dir);
    void SET_PWM(int _pwm);
    void motorStop();
    void motorManage();

    // CONVERT [MM/SEC] TO [M/MIN]
};

#endif
