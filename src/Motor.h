#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <Arduino.h>
#include <Wire.h>
#include <TimerOne.h>

#include "enums.h"

class Motor
{
private:
    // relays pins
    byte rl_cw;
    byte rl_ccw;
    byte rl_gear;
    byte rl_start_stop;

    TimerOne *TimerJeden;
    //pwm pin
    byte pwm_pin;

    int PWM_10BitSET; //
    int PWM_10BitOUT; //
    int PWM_10BitLst; //
    uint16_t passedTime;
    uint8_t *motorGear;

    unsigned long LastPauseTime, LastSoftStartTime, LastSmoothStopTime;
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
    bool motorStartOn, motorStopOn;
    bool motorRun;
    bool motorRelayOn;
    bool *MOTOR_CCW_DIR;

    int *softStartTIME;   // ms
    int *smoothStopTIME;  // ms
    int *pauseBeforeTIME; // ms

    // uint8_t motorState; // 0:stop  1:pause; 2:fadeUP; 3:CW; 4: CCW; 5:fadeDOWN; 6:to stop
    bool PwmIsUpdated;

    void relayStop();
    void StopMotion();
    void motorStartFunc();
    void motorRunFunc();
    void motorGoStopFunc();
    void OutputPWM_Signal(int _pwm);

public:
    // Motor(FastPWMdac &fastPWMdac) : _fastPWMdac(fastPWMdac)
    Motor(byte _rl_cw, byte _rl_ccw, byte _rl_gear, byte _rl_start_stop, byte _pwm_pin);

    void SetTimes(int *pause, int *softStart, int *smoothStop);
    void SetGears(uint8_t *motorGear);
    void BindCCWDir(bool *MOTOR_CCW_DIR);
    void SetValues(int *val, uint8_t *env);
    void BindMotorState(uint8_t *MotorSt);
    void BindTimer1(TimerOne *timer_);
    uint8_t GetState();
    void motorStart();
    void SET_PWM(int _pwm);
    void motorStop();
    void motorManage();
    void DoSmoothStop(int pwm);
    void DoSofrStart(int pwm);
    void DoPause();
    void relayStart();
};

#endif
