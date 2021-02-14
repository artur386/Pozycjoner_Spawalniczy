// klasa motor
#include "Motor.h"
// Motor(FastPWMdac &fastPWMdac) : _fastPWMdac(fastPWMdac)
Motor::Motor(byte _rl_cw, byte _rl_ccw, byte _rl_gear, byte _rl_start_stop, byte _pwm_pin)
{
    this->rl_cw = _rl_cw;
    this->rl_ccw = _rl_ccw;
    this->rl_gear = _rl_gear;
    this->rl_start_stop = _rl_start_stop;
    this->pwm_pin = _pwm_pin;
    motorRun = false;
    //  this->this->

    pinMode(rl_cw, OUTPUT);
    digitalWrite(rl_cw, LOW);
    pinMode(rl_ccw, OUTPUT);
    digitalWrite(rl_ccw, LOW);
    pinMode(rl_gear, OUTPUT);
    digitalWrite(rl_gear, LOW);
    pinMode(rl_start_stop, OUTPUT);
    digitalWrite(rl_gear, LOW);
}

void Motor::SetTimes(int *pause, int *softStart, int *smoothStop)
{
    pauseBeforeTIME = pause;
    softStartTIME = softStart;
    smoothStopTIME = smoothStop;
}
void Motor::SetGears(uint8_t *motorGear)
{
    this->motorGear = motorGear;
}
void Motor::BindMotorState(uint8_t *MotorSt)
{
    Motor_Status = MotorSt;
}

uint8_t Motor::GetState()
{
    return *Motor_Status;
}
void Motor::BindCCWDir(bool *MOTOR_CCW_DIR)
{
    this->MOTOR_CCW_DIR = MOTOR_CCW_DIR;
}

void Motor::motorStart()
{
    if (*this->motorGear == 1)
    {
        digitalWrite(rl_gear, LOW);
    }
    else
    {
        digitalWrite(rl_gear, HIGH);
    }
    if (*this->MOTOR_CCW_DIR)
    {
        MOTOR_CW_START();
        // digitalWrite(rl_ccw, LOW);
        // digitalWrite(rl_cw, HIGH);
        DBG("MOTOR CW");
    }
    else
    {
        MOTOR_CCW_START();
        // digitalWrite(rl_cw, LOW);
        // digitalWrite(rl_ccw, HIGH);
        DBG("MOTOR CCW");
    }

    this->motorStartOn = true;
    if (*pauseBeforeTIME > 0)
    {
        *Motor_Status = 1;
    }
    else if (*softStartTIME > 0)
    {
        *Motor_Status = 2;
    }
    else
    {
        if (*this->MOTOR_CCW_DIR)
        {
            *Motor_Status = 3;
        }
        else
        {
            *Motor_Status = 4;
        }
    }
}

void Motor::DoPause()
{
    if (*pauseBeforeTIME > 0)
    {
        if (!pauseBeforeOn)
        {
            LastPauseTime = millis();
            pauseBeforeOn = true;
        }
        if (pauseBeforeOn)
        {
            // do
            // {
            delay(*pauseBeforeTIME);
            // } while (millis() - LastPauseTime < *pauseBeforeTIME);
            pauseBeforeOn = false;
        }
    }
    *Motor_Status = 2;
}
void Motor::BindTimer1(TimerOne *timer_)
{
    this->TimerJeden = timer_;
}
void Motor::DoSofrStart(int pwm)
{

    if (*softStartTIME > 0)
    {
 
    }
    // else
    // {
    if (*this->MOTOR_CCW_DIR)
    {
        *Motor_Status = MOTOR_OBR_LEWO;
    }
    else
    {
        *Motor_Status = MOTOR_OBR_PRAWO;
    }
    // }
}
void Motor::DoSmoothStop(int pwm)
{
    if (*smoothStopTIME == 0)
    {
        *smoothStopTIME = 100;
    }

    if (*smoothStopTIME)
    {
        if (!smoothStopOn)
        {
            LastSmoothStopTime = millis();
            smoothStopOn = true;
        }
        if (smoothStopOn)
        {
            while (millis() - LastSmoothStopTime < uint32_t(*smoothStopTIME))
            {
                int pwmb = (int)map((millis() - LastSmoothStopTime), 0, *smoothStopTIME, pwm, 0);
                TimerJeden->pwm(this->pwm_pin, pwmb);
                delay(100);
            }
            TimerJeden->pwm(this->pwm_pin, 0);

            *Motor_Status = MOTOR_GO_TO_STOP;
            smoothStopOn = false;
        }
    }
    else
    {
        *Motor_Status = MOTOR_GO_TO_STOP;
    }
}

void Motor::OutputPWM_Signal(int _pwm)
{
    if (this->PWM_10BitOUT != _pwm)
    {
        this->PWM_10BitOUT = _pwm;
        DBG(this->PWM_10BitOUT);
        TimerJeden->pwm(this->pwm_pin, this->PWM_10BitOUT);
        this->PWM_10BitLst = this->PWM_10BitOUT;
    }
}

void Motor::SET_PWM(int _pwm)
{
    if (_pwm > MAX_PWM_VALUE)
    {
        PWM_10BitSET = MAX_PWM_VALUE;
    }
    else if (_pwm < MIN_PWM_VALUE)
    {
        PWM_10BitSET = MIN_PWM_VALUE;
    }
    else
    {
        this->PWM_10BitSET = _pwm;
    }
    OutputPWM_Signal(PWM_10BitSET);
}
void Motor::motorStop()
{

    this->motorStopOn = true;
    this->StopMotion();
    *Motor_Status = 0;
}

void Motor::StopMotion()
{
    OutputPWM_Signal(0);
    delay(100);
    MOTOR_STOP();
    // relayStop();
}

void Motor::relayStart()
{
    if (!motorRelayOn)
    {
        digitalWrite(rl_start_stop, HIGH);
        motorRelayOn = true;
    }
}

void Motor::relayStop()
{
    if (motorRelayOn)
    {
        digitalWrite(rl_start_stop, LOW);
        delay(120);
        digitalWrite(rl_cw, LOW);
        digitalWrite(rl_ccw, LOW);
        digitalWrite(rl_gear, LOW);
        motorRelayOn = false;
    }
}
