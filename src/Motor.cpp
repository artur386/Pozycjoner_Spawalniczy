// klasa motor
#include <Motor.h>

// Motor(FastPWMdac &fastPWMdac) : _fastPWMdac(fastPWMdac)
Motor::Motor(byte _rl_cw, byte _rl_ccw, byte _rl_gear, byte _rl_start_stop, byte _pwm_pin)
{
    this->rl_cw = _rl_cw;
    this->rl_ccw = _rl_ccw;
    this->rl_gear = _rl_gear;
    this->rl_start_stop = _rl_start_stop;
    this->pwm_pin = _pwm_pin;

    _fastPWMdac.init(pwm_pin, 10);
    // this->Motor_Status = 0;
    this->softStartTIME = 0;
    this->smoothStopTIME = 0;
    this->pauseBeforeTIME = 0;
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

void Motor::SetSoftStartTime(int time)
{
    softStartTIME = time;
}

void Motor::BindMotorState(uint8_t *MotorSt)
{
    Motor_Status = MotorSt;
}

uint8_t Motor::GetState()
{
    return *Motor_Status;
}
void Motor::SetSmoothStopTime(int time)
{
    smoothStopTIME = time;
}

void Motor::SetPauseTime(int time)
{
    pauseBeforeTIME = time;
}

float Motor::RPM_TO_VC(float _rpm)
{
    return ((PI * weldingDiameter * _rpm) / 1000);
}

float Motor::VC_TO_RPM(float _vc)
{
    return (VC * 1000) / (PI * weldingDiameter);
}

float Motor::M_PER_MIN_to_MM_SEC(float _vc)
{
    return _vc * 16.6666;
}

// CONVERT [MM/SEC] TO [M/MIN]
float Motor::MM_PER_SEC_to_M_PER_MIN(float _vc)
{
    return (_vc * 0.0166);
}

void Motor::motorStart(bool dir)
{
    this->MOTOR_CCW_DIR = dir;
    this->motorStartOn = true;
}
void Motor::motorStartFunc()
{
    if (this->motorStartOn)
    {
        if (*Motor_Status == MT_STOP)
        {
            *Motor_Status = MT_PAUSE;
        }

        if (*Motor_Status == MT_PAUSE && (bool)pauseBeforeTIME)
        {
            if (!pauseBeforeOn)
            {
                timeStart = millis();
                pauseBeforeOn = true;
            }
            else
            {
                if ((millis() - timeStart) < pauseBeforeTIME)
                {
                    timeLeft = (timeStart + pauseBeforeTIME) - millis();
                    delay(50);
                }
                else
                {
                    timeLeft = 0;
                    pauseBeforeOn = false;
                    *Motor_Status = MT_SOFT_START;
                }
            }
        }
        else
        {
            *Motor_Status = MT_SOFT_START;
        }

        if (*Motor_Status == MT_SOFT_START && (bool)softStartTIME)
        {
            if (!softStartOn)
            {
                timeStart = millis();
                softStartOn = true;
            }
            if (softStartOn)
            {
                if (millis() - timeStart <= softStartTIME)
                {
                    if ((bool)PWM_10BitLst)
                    {
                        OutputPWM_Signal(map((millis() - timeStart), 0, ((PWM_10BitLst * softStartTIME) / PWM_10BitSET), PWM_10BitLst, PWM_10BitSET));
                    }
                    else
                    {
                        OutputPWM_Signal(map((millis() - timeStart), 0, softStartTIME, 0, PWM_10BitSET));
                    }
                    relayStart();
                }
                else
                {
                    if (MOTOR_CCW_DIR)
                    {
                        *Motor_Status = MT_RUN_CCW;
                    }
                    else
                    {
                        *Motor_Status = MT_RUN_CW;
                    }

                    softStartOn = false;
                }
            }
        }
        else
        {
            if (MOTOR_CCW_DIR)
            {
                *Motor_Status = MT_RUN_CCW;
            }
            else
            {
                *Motor_Status = MT_RUN_CW;
            }
        }

        if (*Motor_Status == MT_RUN_CW || *Motor_Status == MT_RUN_CCW)
        {
            OutputPWM_Signal(PWM_10BitSET);
            relayStart();
            motorStartOn = false;
        }
    }
}
void Motor::motorRunFunc()
{
    if (*Motor_Status == MT_RUN_CW || *Motor_Status == MT_RUN_CCW) // motor runing
    {
        if (!motorRunOn)
        {
            timeStart = millis();
            motorRunOn = true;
        }
        if (motorRunOn)
        {
            if (PWM_10BitOUT != PWM_10BitSET)
            {
                OutputPWM_Signal(PWM_10BitSET);
                // OutputPWM_Signal();
            }
        }
    }
}
void Motor::motorGoStopFunc()
{
    if (*Motor_Status == MT_SMOOTH_STOP) // fade down
    {
        if (!smoothStopOn && (bool)smoothStopTIME)
        {
            timeStart = millis();
            smoothStopOn = true;
            PWM_10BitLst = PWM_10BitOUT;
        }
        if (smoothStopOn)
        {
            if (millis() - timeStart <= smoothStopTIME)
            {
                if (PWM_10BitLst == PWM_10BitSET)
                {
                    OutputPWM_Signal(map((millis() - timeStart), 0, smoothStopTIME, PWM_10BitLst, 0));
                }
                else
                {
                    OutputPWM_Signal(map((millis() - timeStart), 0, ((PWM_10BitLst * smoothStopTIME) / PWM_10BitSET), PWM_10BitLst, 0));
                }
            }
            else
            {
                *Motor_Status = MT_TURN_IT_OFF;
                smoothStopOn = false;
            }
        }
        else
        {
            *Motor_Status = MT_TURN_IT_OFF;
        }
    }

    if (*Motor_Status == MT_TURN_IT_OFF)
    {
        StopMotion();
        *Motor_Status = MT_STOP;
    }
}
void Motor::motorManage()
{
    this->motorStartFunc();
    this->motorRunFunc();
    this->motorGoStopFunc();
}

void Motor::OutputPWM_Signal(uint16_t _pwm)
{
    if (PWM_10BitOUT != _pwm)
    {
        this->PWM_10BitOUT = _pwm;
        _fastPWMdac.analogWrite10bit(this->PWM_10BitOUT);
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
        PWM_10BitSET = _pwm;
    }
}
void Motor::motorStop()
{
    if (*Motor_Status == MT_RUN_CW || *Motor_Status == MT_RUN_CCW)
    {
        *Motor_Status = MT_SMOOTH_STOP;
    }
}

void Motor::StopMotion()
{
    this->PWM_10BitOUT = 0;
    relayStop();
}

void Motor::relayStart()
{
    if (!motorRelayOn)
    {
        if (MOTOR_CCW_DIR)
        {
            digitalWrite(rl_cw, HIGH);
        }
        else
        {
            digitalWrite(rl_ccw, HIGH);
        }
        motorRelayOn = true;
    }
}

void Motor::relayStop()
{
    if (motorRelayOn)
    {
        if (MOTOR_CCW_DIR)
        {
            digitalWrite(rl_cw, LOW);
        }
        else
        {
            digitalWrite(rl_ccw, LOW);
        }
        motorRelayOn = false;
    }
}
