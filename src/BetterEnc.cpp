#include "BetterEnc.h"

BetterEnc::BetterEnc(uint8_t pinA, uint8_t pinB)
{
    STEP = pinA;
    DIR = pinB;

    pinMode(STEP, INPUT);
    pinMode(DIR, INPUT);
    
    master_count = 0;
};

uint8_t BetterEnc::read()
{
    return master_count;
};
void BetterEnc::write(uint8_t addVal)
{
    noInterrupts();
    master_count += addVal;
    interrupts();
};

void BetterEnc::setupInterruptHandler(uint8_t irq_pin, void (*ISR)(void), int value)
{
    attachInterrupt(digitalPinToInterrupt(irq_pin), ISR, value);
}

