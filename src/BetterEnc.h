#ifndef _BETTER_ENC_H_
#define _BETTER_ENC_H_

#include "Arduino.h"

class BetterEnc
{
private:
    volatile uint8_t master_count; // universal count
    uint8_t STEP;
    uint8_t DIR;
    void read_rotary(){};

public:
    BetterEnc(uint8_t pinA, uint8_t pinB);
    uint8_t read();
    void write(uint8_t addVal);
    void setupInterruptHandler(uint8_t irq_pin, void (*ISR)(void), int value),
        handleInterrupt(void);
};

#endif
