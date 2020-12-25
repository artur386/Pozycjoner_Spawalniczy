#ifndef VARIABLE_H
#define VARIABLE_H

#include "Arduino.h"
struct parameterStruct
{
    char *name[];
    uint8_t Id;
    uint8_t minVal;
    uint16_t maxVal;
    uint16_t curVal;
    uint8_t incVal;
    uint8_t frac;
    void (*handler)();
};

extern struct parameterStruct parameter[10];
#endif