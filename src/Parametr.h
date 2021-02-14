

#ifndef PARAMETR_H
#define PARAMETR_H

struct Parametr
{
    uint8_t id;
    char Nazwa[6];
    int val;
    uint8_t offset;
    uint32_t maxVal;
    uint8_t inc;
    uint8_t precision;
    uint8_t unitNum;
    uint8_t type;
};

#endif // !PARAMETR_H