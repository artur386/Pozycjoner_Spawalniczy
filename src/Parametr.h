

#ifndef PARAMETR_H
#define PARAMETR_H

struct Parametr
{
    char Nazwa[8];
    uint32_t val;
    uint16_t offset;
    uint32_t maxVal;
    uint16_t inc;
    uint16_t precision;
    // void (*void)()
};

#endif // !PARAMETR_H