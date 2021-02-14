#include "BigFont.h"

BigFont::BigFont()
{
}

void BigFont::BindLcd(LiquidCrystal_I2C *lcd)
{
    this->lcd = lcd;
    for (nb = 0; nb < 8; nb++)
    {
        for (bc = 0; bc < 8; bc++)
        {
            bb[bc] = pgm_read_byte(&custom[nb][bc]);
            this->lcd->createChar(nb + 1, bb);
        }
    }
}

int BigFont::WriteBigChar(char ch, byte x, byte y)
{
    if (ch < ' ' || ch > '_')
    {
        return 0;
    }
    nb = 0;
    for (bc = 0; bc < 8; bc++)
    {
        bb[bc] = pgm_read_byte(&bigChars[ch - ' '][bc]);
        if (bb[bc] != 0)
        {
            nb++;
        }
    }

    bc = 0;
    for (row = y; row < y + 2; row++)
    {
        for (col = x; col < x + nb / 2; col++)
        {
            this->lcd->setCursor(col, row); // move to position
            this->lcd->write(bb[bc++]);     // write byte and increment to next
        }
    }
    return nb / 2 - 1; // returns number of columns used by char
}

void BigFont::WriteBigString(char const *str, byte x, byte y)
{
    char c;
    while ((c = *str++))
        x += WriteBigChar(c, x, y) + 1;
    // return 0;
}