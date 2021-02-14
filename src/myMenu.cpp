#include "myMenu.h"

myMenu::myMenu(LiquidCrystal_I2C *lcd, Parametr *parameter_p, byte *BT_ST, uint8_t *AppMode)
{
    this->lcd = lcd;
    this->BT_ST = BT_ST;
    this->AppMode = AppMode;
    this->parameter_p = parameter_p;
    // this->enu_p = enu_p;
    this->cursor = 0;
    this->scroll = 0;
    this->menuIsOn = false;
    this->scrollMenu = true;
    this->menuLevel = 0;
    this->curMenu = 0;
}
void myMenu::MoveUp()
{
    if (this->cursor > 0 || this->scroll > 0)
    {
        if (this->cursor == 0 && this->scroll > 0)
        {
            this->scroll--;
            DrawMenu();
        }
        else if (this->cursor > 0)
        {
            this->cursor--;
            UpdateCursor();
        }
    }
}

void myMenu::MoveDown()
{
    if ((this->cursor + this->scroll) < (this->child - 1))
    {
        if (this->cursor == 3)
        {
            this->scroll++;
            DrawMenu();
        }
        else
        {
            this->cursor++;
            UpdateCursor();
        }
    }
}

void myMenu::GetMenu(uint8_t menuNumber)
{
    this->lastCursor[this->menuLevel] = 0; //this->cursor;
    this->lastMenuNb[this->menuLevel] = this->curMenu;
    this->lastScroll[this->menuLevel] = this->scroll;
    this->curMenu = menuNumber;

    this->child = pgm_read_byte(&(menuSc[menuNumber][1]));
    this->menuOffset = pgm_read_byte(&(menuSc[menuNumber][0]));
    this->menuLevel = MENU_DATA_LVL(this->menuOffset);
    this->scroll = 0;
    this->cursor = 0;
};

void myMenu::DrawMenu()
{
    lcd->clear();

    uint8_t n = 0;
    for (;;)
    {
        if (cursor == n)
        {
            lcd->setCursor(0, n);
            lcd->write(0x7E);
        }

        this->PrintLine(n);

        n++;
        if (n == this->child || n == 4)
        {
            break;
        }
    }
}

void myMenu::PrintLine(uint8_t line)
{
    char *buf = (char *)malloc(sizeof(char) * 20);

    switch (MENU_DATA_TYP(MENU_NUMBER(line)))
    {
        /*******************************
             *  MENU ITEM                  *
             * ****************************/
    case 0:
        GET_TEXT_ID(buf, MENU_DATA_LABEL(MENU_NUMBER(line)));
        // DBG(buf);
        // DBG("case 0 strlen: ", strlen(buf));
        break;

        /*******************************
             *  VALUE                      *
             * ****************************/
    case 1:
        GetValueLine(buf, MENU_DATA_LABEL(MENU_NUMBER(line)), MENU_DATA_CALL(MENU_NUMBER(line)));
        // DBG("case 1: ", buf);
        break;

        /*******************************
             *  ENUM                       *
             * ****************************/
    case 2:
        GetEnumLine(buf, MENU_DATA_LABEL(MENU_NUMBER(line)), MENU_DATA_CALL(MENU_NUMBER(line)));
        break;

        /*******************************
             *  CALLBACK                   *
             * ****************************/
    case 3:
        GetCallBackLine(buf, MENU_DATA_LABEL(MENU_NUMBER(line)));
        break;

        /*******************************
             *  ENUM VAL                   *
             * ****************************/
    case 4:
        GetEnumValLine(buf, MENU_DATA_LABEL(MENU_NUMBER(line)), MENU_DATA_CALL(MENU_NUMBER(line)));
        break;

    default:
        // DBG("defout ", typ);
        break;
    }
    lcd->setCursor(1, line);
    lcd->print(buf);
    free(buf);
}

/*******************************************************************
 * CASE 0
 * - Pobranie tylko tekstu z numeru ID
 * 
 ******************************************************************/
void myMenu::GetMenuItemLine(char *bufOut, uint8_t _id)
{
    DBG("GetMenuItemLine START");
    GET_TEXT_ID(bufOut, _id);
    DBG("GetMenuItemLine END");
}

/*******************************************************************
 * CASE 1
 * label    1000unit
 * label    10.0unit
 * - POBIERZ LABEL (TEXT LEFT)
 * - POBIERZ WARTOŚĆ ZMIENNEJ -> FORMAT + DODANIE UNIT
 ******************************************************************/
void myMenu::GetValueLine(char *bufOut, uint8_t _id, uint8_t _call)
{
    DBG("GetValueLine START");
    char *leftTxt = (char *)malloc(sizeof(char) * 15);
    char *unit = (char *)malloc(sizeof(char) * 8);
    char *sBuf = (char *)malloc(sizeof(char) * 10);
    char *value = (char *)malloc(sizeof(char) * 11);

    // SET_VALUE_P();
    // this->par_p = ;
    // uint32_t val = (*par_p).val;
    // uint16_t offset = ;
    // uint32_t maxVal;
    // uint16_t inc;
    // uint16_t precision = ;
    // uint8_t unitNum = ;0

    GET_TEXT_ID(leftTxt, _id);
    GET_TEXT_ID(unit, (*(this->parameter_p + _call)).unitNum);

    if ((bool)(*(this->parameter_p + _call)).precision)
    {
        // DBG("PRINT FLOAT");
        dtostrf((((*(this->parameter_p + _call)).val + (*(this->parameter_p + _call)).offset) / pow(10, (*(this->parameter_p + _call)).precision)), 1, (*(this->parameter_p + _call)).precision, value);
    }
    else
    {
        // DBG("PRINT INT");
        sprintf(value, "%d", int((*(this->parameter_p + _call)).val + (*(this->parameter_p + _call)).offset));
    }
    sprintf(sBuf, "%%s%%%ds%%s", 19 - strlen(leftTxt) - strlen(unit));
    sprintf(bufOut, sBuf, leftTxt, value, unit);
    // DBG("rBuf: ", rBuf);
    // DBG("rBuf len: ", strlen(rBuf));
    free(value);
    free(sBuf);
    free(leftTxt);
    free(unit);
    DBG("GetValueLine END");
}

/*******************************************************************
 * CASE 2
 * get label and enum data
 * - get left label;
 * - call wskazuje ma wartosc enum. dodajemy ją do numeru id lebel aby wyłuskać weykiete dla enum
 * - wartość 
 ******************************************************************/
void myMenu::GetEnumLine(char *bufOut, uint8_t id, uint8_t call)
{
    DBG("GetEnumLine START");
    DBG(id);
    DBG(call);
    char *leftTxt = (char *)malloc(sizeof(char) * 15);
    char *rightTxt = (char *)malloc(sizeof(char) * 8);
    char *sBuf = (char *)malloc(sizeof(char) * 10);
    // par_p = parameter_p + call;
    // uint32_t val = (*par_p).val;
    // uint8_t unitNum = (*par_p).unitNum;

    GET_TEXT_ID(leftTxt, id);
    GET_TEXT_ID(rightTxt, ((*(this->parameter_p + call)).unitNum + (*(this->parameter_p + call)).val));

    sprintf(sBuf, "%%s%%%ds", 19 - strlen(leftTxt));
    sprintf(bufOut, sBuf, leftTxt, rightTxt);
    free(leftTxt);
    free(rightTxt);
    free(sBuf);
    DBG("GetEnumLine END");
}

/*******************************************************************
 * CASE 3
 * - pobieramy text dla etykiety.
 * 
 ******************************************************************/
void myMenu::GetCallBackLine(char *bufOut, uint8_t _id)
{
    DBG("GetCallBackLine START");
    GET_TEXT_ID(bufOut, _id);
    DBG("GetCallBackLine END");
}

/*******************************************************************
 * CASE 4
 * - pobieramy tekst etykiety dla wskazywanej przez _id
 * - wartość call wskazuje na enum,
 * - nowy call parametru bierzemy z id+enumVAR
 * - wywołujemy GetValueLine(_id, call) z 
 * 
 ***********************    a*******************************************/
void myMenu::GetEnumValLine(char *bufOut, uint8_t _id, uint8_t call)
{
    DBG("GetEnumValLine START");
    // par_p = parameter_p + call;
    // uint32_t val = (*par_p).val;
    // SET_ENUM_P(call);
    GetValueLine(bufOut, _id, (*(this->parameter_p + call)).val);
    DBG("GetEnumValLine END");
}

void myMenu::MenuStart()
{
    if (!this->menuIsOn)
    {
        this->menuIsOn = true;
        this->curMenu = 0;
        this->cursor = 0;
        this->scroll = 0;
    }
}

/*************************************************
 *  check specyfic button
 * **********************************************/
bool myMenu::ChceckBtn(uint8_t btn)
{
    if (bitRead(*this->BT_ST, btn))
    {
        bitWrite(*this->BT_ST, btn, LOW);
        return true;
    }
    else
    {
        return false;
    }
}

/*****************************************************************
 *  clear the specyfic button state after action runinng
 * ***************************************************************/
void myMenu::ClearButton(uint8_t btn)
{
    bitWrite(*this->BT_ST, btn, LOW);
}
void myMenu::ClearAllBtn()
{
    *this->BT_ST = 0x00000000;
}
bool myMenu::CheckAnyBtn()
{
    if (*this->BT_ST > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void myMenu::UpdateCursor()
{
    if (this->lastCursor[this->menuLevel] != this->cursor)
    {
        lcd->setCursor(0, this->cursor);
        lcd->write(0x7E);
        lcd->setCursor(0, this->lastCursor[this->menuLevel]);
        lcd->print(' ');
        this->lastCursor[this->menuLevel] = this->cursor;
    }
    else
    {
        if (this->scrollMenu)
        {
            lcd->setCursor(0, this->cursor);
            lcd->write(0x7E);
        }
        else
        {
            lcd->setCursor(0, this->cursor);
            lcd->write(0xAB);
        }
    }
}

void myMenu::UpdateMenu()
{
    if (CheckAnyBtn())
    {

        /*************************
         *  PRZYCISK: UP
         * **********************/
        if (ChceckBtn(BTN_UP))
        {
            if (this->scrollMenu)
            {
                MoveDown();
            }
            else
            {
                DecData();
            }
        }

        /*************************
         *  PRZYCISK: DOWN
         * **********************/
        if (ChceckBtn(BTN_DOWN))
        {
            if (this->scrollMenu)
            {
                MoveUp();
            }
            else
            {
                IncData();
            }
        }

        /*************************
         *  PRZYCISK: LEFT
         * **********************/
        if (ChceckBtn(BTN_LEFT))
        {
            if (this->scrollMenu)
            {
                MoveUp();
            }
            else
            {
                IncData();
            }
        }

        /*************************
         *  PRZYCISK: RIGHT
         * **********************/
        if (ChceckBtn(BTN_RIGHT))
        {
            if (this->scrollMenu)
            {
                MoveDown();
            }
            else
            {
                DecData();
            }
        }

        /*************************
         *  PRZYCISK: CLICK
         * **********************/
        if (ChceckBtn(BTN_SW_SHORT))
        {
            uint8_t menuType = MENU_DATA_TYP(MENU_NUMBER(this->cursor));

            if (menuType == 4 || menuType == 1 || menuType == 2)
            {
                this->scrollMenu = !this->scrollMenu;
                UpdateCursor();
            }

            if (menuType == 0)
            {
                GetMenu(MENU_DATA_CALL(MENU_NUMBER(this->cursor)));
                DrawMenu();
                UpdateCursor();
            }
            if (menuType == 3)
            {
                *AppMode = APP_IDLE_VIEW;
            }
        }

        /*************************
         *  PRZYCISK: LONG CLICK
         * **********************/
        if (ChceckBtn(BTN_SW_LONG))
        {
            /* code */
        }
    }
}

void myMenu::IncData()
{
    DBG(MENU_DATA_TYP(MENU_NUMBER(this->cursor)));
    int pointer;
    switch (MENU_DATA_TYP(MENU_NUMBER(this->cursor)))
    {
        /*******************************
        *  VALUE                      *
        * ****************************/
    case 1:
        // DBG("CASE 1");
        pointer = MENU_DATA_CALL(MENU_NUMBER(this->cursor));
        break;

        /*******************************
             *  ENUM                       *
             * ****************************/
    case 2:
        // DBG("CASE 2");
        pointer = MENU_DATA_CALL(MENU_NUMBER(this->cursor));
        // SET_VALUE_P(MENU_DATA_CALL(MENU_NUMBER(this->cursor)));
        break;

        /*******************************
             *  ENUM VAL                   *
             * ****************************/
    case 4:
        // DBG("CASE 3");
        pointer = (*(this->parameter_p + MENU_DATA_CALL(MENU_NUMBER(this->cursor)))).val;
        // SET_VALUE_P();
        // DBG((*par_p).val);
        // SET_VALUE_P((*par_p).val);
        break;

    default:
        // DBG("defout ", typ);
        break;
    }

    /*****************************************/
    // *(this->parameter_p + _call).
    // DBG(par_p->val);
    // DBG(par_p->inc);
    // DBG(par_p->maxVal);
    // DBG(this->cursor);
    // DBG(pointer);

    // DBG((*(this->parameter_p + pointer)).val + (*(this->parameter_p + pointer)).inc);
    (*(this->parameter_p + pointer)).val = (*(this->parameter_p + pointer)).val + (*(this->parameter_p + pointer)).inc;
    if ((*(this->parameter_p + pointer)).val > int((*(this->parameter_p + pointer)).maxVal))
    {
        (*(this->parameter_p + pointer)).val = int((*(this->parameter_p + pointer)).maxVal);
    }
    PrintLine(this->cursor);
    // DBG((*(this->parameter_p + pointer)).precision);
    // DBG((*(this->parameter_p + pointer)).val);
    // DBG((*(this->parameter_p + pointer)).val);
    // DBG((*(this->parameter_p + pointer)).inc);
    // DBG((*(this->parameter_p + pointer)).maxVal);
    // DBG(this->cursor);
}
void myMenu::DecData()
{
    int pointer;
    switch (MENU_DATA_TYP(MENU_NUMBER(this->cursor)))
    {
        /*******************************
        *  VALUE                      *
        * ****************************/
    case 1:
        // DBG("CASE 1");
        pointer = MENU_DATA_CALL(MENU_NUMBER(this->cursor));
        break;

        /*******************************
             *  ENUM                       *
             * ****************************/
    case 2:
        // DBG("CASE 2");
        pointer = MENU_DATA_CALL(MENU_NUMBER(this->cursor));
        // SET_VALUE_P(MENU_DATA_CALL(MENU_NUMBER(this->cursor)));
        break;

        /*******************************
             *  ENUM VAL                   *
             * ****************************/
    case 4:
        // DBG("CASE 3");
        pointer = (*(this->parameter_p + MENU_DATA_CALL(MENU_NUMBER(this->cursor)))).val;
        // SET_VALUE_P();
        // DBG((*par_p).val);
        // SET_VALUE_P((*par_p).val);
        break;

    default:
        // DBG("defout ", typ);
        break;
    }

    /*****************************************/
    // DBG(pointer);
    (*(this->parameter_p + pointer)).val = (*(this->parameter_p + pointer)).val - (*(this->parameter_p + pointer)).inc;
    if ((*(this->parameter_p + pointer)).val < 0)
    {
        // DBG("MIN VAL");
        (*(this->parameter_p + pointer)).val = 0;
    }
    PrintLine(this->cursor);
    //     DBG((*(this->parameter_p + pointer)).val);
    //     DBG((*(this->parameter_p + pointer)).inc);
    //     DBG((*(this->parameter_p + pointer)).maxVal);
    //     DBG(this->cursor);
}