#ifndef LCD_FUNCTION_H
#define LCD_FUNCTION_H

#include "Arduino.h"
#include "LiquidCrystal_I2C.h"

void lcdSetup(uint8_t address);
void lcdPrint(String content, uint8_t cursor_x, uint8_t cursor_y, uint8_t desiredLength = 0);

extern LiquidCrystal_I2C _lcd;
extern bool _is_initiated;

#endif
