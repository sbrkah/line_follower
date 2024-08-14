#ifndef MENU_FUNCTIONS_H
#define MENU_FUNCTIONS_H
#include "Arduino.h"
#include "LiquidCrystal_I2C.h"
#include "lcd_functions.h"

void setupMenu(const char* titles[], const int targets[], int size);
void nextMenu();
int currentTarget();
int indexMenu();
static const char** _titles = nullptr;
static const int* _targets = nullptr;
static int _index = 0;
static int _size = 0;

#endif
