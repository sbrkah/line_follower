#include "lcd_functions.h"

LiquidCrystal_I2C _lcd(0x27, 16, 2); 
bool _is_initiated = false;

void lcdSetup(uint8_t address, uint8_t length, uint8_t height) {
  _lcd = LiquidCrystal_I2C(address, length, height);
  _lcd.init();
  _lcd.backlight();
  _is_initiated = true;
}

void lcdPrint(String content, uint8_t cursor_x, uint8_t cursor_y, uint8_t desiredLength) {
  if (!_is_initiated) {
    return;
  }
  if (desiredLength && desiredLength > content.length()) {
    for (uint8_t i = 0; i < desiredLength - content.length(); i++) {
      content = " " + content;
    }
  }
  if (cursor_x > -1 && cursor_y > -1) {
    _lcd.setCursor(cursor_x, cursor_y);
  }
  _lcd.print(content);
}
