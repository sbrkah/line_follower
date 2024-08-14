#include "menu_functions.h"

void printMenu(){
  lcdPrint(_titles[_index], 0, 1);
}

void setupMenu(const char* titles[], const int targets[], int size){
    _titles = titles;
    _targets = targets;
    _index = 0;
    _size = size;
    printMenu();
}

void nextMenu(){
    _index = (_index + 1) % _size;
    printMenu();
}

int currentTarget(){
    return _targets[_index];
}

int indexMenu(){
    return _index;
}
