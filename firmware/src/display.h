#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <U8x8lib.h>
#include "config.h"

class Display
{
private:
    Display();
    static Display *display_;
    char str_ring_buf[OLED_CHAR_HEIGHT][OLED_CHAR_WIDTH + 1] = { 0 };
    unsigned int last_msg_idx = 0;
    bool initialized = false;

public:
    static Display *get_instance();
    void beep(const unsigned int time_ms = 100,
              const unsigned int count = 1,
              const unsigned int interval = 100,
              const bool debug = false);
    void print(const char *str,
               const bool debug = false);
    void panic(const char *str);
    void clear();
};

#endif /* DISPLAY_H */
