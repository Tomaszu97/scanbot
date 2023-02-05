#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>
#include "config.h"
#include "util.h"
#include "display.h"

class Watchdog
{
private:
    Watchdog();
    static Watchdog *watchdog_;
    Display *display;
    unsigned long last_millis;

public:
    static Watchdog *get_instance();
    void ping();
    bool is_active();
};

#endif /* WATCHDOG_H */
