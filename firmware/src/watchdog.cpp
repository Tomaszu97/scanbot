#include <Arduino.h>
#include "config.h"
#include "util.h"
#include "display.h"
#include "watchdog.h"

Watchdog *Watchdog::watchdog_;

Watchdog *
Watchdog::get_instance()
{
    if (Watchdog::watchdog_ == nullptr)
        Watchdog::watchdog_ = new Watchdog();
    return Watchdog::watchdog_;
}

Watchdog::Watchdog()
{
    display = Display::get_instance();
    last_millis = millis();
    display->print("watchdog init ok");
}

void
Watchdog::ping()
{
    last_millis = millis();
}

bool
Watchdog::is_active()
{
    const bool active = (millis() > last_millis + PING_TIMEOUT_MS);
    return active;
}
