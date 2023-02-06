#include <Arduino.h>
#include <U8x8lib.h>
#include "config.h"
#include "display.h"

U8X8_SSD1306_128X64_NONAME_4W_SW_SPI u8x8(OLED_CLK, OLED_DATA, OLED_CS, OLED_DC, OLED_RST);

static bool
is_debug_on()
{
#ifdef DEBUG
    return true;
#else
    return false;
#endif
}

Display *Display::display_;

Display *
Display::get_instance()
{
    if (Display::display_ == nullptr)
        Display::display_ = new Display();
    return Display::display_;
}

Display::Display()
{
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    u8x8.begin();
    u8x8.setFont(u8x8_font_pxplusibmcga_f);
    initialized = true;

    print("display init ok");
}

void
Display::beep(const unsigned int time_ms,
              const unsigned int count,
              const unsigned int interval,
              const bool debug)
{
    if (initialized == false) return;

    const bool ignore = (debug == true && is_debug_on() == false);
    if (ignore == true) return;

    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(time_ms);
        digitalWrite(BUZZER_PIN, LOW);

        unsigned long tmout = millis() + interval;
        while (millis() < tmout) {}
    }
}

void
Display::print(const char *str,
               const bool debug)
{
    if (initialized == false) return;

    const bool ignore = (debug == true && is_debug_on() == false);
    if (ignore == true) return;

    /* update  ring buffer */
    strncpy(str_ring_buf[last_msg_idx], str, OLED_CHAR_WIDTH);
    str_ring_buf[last_msg_idx][OLED_CHAR_WIDTH] = '\0';
    last_msg_idx = (last_msg_idx+1) % OLED_CHAR_HEIGHT;

    /* print ring buffer */
    for (unsigned int i=0; i<OLED_CHAR_HEIGHT; i++) {
        unsigned int row = (OLED_CHAR_HEIGHT-1) - i;
        u8x8.clearLine(row);
        u8x8.setCursor(0, row);
        u8x8.print(str_ring_buf[(i+last_msg_idx) % OLED_CHAR_HEIGHT]);
    }
}

void
Display::panic(const char *str)
{
    if (initialized == false) while(true) {};

    print(str);
    while (true) {
        beep();
    }
}

void
Display::clear()
{
    if (initialized == false) return;

    last_msg_idx = 0;
    for (unsigned int i=0; i<OLED_CHAR_HEIGHT; i++) {
        str_ring_buf[i][0] = '\0';
    }
    u8x8.clear();
}
