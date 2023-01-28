#include <Arduino.h>
#include <U8x8lib.h>
#include "config.h"

U8X8_SSD1306_128X64_NONAME_4W_SW_SPI u8x8(OLED_CLK, OLED_DATA, OLED_CS, OLED_DC, OLED_RST);

class Display
{
private:
    char str_ring_buf[OLED_CHAR_HEIGHT][OLED_CHAR_WIDTH+1] = {0};
    unsigned int last_msg_idx = 0;
public:
    void init();
    void beep(const unsigned int time_ms = 100,
              const unsigned int count = 1,
              const unsigned int interval = 100);
    void dbg_beep(const unsigned int time_ms= 100,
                  const unsigned int count = 1,
                  const unsigned int interval = 100);
    void print(const char *str);
    void dbg_print(const char *str);
    void panic(const char *str);
    void clear();
};

Display display;

void
Display::init()
{
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    u8x8.begin();
    u8x8.setFont(u8x8_font_pxplusibmcga_f);

    beep(30, 3);
    print("hello");
}

void
Display::beep(const unsigned int time_ms,
              const unsigned int count,
              const unsigned int interval)
{
    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(time_ms);
        digitalWrite(BUZZER_PIN, LOW);

        unsigned long tmout = millis() + interval;
        while (millis() < tmout){}
    }
}

void
Display::dbg_beep(const unsigned int time_ms,
              const unsigned int count,
              const unsigned int interval)
{
#ifdef DEBUG
    beep(time_ms,
         count,
         interval);
#endif
}

void
Display::print(const char *str)
{
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
Display::dbg_print(const char *str)
{
#ifdef DEBUG
    print(str);
    beep(5,1,0);
#endif
}

void
Display::panic(const char *str)
{
    print(str);
    while (true) {
        beep();
    }
}

void
Display::clear()
{
    last_msg_idx = 0;
    for (unsigned int i=0; i<OLED_CHAR_HEIGHT; i++) {
        str_ring_buf[i][0] = '\0';
    }
    u8x8.clear();
}
