#ifndef SCAN_H
#define SCAN_H

#include <Arduino.h>
#include "Servo/src/Servo.h"
#include <Wire.h>
#include "config.h"
#include "util.h"
#include "display.h"
#include "command.h"

enum scan_state {
    SCAN_WORKING,
    SCAN_PAUSED,
    SCAN_DISABLED
};

class Scan
{
private:
    Scan();
    bool lidar_init();
    void lidar_trigger();
    void lidar_update(unsigned int curr_pos);
    void tower_init();
    void tower_enable();
    void tower_disable();
    void tower_step();
    bool throttle();
    void set_reg_u8(uint8_t reg, uint8_t val);
    void set_reg_u16(uint8_t reg_low, uint16_t val);
    uint8_t get_reg_u8(uint8_t reg);
    uint16_t get_reg_u16(uint8_t reg_low);
    bool is_scan_full();
    void notify_scan_state();
    void clear();
    void adjust_pos();
    void process_scan();
    static Scan *scan_;
    Display *display;
    Command *command;
    enum scan_state state = SCAN_DISABLED;
    unsigned int pos = 0; /* buffer cursor */
    unsigned int zero_pos = 0; /* zero degree position detected from scan */
    int first_scan_pos = 0; /* position where full rotation scan started */
    uint16_t scan_buf[SCAN_STEPS_PER_ROTATION];

public:
    static Scan *get_instance();
    void start();
    void stop();
    void pause();
    void unpause();
    void work();
};

#endif /* SCAN_H */
