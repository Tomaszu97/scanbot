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
    void tower_step();
    bool throttle();
    void set_reg_u8(uint8_t reg, uint8_t val);
    void set_reg_u16(uint8_t reg_low, uint16_t val);
    uint8_t get_reg_u8(uint8_t reg);
    uint16_t get_reg_u16(uint8_t reg_low);
    bool is_scan_full();
    void notify_scan_state();
    void clear();
    static Scan *scan_;
    Display *display;
    Command *command;
    enum scan_state state = SCAN_DISABLED;
    unsigned int pos;
    uint16_t scan_buf[SCAN_BUF_LEN];
    bool scan_buf_updated[SCAN_BUF_LEN];

public:
    static Scan *get_instance();
    void start();
    void stop();
    void pause();
    void unpause();
    void work();
    void servo_attach();
    void servo_detach();
    void servo_set(unsigned int position);
};

#endif /* SCAN_H */
