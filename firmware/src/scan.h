#ifndef SCAN_H
#define SCAN_H

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "config.h"
#include "util.h"
#include "display.h"
#include "command.h"

class Scan
{
private:
    Scan();
    static Scan *scan_;
    Display *display;
    Command *command;
    bool lidar_init();
    void lidar_trigger();
    void lidar_update(unsigned int curr_pos);
    void servo_init();
    unsigned int servo_next_pos(unsigned int curr_pos);
    bool throttle();
    void set_reg_u8(uint8_t reg, uint8_t val);
    void set_reg_u16(uint8_t reg_low, uint16_t val);
    uint8_t get_reg_u8(uint8_t reg);
    uint16_t get_reg_u16(uint8_t reg_low);
    Servo tower_servo;
    bool working = false;
    unsigned int pos;
    uint16_t scan_buf[SCAN_BUF_LEN];
    bool scan_buf_updated[SCAN_BUF_LEN];
    bool is_scan_full();
    void notify_scan_state();

public:
    static Scan *get_instance();
    void start();
    void stop();
    void work();
    void servo_attach();
    void servo_detach();
    void servo_set(unsigned int position);
};

#endif /* SCAN_H */
