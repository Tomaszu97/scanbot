#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <Servo.h>
#include "config.h"
#include "util.h"

class Drive
{
private:
    Drive();
    static Drive *drive_;
    Display *display;
    Servo left_servo;
    Servo right_servo;
    int left_encoder_counter;
    int right_encoder_counter;
    int speed;
    int turn;

public:
    static Drive *get_instance();
    void attach();
    void detach();
    void set_speed(const int speed);
    void set_turn(const int val);
    void set_motors(int speed_left, int speed_right);
    void update_motors();
    int get_left_encoder_counter();
    int get_right_encoder_counter();
    void inc_left_encoder_counter();
    void inc_right_encoder_counter();
    void reset_encoder_counters();
};

#endif /* DRIVE_H */
