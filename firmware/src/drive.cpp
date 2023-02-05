#include <Arduino.h>
#include <Servo.h>
#include "config.h"
#include "util.h"
#include "display.h"
#include "drive.h"

void
left_encoder_ISR()
{
    Drive *d = Drive::get_instance();
    d->inc_left_encoder_counter();
}

void
right_encoder_ISR()
{
    Drive *d = Drive::get_instance();
    d->inc_right_encoder_counter();
}

Drive *Drive::drive_;

Drive *
Drive::get_instance()
{
    if (Drive::drive_ == nullptr)
        Drive::drive_ = new Drive();
    return Drive::drive_;
}

Drive::Drive()
{
    display = Display::get_instance();

    reset_encoder_counters();
    speed = 0;
    turn = 0;

    pinMode(LEFT_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(LEFT_ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A), left_encoder_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_A), right_encoder_ISR, FALLING);

    display->print("drive init ok");
}


int
Drive::get_left_encoder_counter()
{
    return left_encoder_counter;
}

int
Drive::get_right_encoder_counter()
{
    return right_encoder_counter;
}

void
Drive::inc_left_encoder_counter()
{
    left_encoder_counter++;
}

void
Drive::inc_right_encoder_counter()
{
    right_encoder_counter++;
}

void
Drive::reset_encoder_counters()
{
    left_encoder_counter = 0;
    right_encoder_counter = 0;
}

void
Drive::attach()
{
    left_servo.attach(LEFT_SERVO_PIN, DRIVE_SERVO_US_MIN, DRIVE_SERVO_US_MAX);
    right_servo.attach(RIGHT_SERVO_PIN, DRIVE_SERVO_US_MIN, DRIVE_SERVO_US_MAX);
}

void
Drive::detach()
{
    left_servo.detach();
    right_servo.detach();
}

void
Drive::set_motors(int speed_left,
                  int speed_right)
{
    int deadzone = DRIVE_SERVO_IN_DEADZONE;

    int left_in[] = DRIVE_SERVO_CURVE_LEFT_IN;
    int right_in[] = DRIVE_SERVO_CURVE_RIGHT_IN;

    int left_out[] = DRIVE_SERVO_CURVE_LEFT_OUT;
    int right_out[] = DRIVE_SERVO_CURVE_RIGHT_OUT;

    if (speed_left >= -deadzone && speed_left <= deadzone) left_servo.detach();
    else {
        int new_speed_left = multi_map(speed_left, left_in, left_out, ARRAY_SIZE(left_in));

        if (left_servo.attached() == false)
            left_servo.attach(LEFT_SERVO_PIN, DRIVE_SERVO_US_MIN, DRIVE_SERVO_US_MAX);

        left_servo.write(new_speed_left);
    }
    if (speed_right >= -deadzone && speed_right <= deadzone) right_servo.detach();
    else {
        int new_speed_right = multi_map(speed_right, right_in, right_out, ARRAY_SIZE(right_in));

        if (right_servo.attached() == false)
            right_servo.attach(RIGHT_SERVO_PIN, DRIVE_SERVO_US_MIN, DRIVE_SERVO_US_MAX);

        right_servo.write(new_speed_right);
    }
}

void
Drive::set_speed(const int set_speed)
{
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    else if (speed < MIN_SPEED) speed = MIN_SPEED;
    speed = set_speed;
}

void
Drive::set_turn(const int set_turn)
{
    if (turn > MAX_TURN) turn = MAX_TURN;
    else if (turn < MIN_TURN) turn = MIN_TURN;
    turn = set_turn;
}

void
Drive::update_motors()
{
    int speed_left = speed + turn;
    int speed_right = speed - turn;

    int corrected_speed = speed;
    if (speed_left > MAX_SPEED) corrected_speed = speed_left - MAX_SPEED;
    else if (speed_left < MIN_SPEED) corrected_speed = MIN_SPEED - speed_left;
    else if (speed_right > MAX_SPEED) corrected_speed = speed_right - MAX_SPEED;
    else if (speed_right < MIN_SPEED) corrected_speed = MAX_SPEED - speed_right;

    speed_left = corrected_speed + turn;
    speed_right = corrected_speed - turn;

    set_motors(speed+turn,
               speed-turn);
}
