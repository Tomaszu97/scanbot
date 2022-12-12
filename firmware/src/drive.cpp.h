#include <Arduino.h>
#include <Servo.h>
#include "config.h"
#include "util.cpp.h"

#define MIN_SPEED -90
#define MAX_SPEED 90
#define MIN_TURN -60
#define MAX_TURN 60

class Drive
{
private:
    Servo left_servo;
    Servo right_servo;
    int left_encoder_counter;
    int right_encoder_counter;
    void set_motors(int speed_left, int speed_right);
    int speed;
    int turn;

public:
    void init();
    void attach();
    void detach();
    void set_speed(const int speed);
    void set_turn(const int val);
    void update_motors();
    int get_left_encoder_counter();
    int get_right_encoder_counter();
    void inc_left_encoder_counter();
    void inc_right_encoder_counter();
    void reset_encoder_counters();
};

Drive drive;

void
left_encoder_ISR()
{
    drive.inc_left_encoder_counter();
    // RC filter only on pinA, thus not detecting direction here
    // if (digitalRead(LEFT_ENCODER_PIN_B) == 0)
    //     left_encoder_counter++;
    // else
    //     left_encoder_counter--;
}

void
right_encoder_ISR()
{
    drive.inc_right_encoder_counter();
    // RC filter only on pinA, thus not detecting direction here
    // if (digitalRead(RIGHT_ENCODER_PIN_B) == 0)
    //     right_encoder_counter++;
    // else
    //     right_encoder_counter--;
}


void
Drive::init()
{
    reset_encoder_counters();
    speed = 0;
    turn = 0;

    pinMode(LEFT_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(LEFT_ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A), left_encoder_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_A), right_encoder_ISR, FALLING);
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
    int deadzone = DRIVE_SERVO_DEADZONE;
    int in[] = {-90, 0, 90};
    int left_out[] = {180, DRIVE_SERVO_MIDDLE, 0};
    int right_out[] = {0, DRIVE_SERVO_MIDDLE, 180};

    if (speed_left >= -deadzone && speed_left <= deadzone) left_servo.detach();
    else {
        int new_speed_left = multi_map(speed_left, in, left_out, ARRAY_SIZE(in));

        if (left_servo.attached() == false)
            left_servo.attach(LEFT_SERVO_PIN, DRIVE_SERVO_US_MIN, DRIVE_SERVO_US_MAX);

        left_servo.write(new_speed_left);
    }
    if (speed_right >= -deadzone && speed_right <= deadzone) right_servo.detach();
    else {
        int new_speed_right = multi_map(speed_right, in, right_out, ARRAY_SIZE(in));

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
