#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "config.h"
#include "util.cpp.h"

TwoWire Wire2 = TwoWire(SDA2, SCL2);

class Scan
{
private:
    bool working = false;
    Servo tower_servo;
    void attach();
    void detach();
    void set_tower();
    bool throttle();
    void step_tower();
    int pos;
    bool dir_inc;

public:
    void init();
    void start();
    void stop();
    void work();
    int16_t scan_buf[SCAN_BUF_LEN];
    bool scan_buf_updated[SCAN_BUF_LEN];
};

Scan scan;

void
Scan::init()
{
    Wire2.begin();
    memset(scan_buf, -1, sizeof(scan_buf));
    attach();
    pos = (SCAN_BUF_LEN/2);
    dir_inc = true;
    set_tower();
}

void
Scan::attach()
{
    if (tower_servo.attached() == false) {
        tower_servo.attach(TOWER_SERVO_PIN, TOWER_SERVO_US_MIN, TOWER_SERVO_US_MAX);
    }
}

void
Scan::detach()
{
    tower_servo.detach();
}

void
Scan::set_tower()
{
    int in[] = {0, (SCAN_BUF_LEN/2), SCAN_BUF_LEN};
    int out[] = {0, TOWER_SERVO_MIDDLE, 180};
    int new_pos = multi_map(pos, in, out, ARRAY_SIZE(in));
    tower_servo.write(new_pos);
    /* TODO add delay / timer / a way to read position (best) */
}

void
Scan::start()
{
    attach();
    working = true;
}

void
Scan::stop()
{
    detach();
    working = false;
}

bool
Scan::throttle()
{
    /* skip if work called too fast - let servo stabilize */
    static unsigned long last_millis = millis();

    const unsigned long delta_ms = millis() - last_millis;

    const bool pos_extreme = (pos == 0) || (pos == (SCAN_BUF_LEN - 1));
    if (pos_extreme == true && delta_ms < MIN_INTERSCAN_INTERVAL_MS) return true;
    if (pos_extreme == false && delta_ms < MIN_SCAN_INTERVAL_MS) return true;

    last_millis = millis();
    return false;
}

void
Scan::step_tower()
{
    /* traverse left and right */
    if (dir_inc == true) {
        pos++;
        if (pos >= (SCAN_BUF_LEN-1)) dir_inc = false;
    }
    else {
        pos--;
        if (pos <= 0) dir_inc = true;
    }
    set_tower();
}

void
Scan::work()
{
    if (working == false) return;
    if (throttle() == true) return;

    /* read distance */
    int16_t distance = -1;
    Wire2.beginTransmission(0x10);
    Wire2.write(0x00);
    Wire2.endTransmission();
    Wire2.requestFrom(0x10, 0x02);
    distance = Wire2.read();
    distance |= Wire2.read() << 8;
    Wire2.endTransmission();

    /* update value */
    if (dir_inc == true) {
        scan_buf[pos] = distance;
        scan_buf_updated[pos] = true;
    }

    step_tower();
}

