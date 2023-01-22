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
    void set_tower(int pos);
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
    set_tower(0);
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
Scan::set_tower(int pos)
{
    int in[] = {-90, 0, 90};
    int out[] = {180, TOWER_SERVO_MIDDLE, 0};
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

void
Scan::work()
{
    static int pos = 0;
    static bool dir_inc = true; 
    static unsigned long last_millis = millis();

    if (working == false) return;

    /* skip if work called too fast - let servo stabilize */
    const unsigned long delta_ms = millis() - last_millis;
    if (delta_ms < MIN_SCAN_INTERVAL_MS) return;
    last_millis = millis();

    /* update value */
    int16_t distance = -1;
    Wire2.beginTransmission(0x10);
    Wire2.write(0x00);
    Wire2.endTransmission();
    Wire2.requestFrom(0x10, 0x02);
    distance = Wire2.read();
    distance |= Wire2.read() << 8;
    Wire2.endTransmission();
    const unsigned int i = 180 - (pos + 90);
    scan_buf[i] = distance;
    scan_buf_updated[i] = true;

    if (dir_inc == true) {
        pos++;
        if (pos >= 90) dir_inc = false;
    }
    else {
        pos--;
        if (pos <= -90) dir_inc = true;
    }
    set_tower(pos);
}

