#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "config.h"
#include "util.cpp.h"

TwoWire Wire2 = TwoWire(SDA2, SCL2);

class Scan
{
private:
    bool lidar_init();
    void servo_init();
    void attach();
    void detach();
    void set_tower();
    bool throttle();
    void step_tower();
    void set_reg_u8(uint8_t reg, uint8_t val);
    void set_reg_u16(uint8_t reg_low, uint16_t val);
    uint8_t get_reg_u8(uint8_t reg);
    uint16_t get_reg_u16(uint8_t reg_low);
    bool working = false;
    Servo tower_servo;
    int pos;
    bool dir_inc;

public:
    bool init();
    void start();
    void stop();
    void work();
    uint16_t scan_buf[SCAN_BUF_LEN];
    bool scan_buf_updated[SCAN_BUF_LEN];
};

Scan scan;

bool
Scan::init()
{
    if (lidar_init() == false) return false;
    servo_init();
    return true;
}

void
Scan::servo_init()
{
    pos = (SCAN_BUF_LEN/2);
    dir_inc = true;
    set_tower();
    attach();
}

bool
Scan::lidar_init()
{
    Wire2.begin();
    memset(scan_buf, LIDAR_CONST_DUMMY_DIST, sizeof(scan_buf));

    /* set invalid measurement value */
    set_reg_u16(LIDAR_REG_DUMMY_DIST_LOW, LIDAR_CONST_DUMMY_DIST);
    if (get_reg_u16(LIDAR_REG_DUMMY_DIST_LOW) != LIDAR_CONST_DUMMY_DIST) return false;

    /* set valid distance range */
    set_reg_u16(LIDAR_REG_MIN_DIST_LOW, LIDAR_CONST_MIN_DIST * 10);
    if (get_reg_u16(LIDAR_REG_MIN_DIST_LOW) != LIDAR_CONST_MIN_DIST * 10) return false;
    set_reg_u16(LIDAR_REG_MAX_DIST_LOW, LIDAR_CONST_MAX_DIST * 10);
    if (get_reg_u16(LIDAR_REG_MAX_DIST_LOW) != LIDAR_CONST_MAX_DIST * 10) return false;

    /* set valid amp (signal strength) range */
    set_reg_u16(LIDAR_REG_AMP_THR_LOW, LIDAR_CONST_MIN_AMP);
    if (get_reg_u16(LIDAR_REG_AMP_THR_LOW) != LIDAR_CONST_MIN_AMP) return false;

    /* set trigger mode */
    set_reg_u8(LIDAR_REG_MODE, LIDAR_REG_MODE_VAL_TRIG);
    if (get_reg_u8(LIDAR_REG_MODE) != LIDAR_REG_MODE_VAL_TRIG) return false;

    return true;
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
    /* sweep back and forth */
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
Scan::set_reg_u8(uint8_t reg,
                 uint8_t val)
{
    Wire2.beginTransmission(LIDAR_I2C_ADDRESS);
    Wire2.write(reg);
    Wire2.write(val);
    Wire2.endTransmission();
}

void
Scan::set_reg_u16(uint8_t reg_low,
                  uint16_t val)
{
    const uint8_t low = val & 0x00ff;
    const uint8_t high = ((val & 0xff00) >> 8);
    Wire2.beginTransmission(LIDAR_I2C_ADDRESS);
    Wire2.write(reg_low);
    Wire2.write(low);
    Wire2.write(high);
    Wire2.endTransmission();
}

uint8_t
Scan::get_reg_u8(uint8_t reg)
{
    uint8_t ret_val;
    const uint8_t req_byte_count = 1;
    Wire2.beginTransmission(LIDAR_I2C_ADDRESS);
    Wire2.write(reg);
    Wire2.endTransmission();
    Wire2.requestFrom(LIDAR_I2C_ADDRESS, req_byte_count);
    ret_val = Wire2.read();
    Wire2.endTransmission();
    return ret_val;
}

uint16_t
Scan::get_reg_u16(uint8_t reg_low)
{
    uint16_t ret_val = 0x0000;
    const uint8_t req_byte_count = 2;
    Wire2.beginTransmission(LIDAR_I2C_ADDRESS);
    Wire2.write(reg_low);
    Wire2.endTransmission();
    Wire2.requestFrom(LIDAR_I2C_ADDRESS, req_byte_count);
    ret_val = Wire2.read();
    ret_val |= Wire2.read() << 8;
    Wire2.endTransmission();
    return ret_val;
}

void
Scan::work()
{
    if (working == false) return;
    if (throttle() == true) return;

    /* read distance */
    set_reg_u8(LIDAR_REG_TRIG_ONE_SHOT,
               LIDAR_REG_TRIG_ONE_SHOT_VAL_TRIG_ONCE);
    delayMicroseconds(LIDAR_CONST_DELAY_AFTER_TRIG_US);
    uint16_t distance = get_reg_u16(LIDAR_REG_DIST_LOW);

    /* constrain value */
    /* LIDAR will report dummy value in case of weak
     * signal, however when hitting range boundary it
     * returns mininum / maximum value. Code below
     * ensures returning dummy value instead.
     */
    if (distance <= LIDAR_CONST_MIN_DIST ||
        distance >= LIDAR_CONST_MAX_DIST) {
        distance = LIDAR_CONST_DUMMY_DIST;
    }

    /* update value */
    scan_buf[pos] = distance;
    scan_buf_updated[pos] = true;

    step_tower();
}
