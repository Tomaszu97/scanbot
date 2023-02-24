#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "util.h"
#include "command.h"
#include "display.h"
#include "scan.h"

TwoWire Wire2 = TwoWire(SDA2, SCL2);

Scan *Scan::scan_;

Scan *
Scan::get_instance()
{
    if (Scan::scan_ == nullptr)
        Scan::scan_ = new Scan();
    return Scan::scan_;
}

Scan::Scan()
{
    display = Display::get_instance();
    command = Command::get_instance();

    if (lidar_init() == false) display->panic("E:lidar fail");
    tower_init();

    display->print("scan init ok");
}

void
Scan::tower_init()
{
    pinMode(TOWER_CONTROL_PIN ,OUTPUT);
    pinMode(TOWER_ENABLE_PIN ,OUTPUT);
    digitalWrite(TOWER_CONTROL_PIN, LOW);
    digitalWrite(TOWER_ENABLE_PIN, LOW);
}

void
Scan::tower_enable()
{
    digitalWrite(TOWER_ENABLE_PIN, LOW);
    delayMicroseconds(SCAN_CONST_STEPPER_STEP_DELAY_US);
}

void
Scan::tower_disable()
{
    digitalWrite(TOWER_ENABLE_PIN, HIGH);
    delayMicroseconds(SCAN_CONST_STEPPER_STEP_DELAY_US);
}

void
Scan::tower_step()
{
    digitalWrite(TOWER_CONTROL_PIN, HIGH);
    delayMicroseconds(SCAN_CONST_STEPPER_STEP_DELAY_US);
    digitalWrite(TOWER_CONTROL_PIN, LOW);
    pos++;
    if (pos > SCAN_MAX_POS) pos = 0;
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

    /* set frequency */
    set_reg_u16(LIDAR_REG_FPS_LOW, 250);
    if (get_reg_u16(LIDAR_REG_FPS_LOW) != 250) return false;

    return true;
}

void
Scan::clear()
{
    for (int i = 0; i < SCAN_STEPS_PER_ROTATION; i++) {
        scan_buf[i] = 0;
    }
}

void
Scan::start()
{
    state = SCAN_WORKING;
}

void
Scan::stop()
{
    clear();
    state = SCAN_DISABLED;
}

void
Scan::pause()
{
    if (state != SCAN_WORKING) return;
    tower_disable();
    clear();
    state = SCAN_PAUSED;
}

void
Scan::unpause()
{
    if (state != SCAN_PAUSED) return;
    tower_enable();
    first_scan_pos = pos;
    state = SCAN_WORKING;
}

bool
Scan::throttle()
{
    static unsigned long last_us = micros();

    const unsigned long current_us = micros();
    const unsigned long delta_us = current_us - last_us;
    if (delta_us < SCAN_INTERVAL_US) return true;
    last_us = current_us;
    return false;
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
    const unsigned int req_byte_count = 1;
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
    const unsigned int req_byte_count = 2;
    Wire2.beginTransmission(LIDAR_I2C_ADDRESS);
    Wire2.write(reg_low);
    Wire2.endTransmission();
    Wire2.requestFrom(LIDAR_I2C_ADDRESS, req_byte_count);
    ret_val = Wire2.read();
    ret_val |= Wire2.read() << 8;
    Wire2.endTransmission();
    return ret_val;
}

static unsigned int
min_mod_difference(unsigned int a,
               unsigned int b,
               unsigned int mod)
{
    if (a >= b)
        return min(a - b, (b + mod) - a);
    /* else if (a < b) */
        return min(b - a, (a + mod) - b);
}

void
Scan::adjust_pos()
{
    const unsigned int action_low_thr = 3;
    const unsigned int thr = 4;
    unsigned int cnt = 0;
    for (int p = 0;
         p < (SCAN_STEPS_PER_ROTATION + thr);
         p++) {
        unsigned int i = p % SCAN_STEPS_PER_ROTATION;
        uint16_t distance = scan_buf[i];

        if (distance <= LIDAR_CONST_MIN_DIST) cnt++;
        else cnt = 0;

        const bool pole_detected = (cnt >= thr);
        if (pole_detected == true) {

            const bool adjustment_significant = (min_mod_difference(i, zero_pos, SCAN_STEPS_PER_ROTATION) >= action_low_thr);
            if (adjustment_significant == true)
                zero_pos = i;

            break;
        }
    }
}

void
Scan::process_scan()
{
    /* overwrite out of bounds scans with dummy value */
    for (int i = 0; i < SCAN_STEPS_PER_ROTATION; i++) {
        uint16_t *distance = &scan_buf[i];
        if (*distance <= LIDAR_CONST_MIN_DIST ||
            *distance >= LIDAR_CONST_MAX_DIST) {
            *distance = LIDAR_CONST_DUMMY_DIST;
        }
    }
}

void
Scan::notify_scan_state()
{
    command->print(command->get_command_str(NOTIFY_SCAN));
    command->print(CMD_DELIMITER);

    for (int p = (SCAN_REAR_DEADZONE + 1);
         p <= (SCAN_MAX_POS - SCAN_REAR_DEADZONE);
         p++) {

        int buf_idx = (zero_pos + p) % SCAN_STEPS_PER_ROTATION;

        if (scan_buf[buf_idx] == LIDAR_CONST_DUMMY_DIST)
            command->print(scan_buf[buf_idx], DEC);
        else
            command->print(scan_buf[buf_idx] + LIDAR_CONST_AXIS_OFFSET, DEC);

        if (p != (SCAN_MAX_POS - SCAN_REAR_DEADZONE))
            command->print(CMD_PARAM_SEPARATOR_DELIMITER);
    }

    command->println(CMD_TERMINATOR);
}

void
Scan::lidar_update(unsigned int curr_pos)
{
    uint16_t distance = get_reg_u16(LIDAR_REG_DIST_LOW);
    scan_buf[curr_pos] = distance;
}

bool
Scan::is_scan_full()
{
    int distance = pos - first_scan_pos;
    if ( distance < 0 ) distance = SCAN_STEPS_PER_ROTATION + distance;

    if ( distance == (SCAN_STEPS_PER_ROTATION - 1) ) {
        first_scan_pos = (pos + 1) % SCAN_STEPS_PER_ROTATION;
        return true;
    }
    return false;
}

void
Scan::work()
{
    if (state != SCAN_WORKING) return;
    if (throttle() == true) return;

    lidar_update(pos);

    if (is_scan_full() == true) {
        adjust_pos();
        process_scan();
        notify_scan_state();
    }

    tower_step();
}
