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
    digitalWrite(TOWER_CONTROL_PIN, LOW);
    pos = 0;
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
        scan_buf_updated[i] = false;
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
    clear();
    state = SCAN_PAUSED;
}

void
Scan::unpause()
{
    if (state != SCAN_PAUSED) return;
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

void
Scan::notify_scan_state()
{

    command->print(command->get_command_str(NOTIFY_SCAN));
    command->print(CMD_DELIMITER);

    for (int i = (SCAN_REAR_DEADZONE + 1);
         i <= (SCAN_MAX_POS - SCAN_REAR_DEADZONE);
         i++) {

        if (scan_buf[i] == LIDAR_CONST_DUMMY_DIST)
            command->print(scan_buf[i], DEC);
        else
            command->print(scan_buf[i] + LIDAR_CONST_AXIS_OFFSET, DEC);

        if (i != (SCAN_MAX_POS - SCAN_REAR_DEADZONE)) {
            command->print(CMD_PARAM_SEPARATOR_DELIMITER);
        }
    }

    command->println(CMD_TERMINATOR);
}

bool
Scan::is_scan_full()
{
    for (int i = 0; i < SCAN_STEPS_PER_ROTATION; i++) {
        if (scan_buf_updated[i] == false) {
            return false;
        }
    }
    return true;
}

void
Scan::lidar_update(unsigned int curr_pos)
{
    uint16_t distance = get_reg_u16(LIDAR_REG_DIST_LOW);
    if (distance <= LIDAR_CONST_MIN_DIST ||
            distance >= LIDAR_CONST_MAX_DIST) {
        distance = LIDAR_CONST_DUMMY_DIST;
    }
    scan_buf[curr_pos] = distance;
    scan_buf_updated[curr_pos] = true;

    if (curr_pos == SCAN_MAX_POS) {
        notify_scan_state();
    }
}

void
Scan::work()
{
    if (state != SCAN_WORKING) return;
    if (throttle() == true) return;

    lidar_update(pos);
    tower_step();
}
