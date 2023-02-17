#include <Arduino.h>
#include "Servo/src/Servo.h"
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

    recalibrate_pos();

    display->print("scan init ok");
}

void
Scan::tower_init()
{
    pinMode(TOWER_CONTROL_PIN ,OUTPUT);
    digitalWrite(TOWER_CONTROL_PIN, HIGH);
    pos = SCAN_MIN_POS;
    /* TODO zeroing out tower position
     * both here at init and also recalibrating during runtime */
}

void
Scan::tower_step()
{
    digitalWrite(TOWER_CONTROL_PIN, LOW);
    delayMicroseconds(SCAN_CONST_STEPPER_STEP_DELAY_US);
    digitalWrite(TOWER_CONTROL_PIN, HIGH);
    pos++;
    if (pos > SCAN_MAX_POS) pos = SCAN_MIN_POS;
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
Scan::clear()
{
    for (int i = 0; i < SCAN_BUF_LEN; i++) {
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
    /* FIXME this logic is really shaky - use timer interrupts */
    static unsigned long last_millis = millis();
    const unsigned long delta_ms = millis() - last_millis;
    if (delta_ms < SCAN_INTERVAL_MS) return true;
    last_millis = millis();
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
Scan::lidar_trigger()
{
    set_reg_u8(LIDAR_REG_TRIG_ONE_SHOT,
               LIDAR_REG_TRIG_ONE_SHOT_VAL_TRIG_ONCE);
}

void
Scan::notify_scan_state()
{

    command->print(command->get_command_str(NOTIFY_SCAN));
    command->print(CMD_DELIMITER);

    for (int i = 0; i < SCAN_BUF_LEN; i++) {

        /* FIXME return 0 and MAX two times in next_pos getter instead of omitting updated boolean */
        command->print(scan_buf[i] + LIDAR_CONST_AXIS_OFFSET, DEC);
        //if (scan_buf_updated[i] == true) {
        //    command->print(scan_buf[i], DEC);
        //    scan_buf_updated[i] = false;
        //}

        if (i != SCAN_BUF_LEN - 1) {
            command->print(CMD_PARAM_SEPARATOR_DELIMITER);
        }
    }

    command->println(CMD_TERMINATOR);
}

bool
Scan::is_scan_full()
{
    for (int i = 0; i < SCAN_BUF_LEN; i++) {
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

    lidar_trigger();
    delayMicroseconds(LIDAR_CONST_AFTER_TRIG_DELAY_US);
    lidar_update(pos);

    tower_step();
}

void
Scan::recalibrate_pos()
{
    clear();

    int steps = SCAN_BUF_LEN * 4;
    while (steps--) {
        uint16_t distance = get_reg_u16(LIDAR_REG_DIST_LOW);
        if (distance < LIDAR_CONST_MIN_DIST) {
            /* calibration success */
            display->beep(5, 3);
            pos = 0;
            return;
        }
        tower_step();
        delayMicroseconds(SCAN_INTERVAL_MS * 1000);
    }

    /* calibration failure */
    display->beep(400, 5);
    return;
}
