#include <Arduino.h>
#include <Servo.h>
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
    servo_init();

    display->print("scan init ok");
}

void
Scan::servo_init()
{
    pos = (SCAN_BUF_LEN/2);
    servo_set(pos);
    servo_attach();
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
Scan::servo_attach()
{
    if (tower_servo.attached() == false) {
        tower_servo.attach(TOWER_SERVO_PIN, TOWER_SERVO_US_MIN, TOWER_SERVO_US_MAX);
    }
}

void
Scan::servo_detach()
{
    tower_servo.detach();
}

void
Scan::servo_set(unsigned int position)
{
    int in[] = {0, (SCAN_BUF_LEN/2), SCAN_BUF_LEN};
    int out[] = {0, TOWER_SERVO_MIDDLE, 180};
    int raw_pos = multi_map(position, in, out, ARRAY_SIZE(in));
    tower_servo.write(raw_pos);
}

void
Scan::start()
{
    servo_attach();
    working = true;
}

void
Scan::stop()
{
    servo_detach();
    working = false;
}

bool
Scan::throttle()
{
    static unsigned long last_millis = millis();
    const unsigned long delta_ms = millis() - last_millis;

    const bool pos_extreme = (pos == 0) || (pos == (SCAN_BUF_LEN - 1));
    if (pos_extreme == true && delta_ms < SCAN_INTERSCAN_INTERVAL_MS) return true;
    if (pos_extreme == false && delta_ms < SCAN_INTERVAL_MS) return true;

    last_millis = millis();
    return false;
}

unsigned int
Scan::servo_next_pos(unsigned int curr_pos)
{
    static bool dir_inc = true;
    unsigned int next_pos;

    if (dir_inc == true) next_pos = curr_pos + 1;
    else next_pos = curr_pos - 1;
    if (curr_pos == (SCAN_BUF_LEN-1)) dir_inc = false;
    else if (curr_pos == 0) dir_inc = true;

    return next_pos;
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

        if (scan_buf_updated[i] == true) {
            command->print(scan_buf[i], DEC);
            scan_buf_updated[i] = false;
        }

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

    if (is_scan_full() == true)
        notify_scan_state();
}

void
Scan::work()
{
    if (working == false) return;
    if (throttle() == true) return;

    unsigned int next_pos = servo_next_pos(pos);
    servo_set(next_pos);

    lidar_trigger();
    delayMicroseconds(LIDAR_CONST_AFTER_TRIG_DELAY_US);
    if (next_pos > pos) {
        lidar_update(pos);
    }
    else {
        /* compensate */
        const unsigned int pos_w_compensation = pos + SCAN_DIRECTION_COMPENSATION;
        if (pos_w_compensation >=0 &&
                pos_w_compensation <= (SCAN_BUF_LEN - 1))
            lidar_update(pos_w_compensation);
    }

    pos = next_pos;
}