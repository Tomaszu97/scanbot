#ifndef CONFIG_H
#define CONFIG_H

/* constant parameters */
#define TOWER_CONTROL_PIN PB6
#define LEFT_SERVO_PIN PB7
#define RIGHT_SERVO_PIN PB8
#define BUZZER_PIN PB13
#define OLED_CLK PB3
#define OLED_DATA PB5
#define OLED_CS PA0
#define OLED_DC PB14
#define OLED_RST PB15
#define LEFT_ENCODER_PIN_A PB1
#define LEFT_ENCODER_PIN_B PB0
#define RIGHT_ENCODER_PIN_A PA7
#define RIGHT_ENCODER_PIN_B PA6
#define BLUETOOTH_SERIAL_RX PA3
#define BLUETOOTH_SERIAL_TX PA2
#define SDA2 PB11
#define SCL2 PB10
#define LIDAR_I2C_ADDRESS 0x10
#define LIDAR_REG_DIST_LOW 0x00
#define LIDAR_REG_DIST_HIGH 0x01
#define LIDAR_REG_MODE 0x23
#define LIDAR_REG_MODE_VAL_CONT 0x00
#define LIDAR_REG_MODE_VAL_TRIG 0x01
#define LIDAR_REG_TRIG_ONE_SHOT 0x24
#define LIDAR_REG_TRIG_ONE_SHOT_VAL_TRIG_ONCE 0x01
#define LIDAR_REG_DUMMY_DIST_LOW 0x2c
#define LIDAR_REG_DUMMY_DIST_HIGH 0xd
#define LIDAR_REG_MIN_DIST_LOW 0x2e
#define LIDAR_REG_MIN_DIST_HIGH 0x2f
#define LIDAR_REG_MAX_DIST_LOW 0x30
#define LIDAR_REG_MAX_DIST_HIGH 0x31
#define LIDAR_REG_AMP_THR_LOW 0x2a
#define LIDAR_REG_AMP_THR_HIGH 0x2b
#define LIDAR_CONST_MIN_AMP 100
#define LIDAR_CONST_DUMMY_DIST 0
#define LIDAR_CONST_MIN_DIST 19
#define LIDAR_CONST_MAX_DIST 501
#define LIDAR_CONST_AXIS_OFFSET -5
#define LIDAR_CONST_AFTER_TRIG_DELAY_US 100
#define TOWER_CONST_STEPPER_STEP_DELAY_US 200
#define SCAN_MIN_POS 0
#define SCAN_MAX_POS 199
#define SCAN_BUF_LEN ( SCAN_MAX_POS - SCAN_MIN_POS + 1 )
#define DRIVE_SERVO_US_MIN 544
#define DRIVE_SERVO_US_MAX 2400
#define DRIVE_SERVO_IN_DEADZONE 119
#define DRIVE_SERVO_CURVE_LEFT_IN   {-390, -365, -341, -322, -269,    0,  269,  322,  341,  365,  390}
#define DRIVE_SERVO_CURVE_RIGHT_IN  {-390, -365, -341, -322, -269,    0,  269,  322,  341,  365,  390}
#define DRIVE_SERVO_CURVE_LEFT_OUT  { 180,  135,  130,  125,  120,  100,   60,   55,   50,  45,    0}
#define DRIVE_SERVO_CURVE_RIGHT_OUT {   0,   45,   50,   55,   60,  105,  120,  125,  130,  135,  180}
#define MIN_SPEED -390
#define MAX_SPEED 390
#define MIN_TURN -390
#define MAX_TURN 390
#define DEBUG_SERIAL_BAUD 115200
#define COMMAND_SERIAL_BAUD 38400
#define DISPLAY_PRINT_BUF_SIZE 128
#define OLED_CHAR_WIDTH 16
#define OLED_CHAR_HEIGHT 8
#define CMD_BUF_SIZE 256
#define CMD_MAX_ARG_N 20
#define CMD_DELIMITER ":"
#define CMD_PARAM_SEPARATOR_DELIMITER ","
#define CMD_TERMINATOR '#'
#define PING_TIMEOUT_MS 5000
//#define DEBUG

#define SCAN_INTERVAL_MS 4
#define SCAN_SETPOINT_OVERSHOOT 12

#endif /* CONFIG_H */
