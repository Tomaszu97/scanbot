#ifndef CONFIG_H
#define CONFIG_H

#define TOWER_SERVO_PIN PB6
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

#define TOWER_SERVO_US_MIN 600
#define TOWER_SERVO_US_MAX 2350
#define TOWER_SERVO_MIDDLE 85
#define SCAN_BUF_LEN 180
#define MIN_SCAN_INTERVAL_MS 4

#define DRIVE_SERVO_US_MIN 544
#define DRIVE_SERVO_US_MAX 2400
#define DRIVE_SERVO_IN_DEADZONE 199
#define DRIVE_SERVO_CURVE_LEFT_IN   {-390, -365, -341, -322, -269,    0,  269,  322,  341,  365,  390}
#define DRIVE_SERVO_CURVE_RIGHT_IN  {-390, -365, -341, -322, -269,    0,  269,  322,  341,  365,  390}
#define DRIVE_SERVO_CURVE_LEFT_OUT  { 180,  135,  130,  125,  120,  100,   60,    55,   50,  45,    0}
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

//#define PING_TIMEOUT_MS 5000
#define PING_TIMEOUT_MS 5000000
//#define DEBUG

#endif /* CONFIG_H */
