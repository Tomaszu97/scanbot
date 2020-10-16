/*
TODO
get rid of u8x8 or optimize it
move LIS3MDL out of library
get accel data
*/

//ST STM32 stack (STM32duino) 6.1.1
#include <Arduino.h>
#include <IWatchdog.h>
#include <EEPROM.h>
#include <Servo.h>
#include <U8x8lib.h>
#include <SPI.h>
#include "Wire2/Wire.h"
#include "LIS3MDL/LIS3MDL.h"

#define TOWER_SERVO_PIN PB6
#define LEFT_SERVO_PIN PB7
#define RIGHT_SERVO_PIN PB8
#define BUZZER_PIN PB13
#define OLED_CLK PB3
#define OLED_DATA PB5
#define OLED_CS PA0
#define OLED_DC PB14
#define OLED_RST PB15
#define SHARP_SENSOR_PIN PA5

#define TOWER_SERVO_MS_MIN 500
#define TOWER_SERVO_MS_MAX 2400
#define DRIVE_SERVO_US_MIN 544
#define DRIVE_SERVO_US_MAX 2400
#define DRIVE_SERVO_DEADZONE 1
#define DRIVE_SERVO_MIDDLE 105
int16_t hardiron_x;       //EEPROM ADDRESS 0
int16_t hardiron_y;       //EEPROM ADDRESS 2
float theta;              //EEPROM ADDRESS 4
float sigma;              //EEPROM ADDRESS 8
float sintheta, costheta; //not stored in eeprom

Servo leftservo;
Servo rightservo;
Servo towerservo;
U8X8_SSD1306_128X64_NONAME_4W_SW_SPI u8x8(OLED_CLK, OLED_DATA, OLED_CS, OLED_DC, OLED_RST);
HardwareSerial Serial2(PA3, PA2);
LIS3MDL mag;

const uint8_t printbuffer_size = 128;
char printbuffer[printbuffer_size];

const uint8_t command_count = 16;
String commands[command_count] = {
    "DRIVE",
    "ROTATE_TOWER",
    "KILL",
    "PRINT",
    "BEEP",
    "GET_MAG",
    "GET_AZIMUTH",
    "GET_DISTANCE",
    "GET_TIME",
    "MOVE",
    "ROTATE_TO",
    "ROTATE",
    "SCAN",
    "SET_MAG_CAL",
    "GET_MAG_CAL",
    "RESET"};

void reset()
{
    IWatchdog.begin(10000);
    while (1)
    {
    }
}

void towerAttach()
{
    towerservo.attach(TOWER_SERVO_PIN, TOWER_SERVO_MS_MIN, TOWER_SERVO_MS_MAX);
}

void towerDetach()
{
    towerservo.detach();
}

void motorsAttach()
{
    leftservo.attach(LEFT_SERVO_PIN, DRIVE_SERVO_US_MIN, DRIVE_SERVO_US_MAX);
    rightservo.attach(RIGHT_SERVO_PIN, DRIVE_SERVO_US_MIN, DRIVE_SERVO_US_MAX);
}

void motorsDetach()
{
    leftservo.detach();
    rightservo.detach();
}

int multiMap(int val, int *_in, int *_out, uint8_t size)
{
    // note: the _in array should have increasing values
    //constrain value between boundaries
    if (val <= _in[0])
        return _out[0];
    if (val >= _in[size - 1])
        return _out[size - 1];

    // search right interval
    uint8_t pos = 1; // _in[0] allready tested
    while (val > _in[pos])
        pos++;

    // this will handle all exact "points" in the _in array
    if (val == _in[pos])
        return _out[pos];

    // interpolate in the right segment for the rest
    return (val - _in[pos - 1]) * (_out[pos] - _out[pos - 1]) / (_in[pos] - _in[pos - 1]) + _out[pos - 1];
}

void rotateTower(int degrees)
{
    int in[] = {0, 90, 180};
    int out[] = {3, 75, 165};
    degrees = multiMap(degrees, in, out, 3);

    towerAttach();
    towerservo.write(degrees);
}

void driveMotors(int speed_left, int speed_right)
{
    int deadzone = DRIVE_SERVO_DEADZONE;
    int in[] = {-90, 0, 90};
    int left_out[] = {180, DRIVE_SERVO_MIDDLE, 0};
    int right_out[] = {0, DRIVE_SERVO_MIDDLE, 180};

    if (speed_left <= deadzone && speed_left >= -deadzone)
    {
        leftservo.detach();
    }
    else
    {
        int new_speed_left = multiMap(speed_left, in, left_out, 3);

        if (!leftservo.attached())
        {
            leftservo.attach(LEFT_SERVO_PIN, DRIVE_SERVO_US_MIN, DRIVE_SERVO_US_MAX);
        }

        leftservo.write(new_speed_left);
    }

    if (speed_right <= deadzone && speed_right >= -deadzone)
    {
        rightservo.detach();
    }
    else
    {
        int new_speed_right = multiMap(speed_right, in, right_out, 3);

        if (!rightservo.attached())
        {
            rightservo.attach(RIGHT_SERVO_PIN, DRIVE_SERVO_US_MIN, DRIVE_SERVO_US_MAX);
        }

        rightservo.write(new_speed_right);
    }
}

void beep(int time_ms, int count)
{
    for (int i = 0; i < count; i++)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(time_ms);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
    }
}

void print(char *str)
{
    u8x8.clear();
    u8x8.print(str);
}

int calcAngleDistance(int current, int target)
{
    int d = abs(target - current) % 360;
    int r = d > 180 ? 360 - d : d;
    int sign = (target - current >= 0 && target - current <= 180) || (target - current <= -180 && target - current >= -360) ? 1 : -1;
    r *= sign;

    return r;
}

int signed_mod(int value, int n)
{
    return value < 0 ? ((value + 1) % n) + n - 1 : value % n;
}

int getAzimuth()
{
    mag.read();
    //hard iron offset correction
    float x = float(int16_t(mag.m.x) + hardiron_x);
    float y = float(int16_t(mag.m.y) + hardiron_y);
    //soft iron correction - rotate ellipse
    float x2 = (x * costheta) + (y * sintheta);
    float y2 = (x * -sintheta) + (y * costheta);
    //soft iron correction - squish ellipse to make it a circle
    x2 *= sigma;

    int yaw = signed_mod(-(atan2(x2, y2) * RAD_TO_DEG), 360); //inverted so value rises when turning right
    return yaw;
}

void rotateTo(int azimuth)
{
    if (calcAngleDistance(getAzimuth(), azimuth) > 0)
        driveMotors(90, -90);
    else
        driveMotors(-90, 90);

    while (true)
    {
        if (abs(calcAngleDistance(getAzimuth(), azimuth)) <= 10)
            break;
        delay(10);
    }
    driveMotors(0, 0);
    delay(100);
}

int getDistance()
{
    int16_t distance = -1;
    Wire.beginTransmission(0x10);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(0x10, (uint8_t)2);
    distance = Wire.read();
    distance |= Wire.read() << 8;
    Wire.endTransmission();

    return distance;
}

String getArgument(String command, uint8_t argnum = 0)
{
    char delimiter = ':';
    for (uint8_t i = 0; i < 10; i++)
    {
        if (argnum == i)
        {
            if (command.indexOf(delimiter) == -1)
                return command;
            else
                return command.substring(0, command.indexOf(delimiter));
        }

        if (command.indexOf(delimiter) == -1)
            return "";
        else
            command = command.substring(command.indexOf(delimiter) + 1);

        //change it only once
        if (i == 0)
            delimiter = ',';
    }

    return "";
}

uint8_t parseCommand(String command)
{
    command = getArgument(command, 0);

    for (uint8_t i = 0; i < command_count; i++)
    {
        if (command == commands[i])
            return i;
    }
    return -1;
}

void printMagCal()
{
    int16_t tempint;
    float tempfloat;
    EEPROM.get(0, tempint);
    Serial2.print(tempint, DEC);
    Serial2.print(",");
    EEPROM.get(2, tempint);
    Serial2.print(tempint, DEC);
    Serial2.print(",");
    EEPROM.get(4, tempfloat);
    Serial2.print(tempfloat, DEC);
    Serial2.print(",");
    EEPROM.get(8, tempfloat);
    Serial2.print(tempfloat, DEC);
    Serial2.println("");
}

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    Serial2.begin(38400);
    u8x8.begin();
    u8x8.setFont(u8x8_font_pxplusibmcga_f);
    mag.init();
    mag.enableDefault();
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    EEPROM.get(0, hardiron_x);
    EEPROM.get(2, hardiron_y);
    EEPROM.get(4, theta);
    EEPROM.get(8, sigma);
    sintheta = sin(theta);
    costheta = cos(theta);

    beep(30, 3);
    Serial.println("Debug Serial ready");
    print("ready");
}

void loop()
{
    if (Serial2.available())
    {
        String command = Serial2.readStringUntil('#');
        command.trim();

        switch (parseCommand(command))
        {
        // DRIVE
        case 0:
        {
            driveMotors(getArgument(command, 1).toInt(), getArgument(command, 2).toInt());
            Serial2.println("OK");
        }
        break;

        // ROTATE_TOWER
        case 1:
        {
            rotateTower(getArgument(command, 1).toInt());
            Serial2.println("OK");
        }
        break;

        // KILL
        case 2:
        {
            motorsDetach();
            towerDetach();
            Serial2.println("OK");
        }
        break;

        // PRINT
        case 3:
        {
            command.substring(command.indexOf(':') + 1).toCharArray(printbuffer, printbuffer_size);
            print(printbuffer);
            Serial2.println("OK");
        }
        break;

        // BEEP
        case 4:
        {
            beep(getArgument(command, 1).toInt(), getArgument(command, 2).toInt());
            Serial2.println("OK");
        }
        break;

        // GET_MAG
        case 5:
        {
            mag.read();
            ////COPY-PASTE from getAzimuth()
            //hard iron offset correction
            float x = float(int16_t(mag.m.x) + hardiron_x);
            float y = float(int16_t(mag.m.y) + hardiron_y);
            //soft iron correction - rotate ellipse
            float x2 = (x * costheta) + (y * sintheta);
            float y2 = (x * -sintheta) + (y * costheta);
            //soft iron correction - squish ellipse to make it a circle
            x2 *= sigma;
            ////
            Serial2.print(int(x2), DEC);
            Serial2.print(",");
            Serial2.print(int(y2), DEC);
            Serial2.println("");
        }
        break;

        //GET_AZIMUTH
        case 6:
        {
            Serial2.println(getAzimuth(), DEC);
        }
        break;

        // GET_DISTANCE
        case 7:
        {
            Serial2.println(getDistance(), DEC);
        }
        break;

        // GET_TIME
        case 8:
        {
            Serial2.println(millis(), DEC);
        }
        break;

        //MOVE
        case 9:
        {
            int val = getArgument(command, 1).toInt();
            int dtime = abs(val) * 100;
            if (val > 0)
                driveMotors(90, 90);
            else
                driveMotors(-90, -90);
            delay(dtime);
            driveMotors(0, 0);
            Serial2.println("OK");
        }
        break;

        // ROTATE_TO
        case 10:
        {
            rotateTo(getArgument(command, 1).toInt());
            Serial2.println(getAzimuth(), DEC);
        }
        break;

        // ROTATE
        case 11:
        {
            rotateTo((getAzimuth() + getArgument(command, 1).toInt()) % 360);
            Serial2.println(getAzimuth(), DEC);
        }
        break;

        //SCAN
        case 12:
        {
            rotateTower(0);
            delay(300);
            for (uint8_t i = 0; i <= 180; i += 1)
            {
                rotateTower(i);
                delay(5);
                Serial2.print(getDistance(), DEC);
                if (i < 180)
                    Serial2.print(",");
            }
            Serial2.println("");
        }
        break;

        //SET_MAG_CAL
        case 13:
        {
            hardiron_x = getArgument(command, 1).toInt();
            hardiron_y = getArgument(command, 2).toInt();
            theta = getArgument(command, 3).toFloat();
            sigma = getArgument(command, 4).toFloat();
            sintheta = sin(theta);
            costheta = cos(theta);

            EEPROM.put(0, hardiron_x);
            EEPROM.put(2, hardiron_y);
            EEPROM.put(4, theta);
            EEPROM.put(8, sigma);

            printMagCal();
        }
        break;

        //GET_MAG_CAL
        case 14:
        {
            printMagCal();
        }
        break;

        //RESET
        case 15:
        {
            reset();
        }
        break;

        default:
        {
            Serial2.println("COMMAND ERROR");
        }
        break;
        }
    }
}