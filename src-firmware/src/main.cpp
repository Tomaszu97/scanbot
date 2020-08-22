/*
TODO
- change Strings to char arrays to save memory
- add argument parsing function
[OK] migrate to new stm32duino core
*/

//ST STM32 stack (STM32duino) 6.1.1
#include <Arduino.h>
#include <Servo.h>
#include <U8x8lib.h>
#include <SPI.h>
#include "Wire2/Wire.h"
#include "LIS3MDL/LIS3MDL.h"
#include "SharpIR/SharpIR.h"
//#include <SimpleKalmanFilter.h>

#define TOWERSERVO_PIN PB6
#define LEFTSERVO_PIN PB7
#define RIGHTSERVO_PIN PB8
#define BUZZER_PIN PB13
#define OLED_CLK PB3
#define OLED_DATA PB5
#define OLED_CS PA0
#define OLED_DC PB14
#define OLED_RST PB15
#define SHARP_SENSOR_PIN PA5

Servo leftservo;
Servo rightservo;
Servo towerservo;
U8X8_SSD1306_128X64_NONAME_4W_SW_SPI u8x8(OLED_CLK, OLED_DATA, OLED_CS, OLED_DC, OLED_RST);
HardwareSerial Serial2(PA3, PA2);
LIS3MDL mag;
SharpIR IRsensor(1, SHARP_SENSOR_PIN); //GP2Y0A21YK0F
//SimpleKalmanFilter compass_kalman(1, 1, 0.01);

void towerAttach()
{
    towerservo.attach(TOWERSERVO_PIN, 544, 2400); //2 deg to 197 deg?
}

void towerDetach()
{
    towerservo.detach();
}

void motorsAttach()
{
    leftservo.attach(LEFTSERVO_PIN, 544, 2400);
    rightservo.attach(RIGHTSERVO_PIN, 544, 2400);
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
    int out[] = {0, 84, 180};
    degrees = multiMap(degrees, in, out, 3);

    towerAttach();
    towerservo.write(degrees);
}

void driveMotors(int speed_left, int speed_right)
{
    int deadzone = 3;
    int in[] = {-90, 0, 90};
    int left_out[] = {180, 105, 0};
    int right_out[] = {0, 105, 180};

    if (speed_left <= deadzone && speed_left >= -deadzone)
    {
        leftservo.detach();
    }
    else
    {
        int new_speed_left = multiMap(speed_left, in, left_out, 3);

        if (!leftservo.attached())
        {
            leftservo.attach(LEFTSERVO_PIN, 544, 2400);
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
            rightservo.attach(RIGHTSERVO_PIN, 544, 2400);
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
    //u8x8.draw2x2String(0, 3, str);
}

int getAzimuth()
{
    int mx_r, my_r, mz_r;
    float yaw;

    mag.read();
    mx_r = mag.m.x + 1620;
    my_r = mag.m.y + 1307;
    mz_r = mag.m.z - 3082; //hard iron offset correction
    yaw = (atan2(my_r, mx_r) * RAD_TO_DEG) + 180.0;

    if (yaw >= 0 && yaw < 40)
    {
        int intab[2] = {0, 40};
        int outtab[2] = {320, 360};
        yaw = multiMap(yaw, intab, outtab, 2);
    }
    else
    {
        int intab[5] = {40, 117, 172, 257, 360};
        int outtab[5] = {0, 90, 180, 270, 320};
        yaw = multiMap(yaw, intab, outtab, 5);
    }
    yaw += 150;
    return int(yaw) % 360;
}

int calcAngleDistance(int current, int target)
{
    int d = abs(target - current) % 360;
    int r = d > 180 ? 360 - d : d;
    int sign = (target - current >= 0 && target - current <= 180) || (target - current <= -180 && target - current >= -360) ? 1 : -1;
    r *= sign;

    return r;
}

void rotateTo(int azimuth)
{
    if (calcAngleDistance(getAzimuth(), azimuth) > 0)
        driveMotors(90, -90);
    else
        driveMotors(-90, 90);

    while (true)
    {
        if (abs(calcAngleDistance(getAzimuth(), azimuth)) <= 5)
            break;
        delay(10);
    }
    driveMotors(0, 0);
    delay(100);
}

int getDistance()
{
    return (IRsensor.getDistance() / 0.8);
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

    beep(5, 3);
    Serial.println("Serial ready");
    Serial2.println("Serial2 ready");
    print("ready");
}

void loop()
{
    if (Serial2.available())
    {
        String command = Serial2.readStringUntil('#');
        command.trim();

        //DRIVE:<left_motor_speed>,<right_motor_speed> range -90 to 90 each (ex DRIVE:56, 67)
        if (command.indexOf("DRIVE") != -1)
        {
            command = command.substring(command.indexOf(':') + 1);

            int leftval = command.substring(0, command.indexOf(',')).toInt();
            int rightval = command.substring(command.indexOf(',') + 1).toInt();

            driveMotors(leftval, rightval);
            Serial2.println("OK");
        }
        else if (command.indexOf("ROTATE_TOWER") != -1)
        {
            int val = command.substring(command.indexOf(':') + 1).toInt();
            rotateTower(val);
            Serial2.println("OK");
        }
        else if (command.indexOf("KILL") != -1)
        {
            motorsDetach();
            towerDetach();
            Serial2.println("OK");
        }
        else if (command.indexOf("PRINT") != -1)
        {
            char printdata[128];
            command.substring(command.indexOf(':') + 1).toCharArray(printdata, 128);
            print(printdata);
            Serial2.println("OK");
        }
        else if (command.indexOf("BEEP") != -1)
        {
            command = command.substring(command.indexOf(':') + 1);

            int time_ms = command.substring(0, command.indexOf(',')).toInt();
            int count = command.substring(command.indexOf(',') + 1).toInt();

            beep(time_ms, count);
            Serial2.println("OK");
        }
        else if (command.indexOf("GET_AZIMUTH") != -1)
        {
            float yaw = getAzimuth();

            char outstr[8];
            dtostrf(yaw, 8, 2, outstr);
            Serial2.print(String(yaw));
            Serial2.println("");
            print(outstr);
        }
        else if (command.indexOf("GET_DISTANCE") != -1)
        {
            Serial2.println(getDistance(), DEC);
        }
        else if (command.indexOf("GET_TIME") != -1)
        {
            Serial2.println(millis(), DEC);
        }
        else if (command.indexOf("MOVE") != -1)
        {
            int val = command.substring(command.indexOf(':') + 1).toInt();

            int dtime = abs(val) * 100;
            if (val > 0)
                driveMotors(90, 90);
            else
                driveMotors(-90, -90);
            delay(dtime);
            driveMotors(0, 0);
            Serial2.println("OK");
        }
        else if (command.indexOf("ROTATE_TO") != -1)
        {
            int val = command.substring(command.indexOf(':') + 1).toInt();
            rotateTo(val);
            Serial2.println(getAzimuth(), DEC);
        }
        else if (command.indexOf("ROTATE") != -1)
        {
            int val = command.substring(command.indexOf(':') + 1).toInt();
            rotateTo((getAzimuth() + val) % 360);
            Serial2.println(getAzimuth(), DEC);
        }
        else if (command.indexOf("SCAN") != -1)
        {
            rotateTower(0);
            delay(200);
            for (int i = 0; i <= 180; i += 10)
            {
                rotateTower(i);
                delay(80);
                Serial2.print(getDistance(), DEC);
                Serial2.print(",");
            }
            Serial2.println("");
        }
    }
}