/*
TODO
- change Strings to char arrays to save memory
- add argument parsing function
- migrate to new stm32duino core 
*/

#include <Arduino.h>
#include <Servo.h>
#include <U8x8lib.h>
#include <SPI.h>
#include "Wire/Wire.h"
#include "DFRobot_QMC5883/DFRobot_QMC5883.h"
#include <SimpleKalmanFilter.h>

#define TOWERSERVO_PIN PB6
#define LEFTSERVO_PIN PB7
#define RIGHTSERVO_PIN PB8
#define BUZZER_PIN PB13
#define OLED_CLK PB3
#define OLED_DATA PB5
#define OLED_CS PA0
#define OLED_DC PB14
#define OLED_RST PB15

Servo leftservo;
Servo rightservo;
Servo towerservo;
U8X8_SSD1306_128X64_NONAME_4W_SW_SPI u8x8(OLED_CLK, OLED_DATA, OLED_CS, OLED_DC, OLED_RST);
DFRobot_QMC5883 compass;
SimpleKalmanFilter compass_kalman(1, 1, 0.01);

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
    int in[] = {-90, 0, 90};
    int out[] = {180, 84, 0};
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

void setup()
{
    Serial.begin(115200);
    Serial2.begin(38400);
    u8x8.begin();
    u8x8.setFont(u8x8_font_pxplusibmcga_f);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    while (!compass.begin())
    {
        Serial.println("ERROR: CAN'T FIND QMC5883#");
        delay(200);
    }

    Serial.println("INFO: INITIALIZING QMC5883");
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);

    Serial.println("INFO: READY#");
}

void loop()
{
    //////////////////// compass
    //raw data
    Vector norm = compass.readNormalize();
    float heading = atan2(norm.YAxis, norm.XAxis);
    float declinationAngle = (4.0 + (26.0 / 60.0)) / 57.296; // For Bytom / Poland declination angle is 4'26E (positive)
    heading += declinationAngle;
    if (heading < 0) // Correct for heading < 0deg and heading > 360deg
    {
        heading += 2 * PI;
    }
    else if (heading > 2 * PI)
    {
        heading -= 2 * PI;
    }
    float headingDegrees = heading * 57.296; //180/pi=57.296

    //filtering
    float estimate = compass_kalman.updateEstimate(headingDegrees);

    //print
    Serial.print(headingDegrees);
    Serial.print(",");
    Serial.println(estimate);

    ////////////////////

    if (Serial2.available())
    {
        String command = Serial2.readStringUntil('#');
        command.trim();

        //Serial.println(command);

        //DRIVE:<left_motor_speed>,<right_motor_speed> range -90 to 90 each (ex DRIVE:56, 67)
        if (command.indexOf("DRIVE") != -1)
        {
            command = command.substring(command.indexOf(':') + 1);

            int leftval = command.substring(0, command.indexOf(',')).toInt();
            int rightval = command.substring(command.indexOf(',') + 1).toInt();

            driveMotors(leftval, rightval);
        }
        else if (command.indexOf("ROTATE_TOWER") != -1)
        {
            int val = command.substring(command.indexOf(':') + 1).toInt();
            rotateTower(val);
        }
        else if (command.indexOf("KILL") != -1)
        {
            motorsDetach();
            towerDetach();
        }
        else if (command.indexOf("PRINT") != -1)
        {
            char printdata[128];
            command.substring(command.indexOf(':') + 1).toCharArray(printdata, 128);
            u8x8.clear();
            //u8x8.print(printdata);
            u8x8.draw2x2String(0, 3, printdata);
        }
        else if (command.indexOf("BEEP") != -1)
        {
            command = command.substring(command.indexOf(':') + 1);

            int time_ms = command.substring(0, command.indexOf(',')).toInt();
            int count = command.substring(command.indexOf(',') + 1).toInt();

            beep(time_ms, count);
        }
    }
}