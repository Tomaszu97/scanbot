#include <Arduino.h>
#include <Servo.h>

#define TOWERSERVO_PIN PB6
#define LEFTSERVO_PIN PB7
#define RIGHTSERVO_PIN PB8

Servo leftservo;
Servo rightservo;
Servo towerservo;

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

void setup()
{
    Serial.begin(115200);
    Serial2.begin(38400);
    delay(2000); //wait for serials to start
    Serial.println("READY");
}

void loop()
{
    if (Serial2.available())
    {
        String command = Serial2.readStringUntil('#');
        command.trim();

        Serial.println(command);

        //DRIVE:<left_motor_speed>,<right_motor_speed> range -90 to 90 each (ex DRIVE:56, 67)
        if (command.indexOf("DRIVE") != -1)
        {
            command = command.substring(command.indexOf(':') + 1);

            int leftval = command.substring(0, command.indexOf(',')).toInt();
            int rightval = command.substring(command.indexOf(',') + 1).toInt();

            driveMotors(leftval, rightval);
        }
        if (command.indexOf("ROTATE_TOWER") != -1)
        {
            int val = command.substring(command.indexOf(':') + 1).toInt();
            rotateTower(val);
        }
        else if (command.indexOf("KILL") != -1)
        {
            motorsDetach();
            towerDetach();
        }
    }
}
