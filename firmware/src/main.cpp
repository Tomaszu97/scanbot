#include <Arduino.h>
#include "util.h"
#include "display.h"
#include "command.h"
#include "scan.h"
#include "drive.h"
#include "watchdog.h"

Display *display;
Command *command;
Scan *scan;
Drive *drive;
Watchdog *watchdog;

void
cmd_handle(command_t cmd)
{
    switch (cmd.type) {
        case PING:
            if (command->assert_argc(1 + 0, cmd) == false) break;
            break;

        case DRIVE:
            if (command->assert_argc(1 + 2, cmd) == false) break;
            drive->set_speed(atoi(cmd.argv[1]));
            drive->set_turn(atoi(cmd.argv[2]));
            drive->update_motors();
            break;

        case DRIVE_RAW:
            if (command->assert_argc(1 + 2, cmd) == false) break;
            drive->set_motors(atoi(cmd.argv[1]),
                              atoi(cmd.argv[2]));
            //TODO async odometry
            //command.print(drive.get_left_encoder_counter());
            //command.print(CMD_PARAM_SEPARATOR_DELIMITER);
            //command.print(drive.get_right_encoder_counter());
            //command.println(CMD_TERMINATOR);
            //drive->reset_encoder_counters();
            break;

        case KILL:
            if (command->assert_argc(1 + 0, cmd) == false) break;
            drive->detach();
            break;

        case PRINT:
            if (command->assert_argc(1 + 1, cmd) == false) break;
            display->print(cmd.argv[1]);
            break;

        case CLEAR:
            if (command->assert_argc(1 + 0, cmd) == false) break;
            display->clear();
            break;

        case BEEP:
            if (command->assert_argc(1 + 2, cmd) == false) break;
            display->beep(atoi(cmd.argv[1]), atoi(cmd.argv[2]));
            break;

        case SCAN_START:
            if (command->assert_argc(1 + 0, cmd) == false) break;
            scan->start();
            break;

        case SCAN_STOP:
            if (command->assert_argc(1 + 0, cmd) == false) break;
            scan->stop();
            break;

            //TODO async scan results
//        case GET_SCAN:
//            {
//                if (assert_argc(1 + 0, cmd) == false) break;
//
//                uint16_t *scan_array = scan.scan_buf;
//                bool *scan_array_updated = scan.scan_buf_updated;
//
//                for (int i = 0; i < SCAN_BUF_LEN; i++) {
//                    if (scan_array_updated[i] == true) {
//                        command.print(scan_array[i], DEC);
//                        scan_array_updated[i] = false;
//                    }
//                    if (i != SCAN_BUF_LEN - 1) command.print(",");
//                }
//                command.println(CMD_TERMINATOR);
//                break;
//            }

        case RESET_PLATFORM:
            if (command->assert_argc(1 + 0, cmd) == false) break;
            reset_platform();
            break;

        case SET_TOWER:
            if (command->assert_argc(1 + 1, cmd) == false) break;
            scan->servo_attach();
            scan->servo_set(atoi(cmd.argv[1]));
            break;

        case NO_COMMAND:
            break;

        default:
            display->print("E:unhandled cmd", true);
            break;
    }
}

bool
can_cmd_ping(command_t cmd)
{
    switch (cmd.type) {
        case PING:
        case DRIVE:
        case DRIVE_RAW:
        case KILL:
        case PRINT:
        case CLEAR:
        case BEEP:
        case SCAN_START:
        case SCAN_STOP:
        case SET_TOWER:
            return true;
    }
    return false;
}

bool
can_cmd_wakeup(command_t cmd)
{
    switch (cmd.type) {
        case PING:
            return true;
    }
    return false;
}

void
cmd_watchdog_work(command_t cmd)
{
    if (can_cmd_ping(cmd)) watchdog->ping();
    if (watchdog->is_active() == true) {
        drive->detach();
        scan->stop();
        display->beep(50,4);
        while (watchdog->is_active() == true) {
            command_t cmd = command->get_command();
            if (can_cmd_wakeup(cmd)) watchdog->ping();
        }
    }
}

void
setup()
{
    SystemClock_Config();

    display = Display::get_instance();
    command = Command::get_instance();
    scan = Scan::get_instance();
    drive = Drive::get_instance();
    watchdog = Watchdog::get_instance();

    if (display == nullptr) while (true);
    if (command == nullptr ||
        scan == nullptr ||
        drive == nullptr ||
        watchdog == nullptr)
        display->panic("E:singleton init");

    display->beep(30, 3);
    display->print("init ok");
}

void
loop()
{
    command_t cmd = command->get_command();
    cmd_watchdog_work(cmd);
    cmd_handle(cmd);
    scan->work();
}
