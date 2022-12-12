#include <Arduino.h>
#include <IWatchdog.h>

#include "config.h"
#include "scan.cpp.h"
#include "drive.cpp.h"
#include "display.cpp.h"
#include "command.cpp.h"
#include "util.cpp.h"

static unsigned long ping_watchdog;

bool
assert_argc(const unsigned int argc,
            const command_t command)
{
    if (command.argc == argc) return true;

    if(command.argc > 0) {
        display.dbg_print("E:incorrect argc ");
    }
    else {
        display.dbg_print("E:no arguments");
    }
    return false;
}

void
setup()
{
    SystemClock_Config();
    scan.init();
    drive.init();
    display.init();
    command.init();

    ping_watchdog = millis();
}

void
loop()
{
    command_t cmd = command.get_command();

    switch (cmd.type) {
        case PING:
            if (assert_argc(1 + 0, cmd) == false) break;
            ping_watchdog = millis();
            command.respond(true);
            break;

        case DRIVE:
            if (assert_argc(1 + 2, cmd) == false) break;
            drive.set_speed(atoi(cmd.argv[1]));
            drive.set_turn(atoi(cmd.argv[2]));
            drive.update_motors();
            command.respond(true);
            break;

        case KILL:
            if (assert_argc(1 + 0, cmd) == false) break;
            drive.detach();
            command.respond(true);
            break;

        case PRINT:
            if (assert_argc(1 + 1, cmd) == false) break;
            display.print(cmd.argv[1]);
            command.respond(true);
            break;

        case CLEAR:
            display.clear();
            command.respond(true);
            break;

        case BEEP:
            if (assert_argc(1 + 2, cmd) == false) break;
            display.beep(atoi(cmd.argv[1]), atoi(cmd.argv[2]));
            command.respond(true);
            break;

        case GET_ENCODERS:
            if (assert_argc(1 + 0, cmd) == false) break;
            command.print(drive.get_left_encoder_counter());
            command.print(CMD_PARAM_SEPARATOR_DELIMITER);
            command.print(drive.get_right_encoder_counter());
            command.println(CMD_TERMINATOR);
            drive.reset_encoder_counters();
            break;

        case GET_TIME:
            if (assert_argc(1 + 0, cmd) == false) break;
            command.print(millis(), DEC);
            command.println(CMD_TERMINATOR);
            break;

        case SCAN_START:
            if (assert_argc(1 + 0, cmd) == false) break;
            scan.start();
            command.respond(true);
            break;

        case SCAN_STOP:
            if (assert_argc(1 + 0, cmd) == false) break;
            scan.stop();
            command.respond(true);
            break;

        case GET_SCAN:
            if (assert_argc(1 + 0, cmd) == false) break;

            static int16_t scan_array[SCAN_BUF_LEN];
            scan.get_scan(scan_array);

            for (int i = 0; i < SCAN_BUF_LEN; i++) {
                command.print(scan_array[i], DEC);
                if (i != SCAN_BUF_LEN - 1) command.print(",");
            }
            command.println(CMD_TERMINATOR);
            break;

        case RESET_PLATFORM:
            IWatchdog.begin(10000);
            while (true);
            break;

        case NO_COMMAND:
            /* display.dbg_print("I:no_command"); */ /* too noisy */
            break;

        default:
            display.dbg_print("E:unhandled cmd enum");
            command.respond(false);
            break;
    }

    scan.work();

    if (millis() > ping_watchdog + PING_TIMEOUT_MS) {
        drive.detach();
        scan.stop();
        display.dbg_print("E:ping timeout");
        display.dbg_beep(500,3);
    }
}
