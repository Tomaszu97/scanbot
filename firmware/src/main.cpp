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
            command.print(drive.get_left_encoder_counter());
            command.print(CMD_PARAM_SEPARATOR_DELIMITER);
            command.print(drive.get_right_encoder_counter());
            command.println(CMD_TERMINATOR);
            drive.reset_encoder_counters();
            break;

        case DRIVE_RAW:
            if (assert_argc(1 + 2, cmd) == false) break;
            drive.set_motors(atoi(cmd.argv[1]),
                             atoi(cmd.argv[2]));
            command.print(drive.get_left_encoder_counter());
            command.print(CMD_PARAM_SEPARATOR_DELIMITER);
            command.print(drive.get_right_encoder_counter());
            command.println(CMD_TERMINATOR);
            drive.reset_encoder_counters();
            break;

        case KILL:
            if (assert_argc(1 + 0, cmd) == false) break;
            drive.detach();
            /* no response */
            break;

        case PRINT:
            if (assert_argc(1 + 1, cmd) == false) break;
            display.print(cmd.argv[1]);
            /* no response */
            break;

        case CLEAR:
            display.clear();
            /* no response */
            break;

        case BEEP:
            if (assert_argc(1 + 2, cmd) == false) break;
            display.beep(atoi(cmd.argv[1]), atoi(cmd.argv[2]));
            /* no response */
            break;

        case GET_TIME:
            if (assert_argc(1 + 0, cmd) == false) break;
            command.print(millis(), DEC);
            command.println(CMD_TERMINATOR);
            break;

        case SCAN_START:
            if (assert_argc(1 + 0, cmd) == false) break;
            scan.start();
            /* no response */
            break;

        case SCAN_STOP:
            if (assert_argc(1 + 0, cmd) == false) break;
            scan.stop();
            /* no response */
            break;

        case GET_SCAN:
            if (assert_argc(1 + 0, cmd) == false) break;

            static int16_t *scan_array = scan.scan_buf;
            static bool *scan_array_updated = scan.scan_buf_updated;

            for (int i = 0; i < SCAN_BUF_LEN; i++) {
                if (scan_array_updated[i] == true) {
                    command.print(scan_array[i], DEC);
                    scan_array_updated[i] = false;
                }
                if (i != SCAN_BUF_LEN - 1) command.print(",");
            }
            command.println(CMD_TERMINATOR);
            break;

        case RESET_PLATFORM:
            IWatchdog.begin(10000);
            while (true);
            /* no response */
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
