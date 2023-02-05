#include <Arduino.h>
#include <string.h>
#include "config.h"
#include "util.h"
#include "display.h"
#include "command.h"

HardwareSerial Serial2(BLUETOOTH_SERIAL_RX, BLUETOOTH_SERIAL_TX);

Command *Command::command_;

Command *
Command::get_instance()
{
    if (Command::command_ == nullptr)
        Command::command_ = new Command();
    return Command::command_;
}

Command::Command()
{
    display = Display::get_instance();
    Serial2.begin(COMMAND_SERIAL_BAUD);
    display->print("command init ok");
}

size_t
Command::write(uint8_t c)
{
    return Serial2.write(c);
}

command_t
Command::get_command()
{
    command_t command = { .type = NO_COMMAND };

    /* check if command available */
    if (Serial2.available() == false) {
        command.type = NO_COMMAND;
        return command;
    }

    /* ignore CR and LF */
    const char newchar = Serial2.peek();
    if (newchar == '\n' || newchar == '\r') {
        Serial2.read();
        return command;
    }

    /* read command */
    int rlen = Serial2.readBytesUntil(CMD_TERMINATOR, cmd_buf, CMD_BUF_SIZE-1);
    cmd_buf[rlen] = '\0';
    if (rlen == 0) {
        display->print("E:len=0 or tmout", true);
        command.type = NO_COMMAND;
        return command;
    }

    /* trim command */
    remove_char(cmd_buf, ' ');
    remove_char(cmd_buf, '\t');
    remove_char(cmd_buf, '\r');
    remove_char(cmd_buf, '\n');
    remove_char(cmd_buf, '\v');
    remove_char(cmd_buf, '\f');
    rlen = strlen(cmd_buf);
    if (rlen == 0) {
        command.type = NO_COMMAND;
        return command;
    }

    /* fill in command type and argc/argv */
    const char *delims = CMD_DELIMITER CMD_PARAM_SEPARATOR_DELIMITER;
    command.argc = 0;
    command.argv[command.argc] = strtok(cmd_buf, delims);
    while (command.argv[command.argc] != NULL) {
        command.argc++;

        if (command.argc >= CMD_MAX_ARG_N) {
            display->print("E:>max arg number", true);
            command.type = NO_COMMAND;
            return command;
        }

        command.argv[command.argc] = strtok(NULL, delims);
    }

    if (command.argc == 0) {
        display->print("E:bad cmd or args", true);
        command.type = NO_COMMAND;
        return command;
    }

    command.type = parse_command_type(command.argv[0]);

    if (command.type == NO_COMMAND) {
        display->print("E:unknown cmd", true);
    }

    return command;
}

command_type_t
Command::parse_command_type(const char *command_type_str)
{
    char msgbuf[OLED_CHAR_WIDTH+1] = "I:cmd ";
    strncat(msgbuf,
            command_type_str,
            (OLED_CHAR_WIDTH+1)-strlen(msgbuf));
    msgbuf[OLED_CHAR_WIDTH] = '\0';
    display->print(msgbuf, true);

    for (unsigned int i = 1; i < COMMANDS_COUNT; i++) {
        /* shortcut method mapping int to enum */
        if (strcmp(command_type_str, command_type_strings[i]) == 0)
            return (command_type_t)i;
    }
    return NO_COMMAND;
}

bool
Command::assert_argc(const unsigned int argc,
                     const command_t command)
{
    if (command.argc == argc) return true;

    if(command.argc > 0) {
        display->print("E:incorrect argc", true);
    }
    else {
        display->print("E:no arguments", true);
    }
    return false;
}

