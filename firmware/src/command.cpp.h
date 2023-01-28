#include <Arduino.h>
#include <string.h>
#include "config.h"
#include "util.cpp.h"

typedef enum {
    NO_COMMAND,
    PING,
    DRIVE,
    DRIVE_RAW,
    KILL,
    PRINT,
    CLEAR,
    BEEP,
    GET_TIME,
    SCAN_START,
    SCAN_STOP,
    GET_SCAN,
    RESET_PLATFORM
} command_type_t;

typedef struct {
    command_type_t type;
    unsigned int argc;
    char *argv[CMD_MAX_ARG_N];
} command_t;

HardwareSerial Serial2(BLUETOOTH_SERIAL_RX, BLUETOOTH_SERIAL_TX);

class Command : public Print
{
public:
    void init();
    virtual size_t write(uint8_t c);
    command_t get_command();
    void respond(bool is_ok);

private:
    char cmd_buf[CMD_BUF_SIZE];
    command_type_t parse_command_type(const char *command);
    static const unsigned int command_type_strings_len = 13;
    char command_type_strings[command_type_strings_len][32] = {
        "NO_COMMAND",
        "PG", /*PING*/
        "DV", /*DRIVE*/
        "DR", /*DRIVE RAW*/
        "KL", /*KILL*/
        "PR", /*PRINT*/
        "CL", /*CLEAR*/
        "BE", /*BEEP*/
        "GT", /*GET_TIME*/
        "SC", /*SCAN START*/
        "SS", /*SCAN STOP*/
        "GS", /*GET SCAN*/
        "RS", /*RESET PLATFORM*/
    };
};

Command command;

void
Command::init()
{
    Serial2.begin(COMMAND_SERIAL_BAUD);
    respond(true);
}

size_t
Command::write(uint8_t c)
{
    return Serial2.write(c);
}

void
Command::respond(bool is_ok) {
    if (is_ok == true)
        Serial2.println("OK#");
    else
        Serial2.println("ERR#");
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
        display.dbg_print("E:len=0 or tmout");
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
            display.dbg_print("E:>max arg number");
            command.type = NO_COMMAND;
            return command;
        }

        command.argv[command.argc] = strtok(NULL, delims);
    }

    if (command.argc == 0) {
        display.dbg_print("E:bad cmd or args");
        command.type = NO_COMMAND;
        return command;
    }

    command.type = parse_command_type(command.argv[0]);

    if (command.type == NO_COMMAND) {
        display.dbg_print("E:unknown cmd");
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
    display.dbg_print(msgbuf);

    for (unsigned int i=1; i<command_type_strings_len; i++)
    {
        /* shortcut method mapping int to enum */
        if (strcmp(command_type_str, command_type_strings[i]) == 0)
            return (command_type_t)i;
    }
    return NO_COMMAND;
}
