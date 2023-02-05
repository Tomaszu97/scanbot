#ifndef COMMAND_H
#define COMMAND_H

#include <Arduino.h>
#include <string.h>
#include "config.h"
#include "util.h"

#define MAX_COMMAND_STR_LEN 32

#define COMMANDS(cmd_transform_fn) \
  cmd_transform_fn( NO_COMMAND,      NC ) \
  cmd_transform_fn( PING,            PG ) \
  cmd_transform_fn( DRIVE,           DV ) \
  cmd_transform_fn( DRIVE_RAW,       DR ) \
  cmd_transform_fn( KILL,            KL ) \
  cmd_transform_fn( PRINT,           PR ) \
  cmd_transform_fn( CLEAR,           CL ) \
  cmd_transform_fn( BEEP,            BE ) \
  cmd_transform_fn( SCAN_START,      SC ) \
  cmd_transform_fn( SCAN_STOP,       SS ) \
  cmd_transform_fn( SET_TOWER,       ST ) \
  cmd_transform_fn( RESET_PLATFORM,  RS )

#define COMMANDS_ENUM(cmd, abbrev) \
  cmd,

#define COMMANDS_STR(cmd, abbrev) \
  #abbrev,

typedef enum {
    COMMANDS(COMMANDS_ENUM)
    COMMANDS_COUNT
} command_type_t;

typedef struct {
    command_type_t type;
    unsigned int argc;
    char *argv[CMD_MAX_ARG_N];
} command_t;

class Command : public Print
{
private:
    Command();
    static Command *command_;
    Display *display;
    char cmd_buf[CMD_BUF_SIZE];
    command_type_t parse_command_type(const char *command);
    const char command_type_strings[COMMANDS_COUNT][MAX_COMMAND_STR_LEN + 1] = {
        COMMANDS(COMMANDS_STR)
    };

public:
    static Command *get_instance();
    command_t get_command();
    bool assert_argc(const unsigned int argc,
                     const command_t command);
    virtual size_t write(uint8_t c);
};

#endif /* COMMAND_H */
