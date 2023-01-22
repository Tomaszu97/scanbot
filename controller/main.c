#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <SDL2/SDL.h>
#include <sys/ioctl.h>

static int g_serial_port;
static char g_send_buf[4096] = {0};
static char g_recv_buf[4096] = {0};
static int speed = 0;
static int turn = 0;

#define CONTROLLER_ID 0

SDL_GameController *g_controller = NULL;


void
open_serial(const char *serial_filename)
{
    g_serial_port = open(serial_filename, O_RDWR);
    struct termios tty;

    if(tcgetattr(g_serial_port, &tty) != 0) {
        fprintf(stderr, "error %i from tcgetattr: %s\n", errno, strerror(errno));
        exit(-1);
    }

    /* tty flags*/
    tty.c_cflag &= ~PARENB;                                             /* clear parity bit */
    tty.c_cflag &= ~CSTOPB;                                             /* one stop bit */
    tty.c_cflag &= ~CSIZE;                                              /* clear all bits that set the data size */
    tty.c_cflag |= CS8;                                                 /* 8 bits per byte */
    tty.c_cflag &= ~CRTSCTS;                                            /* disable rts/cts */
    tty.c_cflag |= CREAD | CLOCAL;                                      /* turn on read & ignore ctrl lines */
    tty.c_lflag |= ICANON;
    tty.c_lflag &= ~ECHO;                                               /* disable echo */
    tty.c_lflag &= ~ECHOE;                                              /* disable erasure */
    tty.c_lflag &= ~ECHONL;                                             /* disable newline echo */
    tty.c_lflag &= ~ISIG;                                               /* disable interpretation of INTR, QUIT and SUSP */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                             /* turn off s/w flow ctrl */
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);    /* disable any special handling of received bytes */
    tty.c_oflag &= ~OPOST;                                              /* prevent special interpretation of output bytes */
    tty.c_oflag &= ~ONLCR;                                              /* prevent conversion of LF to CRLF */
    tty.c_cc[VTIME] = 10;                                               /* 10 = 1s */
    tty.c_cc[VMIN] = 0;

    /* in & out baud rate*/
    cfsetispeed(&tty, B38400);
    cfsetospeed(&tty, B38400);

    if (tcsetattr(g_serial_port, TCSANOW, &tty) != 0) {
        fprintf(stderr, "error %i from tcsetattr: %s\n", errno, strerror(errno));
        exit(-1);
    }
}

void
close_serial()
{
    close(g_serial_port);
}

void
init_controller()
{
    int result = SDL_Init(SDL_INIT_GAMECONTROLLER);
    if (result < 0 ) {
        fprintf(stderr, "sdl controller initialization failed\n");
        exit(-1);
    }
    if (SDL_NumJoysticks() < 1) {
        fprintf(stderr, "no controllers detected\n");
        exit(-1);
    }

    g_controller = SDL_GameControllerOpen(CONTROLLER_ID);
    if (g_controller == NULL) {
        fprintf(stderr, "could not connect controller\n");
        exit(-1);
    }
}

void
deinit_controller()
{
    SDL_GameControllerClose(g_controller);
    SDL_Quit();
}

int
multi_map(int val,
          int *in,
          int *out,
          uint8_t size)
{
    if (val <= in[0])
        return out[0];
    if (val >= in[size - 1])
        return out[size - 1];

    uint8_t pos = 1;
    while (val > in[pos])
        pos++;

    if (val == in[pos])
        return out[pos];

    return (val - in[pos - 1]) * (out[pos] - out[pos - 1]) / (in[pos] - in[pos - 1]) + out[pos - 1];
}

bool
recv_cmd()
{
    int num_bytes = read(g_serial_port, &g_recv_buf, sizeof(g_recv_buf) - 1);
    if (num_bytes < 0) {
        fprintf(stderr, "error reading: %s\n", strerror(errno));
        return false;
    }
    g_recv_buf[num_bytes] = '\0';

    printf("recv: %s\n", g_recv_buf);
    return true;
}

void
serial_flush()
{
    tcflush(g_serial_port, TCIOFLUSH);
}

bool
send_cmd()
{
    printf("send: %s\n",g_send_buf);
    int retval = write(g_serial_port, g_send_buf, strlen(g_send_buf));
    if (retval < 0 ) return false;
    return true;
}

void
send_drive_cmd()
{
    sprintf(g_send_buf, "DV:%d,%d#\n", speed, turn);
    send_cmd();
    recv_cmd();
}

void
update_speed(int gas_pedal, int brake_pedal)
{
    int in[] = {0, 32767};
    int out[] = {0, 390};
    speed = multi_map(gas_pedal, in, out, 2)
          - multi_map(brake_pedal, in, out, 2);
    if (speed%5==0) send_drive_cmd();
}

void
update_turn(int wheel)
{
    int in[] = {-32768, 0, 32767};
    int out[] = {-390, 0, 390};
    turn = multi_map(wheel, in, out, 3);
    if (turn%5==0) send_drive_cmd();
}

int
main(int argc, char *argv[])
{
    /* arg1 is serial port filename */
    if (argc != 2) {
        fprintf(stderr, "provide serial port as first argument\n");
        return EXIT_FAILURE;
    }

    init_controller();

    const char *serial_filename = argv[1];
    open_serial(serial_filename);

    int gas_pedal = 0;
    int brake_pedal = 0;
    int wheel = 0;

    while (1) {
        SDL_Event e;
        while(SDL_PollEvent(&e) != 0) {
            switch(e.type) {
                case SDL_QUIT:
                    close_serial();
                    deinit_controller();
                    SDL_Quit();
                    break;

                case SDL_CONTROLLERAXISMOTION:
                    if (e.caxis.which != CONTROLLER_ID) break;

                    if (e.caxis.axis == SDL_CONTROLLER_AXIS_LEFTX) {
                        wheel = e.caxis.value;
                        update_turn(wheel);
                    }
                    if (e.caxis.axis == SDL_CONTROLLER_AXIS_TRIGGERRIGHT) {
                        gas_pedal = e.caxis.value;
                        update_speed(gas_pedal, brake_pedal);
                    }
                    if (e.caxis.axis == SDL_CONTROLLER_AXIS_TRIGGERLEFT) {
                        brake_pedal = e.caxis.value;
                        update_speed(gas_pedal, brake_pedal);
                    }
                    break;

                case SDL_CONTROLLERBUTTONDOWN:
                    if (e.cbutton.which != CONTROLLER_ID) break;

                    if (e.cbutton.button == SDL_CONTROLLER_BUTTON_A) {
                        serial_flush();
                        sprintf(g_send_buf, "RS#\n");
                        send_cmd();
                    }
                    if (e.cbutton.button == SDL_CONTROLLER_BUTTON_B) {
                        serial_flush();
                        sprintf(g_send_buf, "BE:10,1#\n");
                        send_cmd();
                    }
                    if (e.cbutton.button == SDL_CONTROLLER_BUTTON_X) {
                        serial_flush();
                        sprintf(g_send_buf, "SC#\n");
                        send_cmd();
                    }
                    if (e.cbutton.button == SDL_CONTROLLER_BUTTON_Y) {
                        serial_flush();
                        sprintf(g_send_buf, "SS#\n");
                        send_cmd();
                    }
                    if (e.cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_UP) {
                        serial_flush();
                        sprintf(g_send_buf, "GT#\n");
                        send_cmd();
                        recv_cmd();
                    }
                    if (e.cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_LEFT) {
                        serial_flush();
                        sprintf(g_send_buf, "GS#\n");
                        send_cmd();
                        recv_cmd();
                    }
                    if (e.cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_DOWN) {
                        serial_flush();
                        sprintf(g_send_buf, "PG#\n");
                        send_cmd();
                        recv_cmd();
                    }
                    break;
            }
        }
    }

    return 0;
}
