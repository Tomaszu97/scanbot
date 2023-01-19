#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>

int serial_port = -1;
char send_buf[4096] = {0};
char recv_buf[4096] = {0};

int
multi_map(int val,
          int *in,
          int *out,
          uint8_t size)
{
    /* "in" array should be sorted increasingly */
    /* both arrays should be the same size */

    /* constrain value between boundaries */
    if (val <= in[0])
        return out[0];
    if (val >= in[size - 1])
        return out[size - 1];

    /* search right interval */
    uint8_t pos = 1; // in[0] allready tested
    while (val > in[pos])
        pos++;

    /* this will handle all exact "points" in the in array */
    if (val == in[pos])
        return out[pos];

    /* interpolate in the right segment for the rest */
    return (val - in[pos - 1]) * (out[pos] - out[pos - 1]) / (in[pos] - in[pos - 1]) + out[pos - 1];
}

void
open_serial(const char* serial_filename)
{
    serial_port = open(serial_filename, O_RDWR);
    struct termios tty;

    if(tcgetattr(serial_port, &tty) != 0) {
        ROS_ERROR("error %i from tcgetattr: %s\n", errno, strerror(errno));
        exit(-1);
    }

    /* tty flags */
    tty.c_cflag &= ~PARENB;                                          /* clear parity bit */
    tty.c_cflag &= ~CSTOPB;                                          /* one stop bit */
    tty.c_cflag &= ~CSIZE;                                           /* clear all bits that set the data size */
    tty.c_cflag |= CS8;                                              /* 8 bits per byte */
    tty.c_cflag &= ~CRTSCTS;                                         /* disable rts/cts */
    tty.c_cflag |= CREAD | CLOCAL;                                   /* turn on read & ignore ctrl lines */
    tty.c_lflag |= ICANON;
    tty.c_lflag &= ~ECHO;                                            /* disable echo */
    tty.c_lflag &= ~ECHOE;                                           /* disable erasure */
    tty.c_lflag &= ~ECHONL;                                          /* disable newline echo */
    tty.c_lflag &= ~ISIG;                                            /* disable interpretation of INTR, QUIT and SUSP */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                          /* turn off s/w flow ctrl */
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); /* disable any special handling of received bytes */
    tty.c_oflag &= ~OPOST;                                           /* prevent special interpretation of output bytes */
    tty.c_oflag &= ~ONLCR;                                           /* prevent conversion of LF to CRLF */
    tty.c_cc[VTIME] = 10;                                            /* 10 = 1s */
    tty.c_cc[VMIN] = 0;

    /* in & out baud rate */
    cfsetispeed(&tty, B38400);
    cfsetospeed(&tty, B38400);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        ROS_ERROR("error %i from tcsetattr: %s\n", errno, strerror(errno));
        exit(-1);
    }
}

void
close_serial()
{
    close(serial_port);
}

bool
recv_cmd()
{
    int num_bytes = read(serial_port, &recv_buf, sizeof(recv_buf) - 1);
    if (num_bytes < 0) {
        ROS_ERROR("error reading: %s", strerror(errno));
        return false;
    }
    recv_buf[num_bytes] = '\0';

    ROS_INFO("recv: %s", recv_buf);
    return true;
}

void
flush_serial()
{
    tcflush(serial_port, TCIOFLUSH);
}

bool
send_cmd()
{
    ROS_INFO("send: %s",send_buf);
    int retval = write(serial_port, send_buf, strlen(send_buf));
    if (retval < 0 ) return false;
    return true;
}
