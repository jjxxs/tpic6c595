/**
 * Simple test-program to demonstrate the tpic6c595-driver. Configures
 * the driver via ioctl() and writes all values from zero to maximum
 * possible value to the device.
 */
#include <fcntl.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <log.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include "tpic6c595.h"

/* size of the device in bytes, e.g. the used device can keep two bytes */
#define DEVICE_SIZE 2

struct timespec ts = {
   .tv_sec = 0,
   .tv_nsec = 10 * 1000 * 1000 /* 10ms */
};

/* helper union to easily split a u16 into two u8s */
union u16_to_u8s {
    uint16_t val;
    uint8_t  parts[sizeof(uint16_t)];
};

int main(int argc, char **argv) {
    int fd;

    /* open device */
    if ((fd = open("/dev/tpic6c59", O_RDWR)) < 0)
        fatal("failed to open device, err=%s", strerror(errno));

    /* set to 16-bit mode, e.g. two chained registers */
    if (ioctl(fd, TPIC6C595_IOCTL_SIZE, DEVICE_SIZE) != 0)
        fatal("failed to set size=%d\n", DEVICE_SIZE);

    /* write 0x0000 - 0xffff */
    union u16_to_u8s tmp;
    for (tmp.val = 0x0000; tmp.val <= 0xffff; tmp.val++) {
        info("writing %d = 0x%02x 0x%02x\n", tmp.val, tmp.parts[0], tmp.parts[1]);
        if (write(fd, tmp.parts, sizeof(tmp.parts)) < sizeof(tmp.parts))
            fatal("failed to write to device\n");
        nanosleep(&ts, NULL);
    }

    /* set button to clear the leds when pressed */
    if (ioctl(fd, TPIC6C595_IOCTL_BTN, TPIC6C595_MODE_BTN_CLEAR) != 0)
        fatal("failed ioctl btn clear\n");
    info("set button to clear LEDs when pressed, press any key to continue\n");
    getchar();

    /* set button to flash the leds when pressed */
    if (ioctl(fd, TPIC6C595_IOCTL_BTN, TPIC6C595_MODE_BTN_FLASH) != 0)
        fatal("failed ioctl btn flash\n");
    info("set button to flash LEDs when pressed, press any key to continue\n");
    getchar();

    /* set button to chase the leds when pressed */
    if (ioctl(fd, TPIC6C595_IOCTL_BTN, TPIC6C595_MODE_BTN_CHASER) != 0)
        fatal("failed ioctl btn chase\n");
    info("set button to chase LEDs when pressed, press any key to continue\n");
    getchar();

    return EXIT_SUCCESS;
}