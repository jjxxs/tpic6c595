#include <fcntl.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <log.h>
#include <sys/ioctl.h>
#include "tpic6c595.h"

union uint16_to_8 {
    uint16_t value;
    uint8_t  parts[sizeof(uint16_t)];
};

int main(int argc, char **argv) {
    int fd = -1;
    uint8_t buf[2];
    struct timespec ts;

    /* 10ms between writes*/
    ts.tv_sec = 0;
    ts.tv_nsec = 10 * 1000 * 1000;

    /* open device */
    if ((fd = open("/dev/tpic6c59", O_RDWR)) < 0)
        fatal("failed to open device, err=%s", strerror(errno));

    /* set to 16-bit mode, e.g. two chained registers */
    if (ioctl(fd, TPIC6C595_IOCTL_DAISY_CHAIN, 2) != 0)
        fatal("failed ioctl daisy_chain\n");

    /* write 0x0000 - 0xffff */
    union uint16_to_8 tmp;
    for (int i = 0x0000; i <= 0xffff; i++) {
        tmp.value = i;
        info("writing %d = 0x%02x 0x%02x\n", i, tmp.parts[0], tmp.parts[1]);
        if (write(fd, tmp.parts, sizeof(tmp.parts)) < sizeof(tmp.parts))
            fatal("failed to write to device\n");
        //nanosleep(&ts, NULL);
    }

    /* set button to flash the leds when pressed */
    if (ioctl(fd, TPIC6C595_IOCTL_BTN, TPIC6C595_MODE_BTN_CLEAR) != 0)
        fatal("failed ioctl btn clear\n");

    /* set button to flash the leds when pressed */
    if (ioctl(fd, TPIC6C595_IOCTL_BTN, TPIC6C595_MODE_BTN_FLASH) != 0)
        fatal("failed ioctl btn flash\n");

    /* set button to nightrider the leds when pressed */
    if (ioctl(fd, TPIC6C595_IOCTL_BTN, TPIC6C595_MODE_BTN_NIGHTRIDER) != 0)
        fatal("failed ioctl btn flash\n");

    return EXIT_SUCCESS;
}