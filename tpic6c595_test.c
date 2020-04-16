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

int main(int argc, char **argv) {
    int fd = -1;
    uint8_t buf[1];
    struct timespec ts;

    /* 10ms between writes*/
    ts.tv_sec = 0;
    ts.tv_nsec = 10 * 1000 * 1000;

    /* open device */
    if ((fd = open("/dev/tpic6c59", O_RDWR)) < 0)
        fatal("failed to open device, err=%s", strerror(errno));

    /* write 0x00 - 0xff */
    for (int i = 0x00; i <= 0xff; i++) {
        buf[0] = i;
        info("writing 0x%02x\n", buf[0]);
        if (write(fd, buf, sizeof(buf)) < sizeof(buf))
            fatal("failed to write to device\n");
        nanosleep(&ts, NULL);
    }

    if (ioctl(fd, TPIC6C595_IOCTL_BTN, TPIC6C595_MODE_BTN_FLASH) != 0)
        fatal("failed ioctl\n");

    return EXIT_SUCCESS;
}