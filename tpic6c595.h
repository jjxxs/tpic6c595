#ifndef TPIC6C595_TPIC6C595_H
#define TPIC6C595_TPIC6C595_H

/**
 * Configures the mode of the button, e.g. to any of TPIC6C595_MODE_BTN_*.
 */
#define TPIC6C595_IOCTL_BTN _IO(0xaa, 0x00)

/**
 * The button has no functionality.
 */
#define TPIC6C595_MODE_BTN_DISABLED 0

/**
 * Pressing the button results in chase of values, this gives a "knightrider"-effect.
 */
#define TPIC6C595_MODE_BTN_CHASER   1

/**
 * Pressing the button results in the device being flushed by zeroes and ones which
 * gives a blinking/flashing effect.
 */
#define TPIC6C595_MODE_BTN_FLASH    2

/**
 * Pressing the button clears the device by flushing it with zeroes.
 */
#define TPIC6C595_MODE_BTN_CLEAR    3

/**
 * Sets the write delay
 */
#define TPIC6C595_IOCTL_BITBANG_DELAY _IO(0xaa, 0x01)
#define TPIC6C595_IOCTL_EFFECT_DURATION _IO(0xaa, 0x02)
#define TPIC6C595_IOCTL_SIZE _IO(0xaa, 0x03)

#endif //TPIC6C595_TPIC6C595_H
