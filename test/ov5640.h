// #pragma once

// #include <linux/types.h>
// #include <sys/ioctl.h>
// #include <stddef.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <sys/ioctl.h>
// #include <errno.h>
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
// #include <unistd.h>
// #include <stdint.h>
// #include <linux/i2c-dev.h>
// #include <linux/i2c.h>

// typedef uint16_t u16;
// typedef uint8_t u8;
// typedef uint32_t u32;

// int ov5640_write_reg(int device_id, u16 regAddr, u8 val);

// int ov5640_read_reg(int device_id, u16 regAddr, u8 *val);

// static int _i2c_read(int device_id, u16 regAddr, u8 &readValue);

/* UVC extension */
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <stdio.h>
// #include "BHNaoCamera.h"
#include <stdint.h>

#define EXTENSION_UNIT_ID 3

#define SELECTOR_3_VERSION 3
#define SELECTOR_8_AVERAGE_LUMINANCE 8
#define SELECTOR_9_EXPOSURE_WEIGHT_TABLE 9
#define SELECTOR_12_HORIZONTAL_FLIP 12
#define SELECTOR_13_VERTICAL_FLIP 13
#define SELECTOR_14_ACCESS_CAMERA_REGISTER 14

#define SELECTOR_3_SIZE 2
#define SELECTOR_8_SIZE 2
#define SELECTOR_9_SIZE 17
#define SELECTOR_12_SIZE 2
#define SELECTOR_13_SIZE 2
#define SELECTOR_14_SIZE 5

#define READ 0
#define WRITE 1
#define READ_WRITE_BYTE 0
#define UPPER_REG_ADDR_BYTE 1
#define LOWER_REG_ADDR_BYTE 2
#define UPPER_REG_VAL_BYTE 3
#define LOWER_REG_VAL_BYTE 4

namespace BHCameraSettings
{
enum PowerLineFrequency
{
    _50Hz,
    _60Hz,
    _Auto
};
}

int set_uvc_xu(int device_id, uint8_t extension_uint_id, uint8_t control_selector, uint8_t queries, uint16_t size, uint8_t *data);

int setFlipControlSetting(int fd, bool flip);

uint16_t getRegisterValue(int device_id, int8_t upper_reg_addr, uint8_t lower_reg_addr);

int setRegisterValue(int device_id, uint8_t upper_reg_addr, uint8_t lower_reg_addr, uint8_t upper_reg_v, uint8_t lower_reg_v);

void lightFrequencySelection(int device_id, BHCameraSettings::PowerLineFrequency powerLineFrequency);

void ov5640_test(int device_id);