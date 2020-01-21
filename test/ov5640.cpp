// #include "OV5640.h"
// #include <iostream>

// using namespace std;

// const char *i2c_dev = "/dev/i2c-1";

// void _delay_ms(int n)
// {
//     usleep(n * 1000);
// }

// static int _i2c_read(int fd, int device_addr, u16 regAddr, u8 *readValue)
// {
//     int ret;
//     u8 buf[2];
//     u8 out[2] = {0, 0};
//     struct i2c_rdwr_ioctl_data i2c_data;
//     struct i2c_msg msg[2];

//     buf[0] = regAddr >> 8;
//     buf[1] = regAddr & 0xff;

//     ioctl(fd, I2C_TIMEOUT, 1); // 10 ms (timeout in units of 10 ms)
//     ioctl(fd, I2C_RETRIES, 2); // twice (number of times a device address should be polled when not acknowledging)

//     msg[0].addr = device_addr;
//     msg[0].flags = 0; // write 0, read 1
//     msg[0].buf = buf;
//     msg[0].len = sizeof(buf);

//     msg[1].addr = device_addr;
//     msg[1].flags = I2C_M_RD;
//     msg[1].buf = out;
//     msg[1].len = 1;

//     i2c_data.nmsgs = 2;
//     i2c_data.msgs = msg;
//     ret = ioctl(fd, I2C_RDWR, (unsigned long)&i2c_data);
//     _delay_ms(100);
//     i2c_data.nmsgs = 1;
//     i2c_data.msgs = &msg[1];
//     ret = ioctl(fd, I2C_RDWR, (unsigned long)&i2c_data);
//     printf("------1--------\n");
//     printf("i2c read 0x%02x: ", regAddr);
//     printf("0x%02x%02x\n", out[1], out[0]);

//     if (ret < 0)
//     {
//         printf("read data 0x%02x 0x%02x error\n", device_addr, regAddr);
//         return -1;
//     }
//     *readValue = buf[0];
//     printf("-------2------\n");
//     printf("i2c read 0x%02x: ", regAddr);
//     printf("0x%02x%02x\n", out[1], out[0]);
//     return ret;
// }

// static int _i2c_write(int fd, int device_addr, u16 regAddr, u8 val)
// {
//     int ret;
//     struct i2c_rdwr_ioctl_data i2c_data;
//     struct i2c_msg msg;
//     u8 buf[3];

//     buf[0] = regAddr >> 8;
//     buf[1] = regAddr & 0xff;
//     buf[2] = val;

//     ioctl(fd, I2C_TIMEOUT, 1);
//     ioctl(fd, I2C_RETRIES, 2);

//     msg.addr = device_addr;
//     msg.flags = 0;
//     msg.buf = buf;
//     msg.len = sizeof(buf);
//     i2c_data.nmsgs = 1;
//     i2c_data.msgs = &msg;

//     ret = ioctl(fd, I2C_RDWR, &i2c_data);
//     if (ret < 0)
//     {
//         printf("write error\n");
//         return -1;
//     }
//     return ret;
// }

// static int i2c_read(int fd, u16 addr, u8 *val)
// {
//     int res;
//     // u8 send_buf[2];
//     // u8 read_buf[2] = {0, 0};
//     // send_buf[0] = addr >> 8;
//     // send_buf[1] = addr & 0xff;
//     u8 buf[2];
//     buf[0] = addr >> 8;
//     buf[1] = addr & 0xff;

//     write(fd, buf, sizeof(buf));
//     _delay_ms(1000);
//     res = read(fd, buf, sizeof(buf));
//     if (res < 0)
//     {
//         printf("read error\n");
//         return -1;
//     }
//     *val = buf[0];
//     printf("read at 0x%x value: 0x%02x%02x\n", addr, buf[1], buf[0]);

//     return res;
// }

// static int i2c_write(int fd, u16 addr, u8 val)
// {
//     int res;
//     u8 buf[3];

//     buf[0] = addr >> 8;
//     buf[1] = addr & 0xFF;
//     buf[2] = val;

//     res = write(fd, buf, sizeof(buf));
//     if (res < 0)
//     {
//         printf("write error\n");
//         return -1;
//     }
//     printf("write 0x%02x at 0x%02x\n", val, addr);
//     return res;
// }

// int main()
// {
//     cout << "test ov5640 i2c\n";
//     int fd, ret;
//     u8 buf[2];
//     u8 slave_addr = 0xF0 >> 1;

//     fd = open(i2c_dev, O_RDWR);
//     if (fd < 0)
//     {
//         printf("no %s\n", i2c_dev);
//         return -1;
//     }

//     ret = ioctl(fd, I2C_TENBIT, 0);
//     ret = ioctl(fd, I2C_SLAVE, slave_addr);

//     // brightness +4
//     // i2c_write(fd, 0x3212, 0x03);
//     // i2c_write(fd, 0x5587, 0x40);
//     // i2c_write(fd, 0x5588, 0x01);
//     // i2c_write(fd, 0x3212, 0x13);
//     // i2c_write(fd, 0x3212, 0xa3);

//     // _i2c_write(fd, slave_addr, 0x3212, 0x03);
//     // _i2c_write(fd, slave_addr, 0x5587, 0x40);
//     // _i2c_write(fd, slave_addr, 0x5588, 0x01);
//     // _i2c_write(fd, slave_addr, 0x3212, 0x13);
//     // _i2c_write(fd, slave_addr, 0x3212, 0xa3);

//     u8 reg_v = 0;
//     // ret = i2c_read(fd, 0x5587, &reg_v);
//     ret = _i2c_read(fd, slave_addr, 0x5587, &reg_v);
//     printf("reg value: 0x%02x\n", reg_v);

//     return 0;
// }

#include "ov5640.h"

/*
 * selector    size    Minimum    Maximum    Meaning
 *     1         2     0          255        unknown
 *     2         2     0          255        unknown
 *     3         2     22         22         Version of the firmware. Read-only
 *     7        20                           unknown
 *     8         2     0          65526      Average luminance
 *     9        17                           Weight table for the auto exposure mechanism. See below for the meaning of the 9 bytes
 *    10         1     1          1          Enable a test pattern 
 *    11         6                           unknown
 *    12         2     1          1          Horizontal flip
 *    13         2     1          1          Vertical flip
 *    14         5                           Interface to directly access camera registers. See below for the meaning of the 5 bytes.
 */
int set_uvc_xu(int device_id, uint8_t extension_uint_id, uint8_t control_selector, uint8_t queries, uint16_t size, uint8_t *data)
{
    struct uvc_xu_control_query query;
    query.unit = extension_uint_id;
    query.selector = control_selector;
    query.query = queries;
    query.size = size;
    query.data = data;

    return ioctl(device_id, UVCIOC_CTRL_QUERY, &query);
}

int setFlipControlSetting(int fd, bool flip)
{
    union {
        uint16_t value;
        uint8_t data[2];
    };
    if (flip)
    {
        value = 1;
    }
    else
    {
        value = 0;
    }
    int v = set_uvc_xu(fd, EXTENSION_UNIT_ID, SELECTOR_13_VERTICAL_FLIP, UVC_SET_CUR, SELECTOR_13_SIZE, data);   // Vertical flip
    int h = set_uvc_xu(fd, EXTENSION_UNIT_ID, SELECTOR_12_HORIZONTAL_FLIP, UVC_SET_CUR, SELECTOR_12_SIZE, data); // Horizontal flip
    if ((v < 0) || (h < 0))
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

// if you want to read a register value, you must set this register value first.
uint16_t getRegisterValue(int device_id, uint16_t reg_addr)
{
    uint8_t upper_reg_addr = (reg_addr >> 8) & 0xff;
    uint8_t lower_reg_addr = reg_addr & 0xff;
    uint8_t data[5] = {0, 0, 0, 0, 0};
    struct uvc_xu_control_query query;
    int ret;

    data[READ_WRITE_BYTE] = READ;
    data[UPPER_REG_ADDR_BYTE] = upper_reg_addr;
    data[LOWER_REG_ADDR_BYTE] = lower_reg_addr;
    data[UPPER_REG_VAL_BYTE] = 0x00;
    data[LOWER_REG_VAL_BYTE] = 0x00;

    query.unit = EXTENSION_UNIT_ID;
    query.selector = SELECTOR_14_ACCESS_CAMERA_REGISTER;
    query.query = UVC_GET_CUR;
    query.size = SELECTOR_14_SIZE;
    query.data = data;
    ret = ioctl(device_id, UVCIOC_CTRL_QUERY, &query);

    printf("register address: 0x%02x%02x  read value: 0x%02x\n", data[UPPER_REG_ADDR_BYTE], data[LOWER_REG_ADDR_BYTE], data[LOWER_REG_ADDR_BYTE]);

    uint16_t re = (data[3] << 8) + data[4];
    return re;
}

int setRegisterValue(int device_id, uint8_t upper_reg_addr, uint8_t lower_reg_addr, uint8_t upper_reg_v, uint8_t lower_reg_v)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};
    struct uvc_xu_control_query query;
    int ret;

    data[READ_WRITE_BYTE] = WRITE;
    data[UPPER_REG_ADDR_BYTE] = upper_reg_addr;
    data[LOWER_REG_ADDR_BYTE] = lower_reg_addr;
    data[UPPER_REG_VAL_BYTE] = upper_reg_v;
    data[LOWER_REG_VAL_BYTE] = lower_reg_v;

    query.unit = EXTENSION_UNIT_ID;
    query.selector = SELECTOR_14_ACCESS_CAMERA_REGISTER;
    query.query = UVC_SET_CUR;
    query.size = SELECTOR_14_SIZE;
    query.data = data;
    ret = ioctl(device_id, UVCIOC_CTRL_QUERY, &query);

    if (ret < 0)
    {
        printf("%s line %d falied setting register: 0x%02x%02x\n", __FILE__, __LINE__, data[UPPER_REG_ADDR_BYTE], data[LOWER_REG_ADDR_BYTE]);
    }

    return ret;
}

/*
 * This is the light frequency selection control function.
 * In OV5640 datasheet page 52, section 4.8, we can get the 
 * light frequency registers and its description
 * datasheet url: https://cdn.sparkfun.com/datasheets/Sensors/LightImaging/OV5640_datasheet.pdf
 */
void lightFrequencySelection(int device_id, BHCameraSettings::PowerLineFrequency powerLineFrequency)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};
    struct uvc_xu_control_query query;
    int ret;
    if (powerLineFrequency == BHCameraSettings::_Auto)
    {
        // set 0x3C01 to Auto
        setRegisterValue(device_id, 0x3C, 0x01, 0x00, 0x00);
    }
    else if (powerLineFrequency == BHCameraSettings::_50Hz)
    {
        setRegisterValue(device_id, 0x3C, 0x01, 0x00, 1 << 7);
        setRegisterValue(device_id, 0x3C, 0x00, 0x00, 1 << 2); // 50 Hz
    }
    else if (powerLineFrequency == BHCameraSettings::_60Hz)
    {
        setRegisterValue(device_id, 0x3C, 0x01, 0x00, 1 << 7);
        setRegisterValue(device_id, 0x3C, 0x00, 0x00, 0x00); // 60 Hz
    }
    else
    {
        printf("In file: %s, line: %d, void lightFrequencySelection(int device_id, BHCameraSettings::PowerLineFrequency powerLineFrequency)"
               " error parameter of powerLineFrequency, _Auto, _50Hz and _60Hz only.\n",
               __FILE__, __LINE__);
    }
}

void ov5640_test(int device_id)
{
    getRegisterValue(device_id, 0x5587);
}