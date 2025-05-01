#include "mbed.h"
#include "bme68x.h"

extern I2C i2c;

// BME680 I2C address (default 0x76 or 0x77 based on breakout)
#define BME68X_I2C_ADDR BME68X_I2C_ADDR_LOW

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    char reg = reg_addr;
    if (i2c.write(BME68X_I2C_ADDR << 1, &reg, 1, true) != 0)
        return -1;
    if (i2c.read(BME68X_I2C_ADDR << 1, (char *)data, len) != 0)
        return -1;
    return 0;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    char buffer[16];
    if (len + 1 > sizeof(buffer)) return -1;
    buffer[0] = reg_addr;
    memcpy(&buffer[1], data, len);
    if (i2c.write(BME68X_I2C_ADDR << 1, buffer, len + 1) != 0)
        return -1;
    return 0;
}

void user_delay_us(uint32_t period, void *) {
    wait_us(period);
}
