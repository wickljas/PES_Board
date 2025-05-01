// MLX90640_I2C_Driver.cpp
#include "mbed.h"
#include "MLX90640_I2C_Driver.h"

// Use the same bus as your other sensors
extern I2C i2c;

// Initialize I2C peripheral and lines
void MLX90640_I2CInit(void) {
    // Mbed’s I2C constructor already set things up
    // Optionally generate STOP to reset any prev. transactions
    char dummy = 0;
    i2c.write(0, &dummy, 0);
}

// Set bus speed: freq in KHz
void MLX90640_I2CFreqSet(int freq) {
    // Mbed doesn’t let you change freq on the fly, but you can:
    // re-init with new frequency in constructor
}

// Read `nMemAddressRead` words starting at `startAddress`
int MLX90640_I2CRead(uint8_t slaveAddr,
                     uint16_t startAddress,
                     uint16_t nMemAddressRead,
                     uint16_t *data) {
    uint8_t addrBuf[2] = { uint8_t(startAddress >> 8), uint8_t(startAddress) };
    if (i2c.write(slaveAddr<<1, (char*)addrBuf, 2, true)) return -1;
    // Read 2 bytes per word
    int bytes = nMemAddressRead * 2;
    if (i2c.read(slaveAddr<<1, (char*)data, bytes)) return -1;
    // Convert big-endian to host order
    for (int i=0; i<nMemAddressRead; i++)
        data[i] = (data[i]>>8) | (data[i]<<8);
    return 0;
}

// Write one word and verify
int MLX90640_I2CWrite(uint8_t slaveAddr,
                      uint16_t writeAddress,
                      uint16_t data16) {
    uint8_t buf[4] = {
        uint8_t(writeAddress >> 8), uint8_t(writeAddress),
        uint8_t(data16 >> 8),       uint8_t(data16)
    };
    if (i2c.write(slaveAddr<<1, (char*)buf, 4))         return -1;
    // Read back
    uint16_t verify;
    MLX90640_I2CRead(slaveAddr, writeAddress, 1, &verify);
    return (verify==data16) ? 0 : -2;
}

// General I2C reset
int MLX90640_I2CGeneralReset(void) {
    char cmd = 0x06;
    return i2c.write(0x00, &cmd, 1) ? -1 : 0;
}
