#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**  
 * Initialize the I2C bus for the MLX90640  
 */
void    MLX90640_I2CInit(void);

/**  
 * Set bus frequency (in kHz)  
 */
void    MLX90640_I2CFreqSet(int freq);

/**  
 * Read `count` words starting at `startAddress`.  
 * Fills `data[]` (length `count`).  
 * Returns 0 on success, <0 on failure.  
 */
int     MLX90640_I2CRead(uint8_t slaveAddr,
                         uint16_t startAddress,
                         uint16_t count,
                         uint16_t *data);

/**  
 * Write one 16-bit `data` to `writeAddress`, verify by read.  
 * Returns 0 on success, <0 on failure.  
 */
int     MLX90640_I2CWrite(uint8_t slaveAddr,
                          uint16_t writeAddress,
                          uint16_t data);

/**  
 * Issue a general I2C reset (0x06 to address 0x00).  
 */
int     MLX90640_I2CGeneralReset(void);

#ifdef __cplusplus
}
#endif
