# pico_i2c_mem
Reading and writing to I2C memories with pico.

To read and write to a I2C memory or any I2C device that need register number before data is sent or received use:

    int i2c_write_mem_blocking(i2c_inst_t *i2c, uint32_t i2caddr, uint32_t addr, size_t addrsize, uint8_t *src, size_t len);
    int i2c_read_mem_blocking(i2c_inst_t *i2c, uint32_t i2caddr, uint32_t addr, size_t addrsize, uint8_t *dst, size_t len);`

The `addr` paramater is the desired address in the I2C memory.
The `addrsize`parameter is its length in bytes, the acceptable range is 1-4.

To use this in your pico project:
* copy the directory MemI2C as a subdirectory of the base project directory.
* add the following line to its CMakeLists.txt:<br>
`add_subdirectory( MemI2C )`
* use `#include "memI2C.h"` as needed.

