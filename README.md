# pico_i2c_mem
Reading and writing to I2C memories with pico.

To read and write to a I2C memory or any I2C device that need register number before data is sent or received use:

    int i2c_write_mem_blocking(i2c_inst_t *i2c, uint32_t i2caddr, uint32_t addr, size_t addrsize, uint8_t *src, size_t len);
    int i2c_read_mem_blocking(i2c_inst_t *i2c, uint32_t i2caddr, uint32_t addr, size_t addrsize, uint8_t *dst, size_t len);`

The `addr` paramater is the desired address in the I2C memory.
The `addrsize`parameter is its length in bytes, the acceptable range is 1-4.

The other parameters are as in the `i2c_write_blocking()`.
The return value is either the number of bytes written to or read from the memory, not including address bytes, or PICO_GENERIC_ERROR in case of errors, including NACK while writing the memory address.

To use this in your pico project:
* copy this directory (MemI2C) as a subdirectory of the base project directory or add it as a git submodule
* add the following line to the main CMakeLists.txt, after the `add_executable(...)` statement:<br>
`add_subdirectory( MemI2C )`
* At least version 3.13 of CMake is needed, make sure the main CMakeLists.txt begins with a<br>
`cmake_minimum_required(VERSION 3.13)` (or later) line to correctly set policies.
* use `#include "memI2C.h"` as needed.
