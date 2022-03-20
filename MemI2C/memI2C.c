/*
 * Copyright (c) 2022 Federico Zuccardi Merli.
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include <stdint.h>

#include "memI2C.h"

#include "hardware/i2c.h"

// Addresses of the form 000 0xxx or 111 1xxx are reserved. No slave should
// have these addresses.
static inline bool i2c_reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int i2c_write_mem_blocking(i2c_inst_t *i2c, uint32_t i2caddr, uint32_t addr, size_t addrsize, uint8_t *src, size_t len)
{
    /* Check that address is reasonably sized */
    invalid_params_if(I2C, addrsize > 4);

    invalid_params_if(I2C, i2caddr >= 0x80); // 7-bit addresses
    invalid_params_if(I2C, i2c_reserved_addr(i2caddr));
    // Synopsys hw accepts start/stop flags alongside data items in the same
    // FIFO word, so no 0 byte transfers.
    invalid_params_if(I2C, (len + addrsize) == 0);

    /* Reverse the address, as I2C EEPROMs want it in Big Endian form */
    addr = __builtin_bswap32(addr);
    /* Get the address of the MSB */
    const uint8_t *p = (const uint8_t *)&addr + sizeof(addr) - addrsize;

    i2c->hw->enable = 0;
    i2c->hw->tar    = i2caddr;
    i2c->hw->enable = 1;

    bool abort = false;

    uint32_t abort_reason = 0;
    int      byte_ctr;

    int ilen = (int)(len + addrsize);
    for (byte_ctr = 0; byte_ctr < ilen; ++byte_ctr)
    {
        bool first = byte_ctr == 0;
        bool last  = byte_ctr == ilen - 1;

        i2c->hw->data_cmd =
            bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
            bool_to_bit(last) << I2C_IC_DATA_CMD_STOP_LSB |
            (byte_ctr < addrsize ? *p++ : *src++);

        // Wait until the transmission of the address/data from the internal
        // shift register has completed. For this to function correctly, the
        // TX_EMPTY_CTRL flag in IC_CON must be set. The TX_EMPTY_CTRL flag
        // was set in i2c_init.
        do
        {
            tight_loop_contents();
        } while (!(i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS));

        abort_reason = i2c->hw->tx_abrt_source;
        if (abort_reason)
        {
            // Note clearing the abort flag also clears the reason, and
            // this instance of flag is clear-on-read! Note also the
            // IC_CLR_TX_ABRT register always reads as 0.
            i2c->hw->clr_tx_abrt;
            abort = true;
        }

        if (abort || last)
        {
            // If the transaction was aborted or if it completed
            // successfully wait until the STOP condition has occured.
            do
            {
                tight_loop_contents();
            } while (!(i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS));

            i2c->hw->clr_stop_det;
        }

        // Note the hardware issues a STOP automatically on an abort condition.
        // Note also the hardware clears RX FIFO as well as TX on abort,
        // because we set hwparam IC_AVOID_RX_FIFO_FLUSH_ON_TX_ABRT to 0.
        if (abort)
            break;
    }

    int rval;

    // A lot of things could have just happened due to the ingenious and
    // creative design of I2C. Try to figure things out.
    if (abort)
    {
        if (!abort_reason || abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS)
        {
            // No reported errors - seems to happen if there is nothing connected to the bus.
            // Address byte not acknowledged
            rval = PICO_ERROR_GENERIC;
        }
        else if (abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS)
        {
            // Address acknowledged, some data not acknowledged
            rval = byte_ctr;
        }
        else
        {
            // panic("Unknown abort from I2C instance @%08x: %08x\n", (uint32_t) i2c->hw, abort_reason);
            rval = PICO_ERROR_GENERIC;
        }
    }
    else
    {
        rval = byte_ctr;
    }

    // nostop means we are now at the end of a *message* but not the end of a *transfer*
    i2c->restart_on_next = false;
    return rval;
}

int i2c_read_mem_blocking(i2c_inst_t *i2c, uint32_t i2caddr, uint32_t addr, size_t addrsize, uint8_t *dst, size_t len)
{
    /* Check that address is reasonably sized */
    invalid_params_if(I2C, addrsize > 4);

    /* Reverse the address, as I2C EEPROM wnat it in Big Endian form */
    addr = __builtin_bswap32(addr);
    /* Get the address of the MSB */
    const uint8_t *p = (const uint8_t *)&addr + 4 - addrsize;
    /* Write register address, no I2C STOP */
    i2c_write_blocking(i2c, i2caddr, p, addrsize, true);
    /* Use repeated start to read back data */
    i2c_read_blocking(i2c, i2caddr, dst, len, false);
}
