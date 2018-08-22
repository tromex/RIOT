/*
* Copyright (C) TODO
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
*/

/**
 * @ingroup     cpu_sam3
 * @ingroup     drivers_periph_i2c
 * @{
 *
 * @file
 * @brief       Low-level I2C driver implementation
 *
 * @author      Sebastian Tromer <tromersebastian@gmail.com>
 *
 * @}
 */

#include "cpu.h"
#include "mutex.h"
#include "assert.h"
#include "periph/gpio.h"
#include "periph/i2c.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"


void i2c_init(i2c_t dev)
{
    (void) dev;
}

int i2c_acquire(i2c_t dev)
{
    (void) dev;
    return -1;
}

int i2c_release(i2c_t dev)
{
    (void) dev;
    return -1;
}

int i2c_read_reg(i2c_t dev, uint16_t addr, uint16_t reg,
                 void *data, uint8_t flags)
{
    (void) dev;
    (void) addr; (void) reg; (void) data; (void) flags;
    return -1;
}


int i2c_read_regs(i2c_t dev, uint16_t addr, uint16_t reg,
                  void *data, size_t len, uint8_t flags)
{
    (void) dev;
    (void) addr; (void) reg; (void) data; (void) len; (void) flags;
    return -1;
}

int i2c_read_byte(i2c_t dev, uint16_t addr, void *data, uint8_t flags)
{
    (void) dev;
    (void) addr; (void) data; (void) flags;
    return -1;
}

int i2c_read_bytes(i2c_t dev, uint16_t addr,
                   void *data, size_t len, uint8_t flags)
{
    (void) dev;
    (void) addr; (void) data; (void) len; (void) flags;
    return -1;
}


int i2c_write_byte(i2c_t dev, uint16_t addr, uint8_t data, uint8_t flags)
{
    (void) dev;
    (void) addr; (void) data; (void) flags;
    return -1;
}

int i2c_write_bytes(i2c_t dev, uint16_t addr, const void *data,
                    size_t len, uint8_t flags)
{
    (void) dev;
    (void) addr; (void) data; (void) len; (void) flags;
    return -1;
}

int i2c_write_reg(i2c_t dev, uint16_t addr, uint16_t reg,
                  uint8_t data, uint8_t flags)
{
    (void) dev;
    (void) addr; (void) reg; (void) data; (void) flags;
    return -1;
}

int i2c_write_regs(i2c_t dev, uint16_t addr, uint16_t reg,
                  const void *data, size_t len, uint8_t flags)
{
    (void) dev;
    (void) addr; (void) reg; (void) data; (void) len; (void) flags;
    return -1;
}

