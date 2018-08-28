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


/* Low level time limit of I2C Fast Mode. */
#define LOW_LEVEL_TIME_LIMIT 384000
#define TWI_CLK_DIVIDER      2
#define TWI_CLK_CALC_ARGU    4
#define TWI_CLK_DIV_MAX      0xFF
#define TWI_CLK_DIV_MIN      7


/**
 * @brief   Array holding one pre-initialized mutex for each I2C device
 */
static mutex_t locks[I2C_NUMOF];

static inline Twi *twi(i2c_t dev)
{
    return i2c_config[dev].dev;
}

static inline void twi_reset(Twi *p_twi)
{
    /* Set SWRST bit to reset TWI peripheral */
    p_twi->TWI_CR = TWI_CR_SWRST;
    p_twi->TWI_RHR;
}

static inline void twi_enable_master_mode(Twi *p_twi)
{
    /* Set Master Disable bit and Slave Disable bit */
    p_twi->TWI_CR = TWI_CR_MSDIS;
    p_twi->TWI_CR = TWI_CR_SVDIS;

    /* Set Master Enable bit */
    p_twi->TWI_CR = TWI_CR_MSEN;
}

static void i2c_set_speed(i2c_t dev, uint32_t ul_speed)
{
    uint32_t ckdiv = 0;
    uint32_t c_lh_div;
    uint32_t cldiv, chdiv;

    if (ul_speed > I2C_SPEED_FAST)
        return;

    /* Low level time not less than 1.3us of I2C Fast Mode. */
    if (ul_speed > LOW_LEVEL_TIME_LIMIT) {
        /* Low level of time fixed for 1.3us. */
        cldiv = CLOCK_CORECLOCK / (LOW_LEVEL_TIME_LIMIT * TWI_CLK_DIVIDER) - TWI_CLK_CALC_ARGU;
        chdiv = CLOCK_CORECLOCK / ((ul_speed + (ul_speed - LOW_LEVEL_TIME_LIMIT)) * TWI_CLK_DIVIDER) - TWI_CLK_CALC_ARGU;

        /* cldiv must fit in 8 bits, ckdiv must fit in 3 bits */
        while ((cldiv > TWI_CLK_DIV_MAX) && (ckdiv < TWI_CLK_DIV_MIN)) {
            /* Increase clock divider */
            ckdiv++;
            /* Divide cldiv value */
            cldiv /= TWI_CLK_DIVIDER;
        }

        /* chdiv must fit in 8 bits, ckdiv must fit in 3 bits */
        while ((chdiv > TWI_CLK_DIV_MAX) && (ckdiv < TWI_CLK_DIV_MIN)) {
            /* Increase clock divider */
            ckdiv++;
            /* Divide cldiv value */
            chdiv /= TWI_CLK_DIVIDER;
        }

        /* set clock waveform generator register */
        twi(dev)->TWI_CWGR =
            TWI_CWGR_CLDIV(cldiv) | TWI_CWGR_CHDIV(chdiv) |
            TWI_CWGR_CKDIV(ckdiv);
    } else {
        c_lh_div = CLOCK_CORECLOCK / (ul_speed * TWI_CLK_DIVIDER) - TWI_CLK_CALC_ARGU;

        /* cldiv must fit in 8 bits, ckdiv must fit in 3 bits */
        while ((c_lh_div > TWI_CLK_DIV_MAX) && (ckdiv < TWI_CLK_DIV_MIN)) {
            /* Increase clock divider */
            ckdiv++;
            /* Divide cldiv value */
            c_lh_div /= TWI_CLK_DIVIDER;
        }

        /* set clock waveform generator register */
        twi(dev)->TWI_CWGR =
            TWI_CWGR_CLDIV(c_lh_div) | TWI_CWGR_CHDIV(c_lh_div) |
            TWI_CWGR_CKDIV(ckdiv);
    }
}

static inline void twi_master_init(i2c_t dev)
{
    /* Disable TWI interrupts */
    twi(dev)->TWI_IDR = ~0UL;

    /* Dummy read in status register */
    twi(dev)->TWI_SR;

    /* Reset TWI peripheral */
    twi_reset(twi(dev));

    /* Master mode and bus speed */
    twi_enable_master_mode(twi(dev));
    i2c_set_speed(dev, i2c_config[dev].speed);
}

void i2c_init(i2c_t dev)
{
    assert(dev < I2C_NUMOF);

    /* initialize device lock */
    mutex_init(&locks[dev]);

    /* program PIO controller */
    gpio_init_mux(i2c_config[dev].scl, i2c_config[dev].mux);
    gpio_init_mux(i2c_config[dev].sda, i2c_config[dev].mux);

    /* initialize TWI master mode */
    twi_master_init(dev);

    /* disable the peripheral clock */
    PMC->PMC_PCER0 &= ~(1 << i2c_config[dev].id);
}

int i2c_acquire(i2c_t dev)
{
    /* lock bus */
    mutex_lock(&locks[dev]);

    /* enable the peripheral clock */
    PMC->PMC_PCER0 |= (1 << i2c_config[dev].id);

    return 0;
}

int i2c_release(i2c_t dev)
{
    /* disable the peripheral clock */
    PMC->PMC_PCER0 &= ~(1 << i2c_config[dev].id);

    /* release device lock */
    mutex_unlock(&locks[dev]);

    return 0;
}

int i2c_read_regs(i2c_t dev, uint16_t addr, uint16_t reg,
        void *data, size_t len, uint8_t flags)
{
    (void) dev;
    (void) addr; (void) reg; (void) data; (void) len; (void) flags;
    return -1;
}

int i2c_read_bytes(i2c_t dev, uint16_t addr,
        void *data, size_t len, uint8_t flags)
{
    (void) dev;
    (void) addr; (void) data; (void) len; (void) flags;
    return -1;
}

int i2c_write_bytes(i2c_t dev, uint16_t addr, const void *data,
        size_t len, uint8_t flags)
{
    (void) dev;
    (void) addr; (void) data; (void) len; (void) flags;
    return -1;
}

int i2c_write_regs(i2c_t dev, uint16_t addr, uint16_t reg,
        const void *data, size_t len, uint8_t flags)
{
    (void) dev;
    (void) addr; (void) reg; (void) data; (void) len; (void) flags;
    return -1;
}

