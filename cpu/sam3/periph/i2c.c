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

#include <stdint.h>
#include <errno.h>

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

/** Time-out value (number of attempts). */
#define TWI_TIMEOUT 15000

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

/**
 * \internal
 * \brief Construct the TWI module address register field
 *
 * The TWI module address register is sent out MSB first. And the size controls
 * which byte is the MSB to start with.
 *
 * Please see the device datasheet for details on this.
 */
static uint32_t twi_mk_addr(const uint16_t reg, int len)
{
    uint32_t val;

    if (len == 0)
        return 0;

    val = reg & 0xFF;
    if (len > 1) {
        val <<= 8;
        val |= (reg & 0xFF00);
    }
    return val;
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
    uint32_t status;
    uint8_t reg_len = (flags | I2C_REG16) ? 2 : 1;
    uint32_t cnt = len;
    uint8_t *buffer = data;
    uint8_t stop_sent = 0;
    uint32_t timeout = TWI_TIMEOUT;

    /* Check argument */
    if (cnt == 0)
        return -EINVAL;

    /* Set read mode, slave address and 3 internal address byte lengths */
    twi(dev)->TWI_MMR = 0;
    twi(dev)->TWI_MMR = TWI_MMR_MREAD | TWI_MMR_DADR(addr) |
        ((reg_len << TWI_MMR_IADRSZ_Pos) &
         TWI_MMR_IADRSZ_Msk);

    /* Set internal address for remote chip */
    twi(dev)->TWI_IADR = 0;
    twi(dev)->TWI_IADR = twi_mk_addr(reg, reg_len);

    /* Send a START condition */
    if (cnt == 1) {
        twi(dev)->TWI_CR = TWI_CR_START | TWI_CR_STOP;
        stop_sent = 1;
    } else {
        twi(dev)->TWI_CR = TWI_CR_START;
        stop_sent = 0;
    }

    while (cnt > 0) {
        status = twi(dev)->TWI_SR;
        if (status & TWI_SR_NACK)
            return -EIO;

        if (!timeout--)
            return -ETIMEDOUT;

        /* Last byte ? */
        if (cnt == 1  && !stop_sent) {
            twi(dev)->TWI_CR = TWI_CR_STOP;
            stop_sent = 1;
        }

        if (!(status & TWI_SR_RXRDY))
            continue;

        *buffer++ = twi(dev)->TWI_RHR;

        cnt--;
        timeout = TWI_TIMEOUT;
    }

    while (!(twi(dev)->TWI_SR & TWI_SR_TXCOMP));
    twi(dev)->TWI_SR;

    return 0;
}

int i2c_read_bytes(i2c_t dev, uint16_t addr,
        void *data, size_t len, uint8_t flags)
{
    (void) flags;

    uint32_t status;
    uint32_t cnt = len;
    uint8_t *buffer = data;
    uint8_t stop_sent = 0;
    uint32_t timeout = TWI_TIMEOUT;

    /* Check argument */
    if (cnt == 0)
        return -EINVAL;

    /* Set read mode and slave address */
    twi(dev)->TWI_MMR = 0;
    twi(dev)->TWI_MMR = TWI_MMR_MREAD | TWI_MMR_DADR(addr);

    /* Send a START condition */
    if (cnt == 1) {
        twi(dev)->TWI_CR = TWI_CR_START | TWI_CR_STOP;
        stop_sent = 1;
    } else {
        twi(dev)->TWI_CR = TWI_CR_START;
        stop_sent = 0;
    }

    while (cnt > 0) {
        status = twi(dev)->TWI_SR;
        if (status & TWI_SR_NACK)
            return -EIO;

        if (!timeout--)
            return -ETIMEDOUT;

        /* Last byte ? */
        if (cnt == 1  && !stop_sent) {
            twi(dev)->TWI_CR = TWI_CR_STOP;
            stop_sent = 1;
        }

        if (!(status & TWI_SR_RXRDY))
            continue;

        *buffer++ = twi(dev)->TWI_RHR;

        cnt--;
        timeout = TWI_TIMEOUT;
    }

    while (!(twi(dev)->TWI_SR & TWI_SR_TXCOMP));
    twi(dev)->TWI_SR;

    return 0;
}

int i2c_write_bytes(i2c_t dev, uint16_t addr, const void *data,
        size_t len, uint8_t flags)
{
    uint32_t status;
    uint32_t cnt = len;
    const uint8_t *buffer = data;

    /* Check argument */
    if (cnt == 0)
        return -EINVAL;

    /* Set write mode and slave address */
    twi(dev)->TWI_MMR = 0;
    twi(dev)->TWI_MMR = TWI_MMR_DADR(addr);

    /* Send all bytes */
    while (cnt > 0) {
        status = twi(dev)->TWI_SR;
        if (status & TWI_SR_NACK)
            return -EIO;
        if (!(status & TWI_SR_TXRDY))
            continue;

        twi(dev)->TWI_THR = *buffer++;
        cnt--;
    }

    while (1) {
        status = twi(dev)->TWI_SR;
        if (status & TWI_SR_NACK)
            return -EIO;

        if (status & TWI_SR_TXRDY)
            break;
    }

    if (!(flags | I2C_NOSTOP)) {
        /* Send stop */
        twi(dev)->TWI_CR = TWI_CR_STOP;
        while (!(twi(dev)->TWI_SR & TWI_SR_TXCOMP));
    }

    return 0;
}

int i2c_write_regs(i2c_t dev, uint16_t addr, uint16_t reg,
        const void *data, size_t len, uint8_t flags)
{
    uint32_t status;
    uint8_t reg_len = (flags | I2C_REG16) ? 2 : 1;
    uint32_t cnt = len;
    const uint8_t *buffer = data;

    /* Check argument */
    if (cnt == 0)
        return -EINVAL;

    /* Set write mode, slave address and 3 internal address byte lengths */
    twi(dev)->TWI_MMR = 0;
    twi(dev)->TWI_MMR = TWI_MMR_DADR(addr) |
        ((reg_len << TWI_MMR_IADRSZ_Pos) &
         TWI_MMR_IADRSZ_Msk);

    /* Set internal address for remote chip */
    twi(dev)->TWI_IADR = 0;
    twi(dev)->TWI_IADR = twi_mk_addr(reg, reg_len);

    /* Send all bytes */
    while (cnt > 0) {
        status = twi(dev)->TWI_SR;
        if (status & TWI_SR_NACK)
            return -EIO;
        if (!(status & TWI_SR_TXRDY))
            continue;

        twi(dev)->TWI_THR = *buffer++;
        cnt--;
    }

    while (1) {
        status = twi(dev)->TWI_SR;
        if (status & TWI_SR_NACK)
            return -EIO;

        if (status & TWI_SR_TXRDY)
            break;
    }

    /* Send stop */
    twi(dev)->TWI_CR = TWI_CR_STOP;
    while (!(twi(dev)->TWI_SR & TWI_SR_TXCOMP));

    return 0;
}

