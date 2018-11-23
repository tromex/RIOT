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


/** Time-out value (number of attempts). */
#define TWI_TIMEOUT 1500000

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
    uint32_t i;

    /* Set SWRST bit to reset TWI peripheral */
    p_twi->TWI_CR = TWI_CR_SWRST;
    p_twi->TWI_RHR;

    /* Wait 10 ms */
    for (i=0; i<1000000; i++);

    /* TWI Slave Mode Disabled, TWI Master Mode Disabled */
    p_twi->TWI_CR = TWI_CR_MSDIS | TWI_CR_SVDIS;
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

static inline uint32_t i2c_compute_speed(i2c_speed_t speed)
{
    switch (speed) {
    case I2C_SPEED_LOW:
        return 10000;
    case I2C_SPEED_NORMAL:
        return 100000;
    case I2C_SPEED_FAST:
        return 400000;
    case I2C_SPEED_FAST_PLUS:
        return 1000000;
    case I2C_SPEED_HIGH:
        return 3400000;
    default:
        return 100000;
    }
}

static void i2c_set_clock(i2c_t dev, uint32_t ul_speed)
{
    uint32_t clock = CLOCK_CORECLOCK;
    uint32_t ckDiv = 0;
    uint32_t clDiv = 0;

    while (1) {
        clDiv = ((clock / (2 * ul_speed)) - 4) / (1 << ckDiv);
        if (clDiv <= 255)
            break;
	else
	    ckDiv++;
    }

    twi(dev)->TWI_CWGR = 0;
    twi(dev)->TWI_CWGR = (ckDiv << 16) | (clDiv << 8) | clDiv;
}

static inline void twi_master_init(i2c_t dev)
{
    /* Disable TWI interrupts */
    twi(dev)->TWI_IDR = ~0UL;

    /* Disable PDC Channel */
    twi(dev)->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

    /* SVEN: TWI Slave Mode Enabled */
    twi(dev)->TWI_CR = TWI_CR_SVEN;

    /* Reset TWI peripheral */
    twi_reset(twi(dev));

    /* Master mode */
    twi(dev)->TWI_CR = TWI_CR_MSEN;

    i2c_set_clock(dev, 100000);
}

void i2c_init(i2c_t dev)
{
    assert(dev < I2C_NUMOF);

    /* initialize device lock */
    mutex_init(&locks[dev]);

    /* program PIO controller */
    gpio_init(i2c_config[dev].scl, GPIO_OD_PU);
    gpio_init(i2c_config[dev].sda, GPIO_OD_PU);
    gpio_init_mux(i2c_config[dev].scl, i2c_config[dev].mux);
    gpio_init_mux(i2c_config[dev].sda, i2c_config[dev].mux);

    /* initialize TWI master mode */
    twi_master_init(dev);

    /* disable the peripheral clock */
    if ((PMC->PMC_PCSR0 & (1u << i2c_config[dev].id)) == (1u << i2c_config[dev].id))
        PMC->PMC_PCDR0 = 1 << i2c_config[dev].id;
}

int i2c_acquire(i2c_t dev)
{
    /* lock bus */
    mutex_lock(&locks[dev]);

    /* enable the peripheral clock */
    if ((PMC->PMC_PCSR0 & (1u << i2c_config[dev].id)) != (1u << i2c_config[dev].id))
        PMC->PMC_PCER0 = 1 << i2c_config[dev].id;

    return 0;
}

int i2c_release(i2c_t dev)
{
    /* disable the peripheral clock */
    if ((PMC->PMC_PCSR0 & (1u << i2c_config[dev].id)) == (1u << i2c_config[dev].id))
        PMC->PMC_PCDR0 = 1 << i2c_config[dev].id;

    /* release device lock */
    mutex_unlock(&locks[dev]);

    return 0;
}

int i2c_read_regs(i2c_t dev, uint16_t addr, uint16_t reg,
        void *data, size_t len, uint8_t flags)
{
    uint32_t status;
    uint8_t reg_len = (flags & I2C_REG16) ? 2 : 1;
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
        ((reg_len << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk);

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

    if (!(flags & I2C_NOSTOP)) {
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
    uint8_t reg_len = (flags & I2C_REG16) ? 2 : 1;
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

