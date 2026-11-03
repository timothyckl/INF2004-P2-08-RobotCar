/*
 *----------------------------------------------------------------------
 *    Device Driver for μT-Kernel 3.00.05
 *
 *    Copyright (C) 2020-2021 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2021/11.
 *
 *----------------------------------------------------------------------
 */


/*
 *	config_device.h
 *	Device configuration definition
 */

#ifndef	__DEV_CONFIG_H__
#define	__DEV_CONFIG_H__

/* ------------------------------------------------------------------------ */
/* Device usage settings
 *	1: Use   0: Do not use
 */

#define DEVCNF_USE_SER		1		// Serial communication device 
#define DEVCNF_USE_ADC		1		// A/D conversion device
#define DEVCNF_USE_IIC		1		// I2C communication device

/* GP0/GP1 carry the right wheel encoder (AGENTS.md), so the boot code
   must leave them alone. sysdef.h defaults BOARD_UART0_FREE to 0, so test
   its value, not only whether it is defined. */
#if defined(_PICO_RP2040_) \
    && (!defined(BOARD_UART0_FREE) || (0 == BOARD_UART0_FREE))
#error "Encoder inputs on GP0/GP1 require UART0 to be disabled or relocated"
#endif

#endif	/* __DEV_CONFIG_H__ */
