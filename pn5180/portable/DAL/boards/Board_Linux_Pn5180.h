/*----------------------------------------------------------------------------*/
/* Copyright 2017-2022 NXP                                                    */
/*                                                                            */
/* NXP Confidential. This software is owned or controlled by NXP and may only */
/* be used strictly in accordance with the applicable license terms.          */
/* By expressly accepting such terms or by downloading, installing,           */
/* activating and/or otherwise using the software, you are agreeing that you  */
/* have read, and that you agree to comply with and are bound by, such        */
/* license terms. If you do not agree to be bound by the applicable license   */
/* terms, then you may not retain, install, activate or otherwise use the     */
/* software.                                                                  */
/*----------------------------------------------------------------------------*/

/** \file
* Generic phDriver Component of Reader Library Framework.
* $Author$		qinyuan
* $Revision$	v1.0
* $Date$		2025/07/23
*
*/

#ifndef BOARD_LINUX_PN5180_H
#define BOARD_LINUX_PN5180_H

/*******************************************************************
 *  Enable User/Kernel space SPI.
 ******************************************************************/
#define PHDRIVER_LINUX_USER_SPI
// #define PHDRIVER_LINUX_KERNEL_SPI

/******************************************************************
 * Board Pin/Gpio configurations
 ******************************************************************/
#define PHDRIVER_PIN_RESET         87   /**< "/sys/class/gpio/gpio87/" */
#define PHDRIVER_PIN_IRQ           10   /**< "/sys/class/gpio/gpio10/" */
#define PHDRIVER_PIN_BUSY          88   /**< "/sys/class/gpio/gpio88/" */
#define PHDRIVER_PIN_DWL           18   /**< "/sys/class/gpio/gpio18/", NOT used */

/******************************************************************
 * PIN Pull-Up/Pull-Down configurations.
 ******************************************************************/
#define PHDRIVER_PIN_RESET_PULL_CFG    PH_DRIVER_PULL_UP
#define PHDRIVER_PIN_IRQ_PULL_CFG      PH_DRIVER_PULL_UP
#define PHDRIVER_PIN_BUSY_PULL_CFG     PH_DRIVER_PULL_UP
#define PHDRIVER_PIN_DWL_PULL_CFG      PH_DRIVER_PULL_UP

/******************************************************************
 * IRQ & BUSY PIN TRIGGER settings
 ******************************************************************/
#define PIN_IRQ_TRIGGER_TYPE         PH_DRIVER_INTERRUPT_RISINGEDGE		// FALLING
#define PIN_BUSY_TRIGGER_TYPE        PH_DRIVER_INTERRUPT_FALLINGEDGE

/*****************************************************************
 * Front End Reset logic level settings
 ****************************************************************/
#define PH_DRIVER_SET_HIGH            1          /**< Logic High. */
#define PH_DRIVER_SET_LOW             0          /**< Logic Low. */
#define RESET_POWERDOWN_LEVEL       PH_DRIVER_SET_LOW
#define RESET_POWERUP_LEVEL         PH_DRIVER_SET_HIGH

/*****************************************************************
 * SPI Configuration
 ****************************************************************/
#ifdef PHDRIVER_LINUX_USER_SPI
#    define PHDRIVER_USER_SPI_BUS                    1   /**< "/dev/spidev1.0" */
#    define PHDRIVER_USER_SPI_CS                     0   /**< "/dev/spidev1.0" */
#    define PHDRIVER_USER_SPI_FREQ                   5000000 /**< 5 MHz. */
#    define PHDRIVER_USER_SPI_CFG_DIR                "/dev/spidev"
#    define PHDRIVER_USER_SPI_CFG_MODE               SPI_MODE_0
#    define PHDRIVER_USER_SPI_CFG_BITS_PER_WORD      8
#endif

#ifdef PHDRIVER_LINUX_KERNEL_SPI
#    define PHDRIVER_KERNEL_SPI_ID                   0x11U       /**< ID for Linux Kernel Spi BAL component */
#    define PHDRIVER_KERNEL_SPI_CFG_DIR              "/dev/bal"
#endif

/*****************************************************************
 * Dummy entries
 * No functionality. To suppress build error in HAL. No pin functionality in SPI Linux BAL.
 *****************************************************************/
#define PHDRIVER_PIN_SSEL                            0xFFFF
#define PHDRIVER_PIN_NSS_PULL_CFG                    PH_DRIVER_PULL_UP

/*****************************************************************
 * STATUS LED Configuration
 ****************************************************************/
#define PHDRIVER_LED_SUCCESS_DELAY      2

#define PHDRIVER_LED_FAILURE_DELAY_MS   250
#define PHDRIVER_LED_FAILURE_FLICKER    4

#endif /* BOARD_PIPN5180_H */

