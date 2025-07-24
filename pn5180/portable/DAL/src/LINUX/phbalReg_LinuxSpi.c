/*
*         Copyright (c), NXP Semiconductors Bangalore / India
*
*                     (C)NXP Semiconductors
*       All rights are reserved. Reproduction in whole or in part is
*      prohibited without the written consent of the copyright owner.
*  NXP reserves the right to make changes without notice at any time.
* NXP makes no warranty, expressed, implied or statutory, including but
* not limited to any implied warranty of merchantability or fitness for any
*particular purpose, or that the use will not infringe any third party patent,
* copyright or trademark. NXP must not be liable for any loss or damage
*                          arising from its use.
*/

/** \file
* Generic phDriver(DAL) Component of Reader Library Framework.
* $Author$		qinyuan
* $Revision$	v1.0
* $Date$		2025/07/23
*
* History:
*  PGh: Fixed case sensitivity for linux build
*  RS:  Generated 24. Jan 2017
*
*/

#include "phDriver.h"
#include "BoardSelection.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <errno.h>

#define PHBAL_REG_LINUX_SPI_ID               0x0DU       /**< ID for LPC Open SPI BAL component */

static void phbalReg_LpcOpenSpiConfig(void);

#ifdef PERF_TEST
static uint32_t dwSpiBaudRate = PHDRIVER_USER_SPI_FREQ;
#endif /* PERF_TEST */

static int 		spi_fd = -1; // spi fd
static uint32_t spi_speed = PHDRIVER_USER_SPI_FREQ;	// spi speed

/**
* \brief Initialize the Linux SPI BAL layer.
*
* \return Status code
* \retval #PH_DRIVER_SUCCESS Operation successful.
* \retval #PH_ERR_INVALID_DATA_PARAMS Parameter structure size is invalid.
*/
phStatus_t phbalReg_Init(
                                      void * pDataParams,
                                      uint16_t wSizeOfDataParams
                                      )
{
	char		spi_dev[32];
	uint8_t 	mode = PHDRIVER_USER_SPI_CFG_MODE;
	uint8_t		bits = PHDRIVER_USER_SPI_CFG_BITS_PER_WORD;
	uint32_t	speed = PHDRIVER_USER_SPI_FREQ;

	if((pDataParams == NULL) || (sizeof(phbalReg_Type_t) != wSizeOfDataParams))
    {
        return (PH_DRIVER_ERROR | PH_COMP_DRIVER);
    }

    ((phbalReg_Type_t *)pDataParams)->wId      = PH_COMP_DRIVER | PHBAL_REG_LINUX_SPI_ID;
    ((phbalReg_Type_t *)pDataParams)->bBalType = PHBAL_REG_TYPE_SPI;

    phbalReg_LinuxSpiConfig();

    snprintf(spi_dev, sizeof(spi_dev), "%s%d.%d", 
             PHDRIVER_USER_SPI_CFG_DIR, PHDRIVER_USER_SPI_BUS, PHDRIVER_USER_SPI_CS);
    
	/* open spi dev */
    spi_fd = open(spi_dev, O_RDWR);
    if (spi_fd < 0) {
        return (PH_DRIVER_ERROR | PH_COMP_DRIVER);
    }

#ifdef PERF_TEST
    speed = dwSpiBaudRate;
#endif
	/* set spi: mode, bits and speed */
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        close(spi_fd);
        spi_fd = -1;
        return (PH_DRIVER_ERROR | PH_COMP_DRIVER);
    }

    spi_speed = speed;
    
	/* wait Startuo time */
	usleep(10000);

    return PH_DRIVER_SUCCESS;
}

/*
 * use ioctl to transfer spi data
 */
phStatus_t phbalReg_Exchange(
                                        void * pDataParams,
                                        uint16_t wOption,
                                        uint8_t * pTxBuffer,
                                        uint16_t wTxLength,
                                        uint16_t wRxBufSize,
                                        uint8_t * pRxBuffer,
                                        uint16_t * pRxLength
                                        )
{
	struct 	spi_ioc_transfer tr = {0};
	int 	ret;

	if(spi_fd < 0)
	{
		return (PH_DRIVER_ERROR | PH_COMP_DRIVER);
	}

	/* set spi_ioc_transfer tr */
	tr.tx_buf = (unsigned long)pTxBuffer;
	tr.rx_buf = (unsigned long)pRxBuffer;
	tr.len = wTxLength;
	tr.speed_hz = spi_speed;
	tr.bits_per_word = PHDRIVER_USER_SPI_CFG_BITS_PER_WORD;

	/* send only */
	if(wRxBufSize == 0)
	{
		tr.tx_buf = 0;
		ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
		if(pRxLength != NULL)
		{
			*pRxLength = (ret < 0) ? 0 : wTxLength;
		}
	}
	else 
	{
		/* read only */
		if(wTxLength == 0)
		{
			tr.tx_buf = 0;
			tr.len = wRxBufSize;
			ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
			if(pRxLength != NULL)
			{
				*pRxLength = (ret < 0) ? 0 : wRxBufSize;
			}
		}
		/* read and send */
		else 
		{
			ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
			if(pRxLength != NULL)
			{
				*pRxLength = (ret < 0) ? 0 : wTxLength;
			}		
		}
	}

	if(ret < 0)
	{
		return (PH_DRIVER_FAILURE | PH_COMP_DRIVER);
	}

	return PH_DRIVER_SUCCESS;
}

phStatus_t phbalReg_SetConfig(
                                         void * pDataParams,
                                         uint16_t wConfig,
                                         uint32_t dwValue
                                         )
{
#ifdef PERF_TEST
    switch(wConfig)
    {
 	   case PHBAL_CONFIG_SPI_BAUD:
    	    dwSpiBaudRate = dwValue;
			if(spi_fd >= 0)
			{
				if(ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &dwSpiBaudRate) < 0)
				{
					return (PH_DRIVER_ERROR | PH_COMP_DRIVER);
				}
				spi_speed = dwSpiBaudRate;
			}
    		break;
    
	   default:
			return (PH_DRIVER_ERROR | PH_COMP_DRIVER);
	}
#endif /* PERF_TEST */
    return PH_DRIVER_SUCCESS;
}

phStatus_t phbalReg_GetConfig(
                                         void * pDataParams,
                                         uint16_t wConfig,
                                         uint32_t * pValue
                                         )
{
#ifdef PERF_TEST
    switch(wConfig)
    {
		case PHBAL_CONFIG_SPI_BAUD:
        	if(spi_fd >= 0)
			{
				if(ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, pValue) < 0)
				{
					*pValue = dwSpiBaudRate;
				}
			}
			else
			{
				*pValue = dwSpiBaudRate;
			}
        	break;
    	
		default:
        	return (PH_DRIVER_ERROR | PH_COMP_DRIVER);
    }
#endif /* PERF_TEST */
    return PH_DRIVER_SUCCESS;
}

static void phbalReg_LinuxSpiConfig(void)
{
    /* SPI pins are configured by device tree in Linux */
}
#if 0
static void phbalReg_LpcOpenSpiConfig(void)
{
    /* Configure SSP pins (SCK, MOSI and MISO) */
    Chip_IOCON_PinMux(LPC_IOCON, (uint8_t)((((uint32_t)PHDRIVER_PIN_MOSI) & 0xFF00) >> 8),
            (uint8_t)(((uint32_t)PHDRIVER_PIN_MOSI) & 0xFF),
            IOCON_MODE_INACT,
            (uint8_t)((((uint32_t)PHDRIVER_PIN_MOSI) & 0xFF0000) >> 16));
    Chip_IOCON_PinMux(LPC_IOCON, (uint8_t)((((uint32_t)PHDRIVER_PIN_MISO) & 0xFF00) >> 8),
            (uint8_t)(((uint32_t)PHDRIVER_PIN_MISO) & 0xFF),
            IOCON_MODE_INACT,
            (uint8_t)((((uint32_t)PHDRIVER_PIN_MISO) & 0xFF0000) >> 16));
    Chip_IOCON_PinMux(LPC_IOCON, (uint8_t)((((uint32_t)PHDRIVER_PIN_SCK) & 0xFF00) >> 8),
            (uint8_t)(((uint32_t)PHDRIVER_PIN_SCK) & 0xFF),
            IOCON_MODE_INACT,
            (uint8_t)((((uint32_t)PHDRIVER_PIN_SCK) & 0xFF0000) >> 16));
}
#endif
