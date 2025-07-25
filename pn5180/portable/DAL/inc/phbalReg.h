/*----------------------------------------------------------------------------*/
/* Copyright 2017-2024 NXP                                                    */
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
* $Author: NXP $
* $Revision: $
* $Date: $
*/

#ifndef PHBALREG_H
#define PHBALREG_H

#ifdef __cplusplus
extern "C" {
#endif    /* __cplusplus */

/** \defgroup phbalReg Bus Abstraction Layer (BAL)
* @{
*/

/**
* \brief BAL type structure
*/
typedef struct
{
    uint16_t       wId;              /**< Layer ID for this BAL component, NEVER MODIFY! */
    uint8_t        bBalType;         /**< BAL type used by HAL to configure the BAL configured at runtime. */
} phbalReg_Type_t;

/**
* \name Generic BAL configuration parameters
*/
/*@{*/
#define PHBAL_REG_CONFIG_WRITE_TIMEOUT_MS   0x0000U /**< Configure transmission timeout in milliseconds. */
#define PHBAL_REG_CONFIG_READ_TIMEOUT_MS    0x0001U /**< Configure reception timeout in milliseconds. */
/*@}*/

/**
* \name BAL types
*/
/*@{*/
#define PHBAL_REG_TYPE_SPI                  0x0001U /**< SPI communication channel. */
#define PHBAL_REG_TYPE_I2C                  0x0002U /**< I2C communication channel. */
#define PHBAL_REG_TYPE_SERIAL_WIN           0x0003U /**< SerialWin communication channel. */
#define PHBAL_REG_TYPE_KERNEL_SPI           0x0004U /**< Linux kernel space SPI communication channel. */
#define PHBAL_REG_TYPE_USER_SPI             0x0005U /**< Linux user space SPI communication channel. */

/*@}*/

/**
* \name BAL set/get config commands
*/
/*@{*/
#define PHBAL_KERNEL_SPI_IOCTL_MODE         (0x0U)

/*@}*/

/**
* \name BAL set/get config values
*/
/*@{*/
#define PHBAL_KERNEL_SPI_MODE_NORMAL        (0x0U)
#define PHBAL_KERNEL_SPI_MODE_DWL           (0x1U)
#define PHBAL_CONFIG_SPI_BAUD               (0x2U)

/*@}*/

#ifdef _WIN32
#ifdef NXPBUILD__PHBAL_REG_SERIALWIN
/**
* \ID for Serial BAL component
*/
/*@{*/
#define PHBAL_REG_SERIALWIN_ID                                                              0x01U   /**< ID for Serial BAL component */

/** \name Option to supress checks after ReadFile is called. */
#define PHBAL_REG_SERIALWIN_SUPRESS_CHECKS                                                  0x0080U /**< Option to supress checks after ReadFile is called.  */

/*@}*/
#endif /* NXPBUILD__PHBAL_REG_SERIALWIN */

#ifdef NXPBUILD__PHBAL_REG_PCSCWIN
/**
* \ID for PCSC BAL component
*/
/*@{*/
#define PHBAL_REG_PCSCWIN_ID                                                                0x06U   /**< ID for PCSC BAL component */
/*@}*/
#endif /* NXPBUILD__PHBAL_REG_PCSCWIN */
#endif /* _WIN32 */

/**
 * end of group phbalReg
 * @}
 */

#ifdef _WIN32

/**
* \name Windows BAL communication interface apis.
*/
/**
 *@{
 */

/**
* \brief List all available ports.
* \return Status code
* \retval #PH_ERR_SUCCESS Operation successful.
* \retval #PH_ERR_BUFFER_OVERFLOW Given enumeration buffer is too small.
* \retval #PH_ERR_INTERFACE_ERROR Error while enumerating devices.
*/
phStatus_t phbalReg_GetPortList(
    void * pDataParams,     /**< [In] Pointer to this layer's parameter structure. */
    uint16_t wPortBufSize,  /**< [In] Buffer Size of Port Name String. */
    uint8_t * pPortNames,   /**< [Out] Port Name as Multi-String. */
    uint16_t * pNumOfPorts  /**< [Out] Number of found port strings. */
);

/**
* \brief Select port to be used.
*
* <em>Example SerialWin:</em> The caller has to ensure that \c pPortName
* is valid throughout the whole lifetime of \c pDataParams.\n
* Furthermore, the caller is responsible for prepending "\\.\" if COM ports above
* COM9 need to be accessed.\n\n
* \b Example:
\code
strcpy(pPortName, "COM9");         <-- correct
strcpy(pPortName, "\\\\.\\COM9");  <-- correct
strcpy(pPortName, "\\\\.\\COM10"); <-- correct
strcpy(pPortName, "COM10");        <-- wrong
\endcode
* \return Status code
* \retval #PH_ERR_SUCCESS Operation successful.
*/
phStatus_t phbalReg_SetPort(
    void * pDataParams, /**< [In] Pointer to this layer's parameter structure. */
    uint8_t * pPortName /**< [In] Port Name as String. */
);

/**
* \brief Open communication port.
* \return Status code
* \retval #PH_ERR_SUCCESS Operation successful.
* \retval #PH_ERR_USE_CONDITION Communication port is already open.
* \retval #PH_ERR_INTERFACE_ERROR Error while opening port.
*/
phStatus_t phbalReg_OpenPort(
    void * pDataParams /**< [In] Pointer to this layer's parameter structure. */
);

/**
* \brief Close communication port.
* \return Status code
* \retval #PH_ERR_SUCCESS Operation successful.
* \retval #PH_ERR_USE_CONDITION Communication port is not open.
* \retval #PH_ERR_INTERFACE_ERROR Error while closing port.
*/
phStatus_t phbalReg_ClosePort(
    void * pDataParams /**< [In] Pointer to this layer's parameter structure. */
);

/**
 * end of Windows BAL communication interface apis
 * @}
 */

#endif /* _WIN32 */

/**
* \name BAL communication interface apis.
*/
/**
 *@{
 */

/**
* \brief Initialize the BAL.
*
* \return Status code
* \retval #PH_DRIVER_SUCCESS Operation successful.
* \retval #PH_DRIVER_ERROR   Parameter structure size is invalid.
*/
phStatus_t phbalReg_Init(void * pDataParams,    /**< [In] Pointer to this layer's parameter structure phbalReg_Type_t. */
        uint16_t wSizeOfDataParams              /**< [In] Size of this layer's parameter structure. */
        );

/**
* \brief Perform data Transmit/Receive/Exchange on the bus. This API is used for command exchange between front-end CLIF
* and HOST.
* SPI : This API will also perform required flow control for particular front-end (Hardware specific).
*
* \return Status code
* \retval #PH_DRIVER_SUCCESS Operation successful.
* \retval #PH_DRIVER_ERROR   \b wOption is invalid or Response is too big for either given receive buffer or internal buffer.
* \retval #PH_DRIVER_TIMEOUT No response received within given time frame.
* \retval #PH_DRIVER_FAILURE Communication error.
*/
phStatus_t phbalReg_Exchange(
                             void * pDataParams,    /**< [In] Pointer to this layer's parameter structure. */
                             uint16_t wOption,      /**< [In] Option parameter, for future use. */
                             uint8_t * pTxBuffer,   /**< [In] Data to transmit. */
                             uint16_t wTxLength,    /**< [In] Number of bytes to transmit, if 0 Tx is not performed. */
                             uint16_t wRxBufSize,   /**< [In] Size of receive buffer / Number of bytes to receive (depending on implementation). If 0 Rx is not performed.  */
                             uint8_t * pRxBuffer,   /**< [Out] Received data. */
                             uint16_t * pRxLength   /**< [Out] Number of received data bytes. */
                             );

/**
* \brief Set configuration parameter.
* \return Status code
* \retval #PH_DRIVER_SUCCESS Operation successful.
* \retval #PH_DRIVER_ERROR   Parameter/Configuration is not supported or invalid.
* \retval #PH_DRIVER_FAILURE Communication error.
*/
phStatus_t phbalReg_SetConfig(
                              void * pDataParams,   /**< [In] Pointer to this layer's parameter structure. */
                              uint16_t wConfig,     /**< [In] Configuration identifier, for future use. */
                              uint32_t dwValue      /**< [In] Configuration value. */
                              );

/**
* \brief Get configuration parameter.
* \return Status code
* \retval #PH_DRIVER_SUCCESS Operation successful.
* \retval #PH_DRIVER_ERROR Configuration is not supported or invalid.
* \retval #PH_DRIVER_FAILURE Communication error.
*/
phStatus_t phbalReg_GetConfig(
                              void * pDataParams,   /**< [In] Pointer to this layer's parameter structure. */
                              uint16_t wConfig,     /**< [In] Configuration identifier, for future use. */
                              uint32_t * pValue     /**< [Out] Configuration value. */
                              );

/**
 * end of BAL communication interface apis
 * @}
 */

#ifdef __cplusplus
} /* Extern C */
#endif

#endif /* PHBALREG_H */
