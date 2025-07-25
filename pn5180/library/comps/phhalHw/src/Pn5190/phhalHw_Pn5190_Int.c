/*----------------------------------------------------------------------------*/
/* Copyright 2019-2022 NXP                                                    */
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
* Internal functions for Pn5190 HAL.
*
* $Author$
* $Revision$ (v07.13.00)
* $Date$
*/

#include <phhalHw.h>
#include <ph_RefDefs.h>
#include <ph_Status.h>

#ifdef NXPBUILD__PHHAL_HW_PN5190
#ifndef _WIN32
#include <phDriver.h>
#include "BoardSelection.h"
#endif
#include <phTools.h>
#include "phhalHw_Pn5190.h"
#include "phhalHw_Pn5190_InstrMngr.h"
#include "phhalHw_Pn5190_Wait.h"
#include "phhalHw_Pn5190_Int.h"
#include "phhalHw_Pn5190_Reg.h"
#include "phhalHw_Pn5190_Instr.h"

#define PHHAL_HW_15693_26KBPS                               (0x00000000)
#define PHHAL_HW_15693_53KBPS                               (0x00000020)
#define PHHAL_HW_15693_106KBPS                              (0x00000040)
#define PHHAL_HW_15693_212KBPS                              (0x00000060)

#define CLIF_SIGPRO_RM_ENABLES_RM_OOK_COL_LOW_SLOPE_MASK    (0x00018000UL)
#define CLIF_SIGPRO_RM_ENABLES_REG_PROTOCOL_INDEX           (0x8U)
#define RETRIEVE_RF_CONFIG_REG_VALUE_PAIR_LENGTH            (0x5U)

#ifndef _WIN32
extern phOsal_Event_t xEventHandle;
#endif

phStatus_t phhalHw_Pn5190_Int_GetTxBuffer(
                                          phhalHw_Pn5190_DataParams_t * pDataParams,
                                          uint8_t ** pTxBuffer,
                                          uint16_t * pTxBufferLen,
                                          uint16_t * pTxBufferSize
                                          )
{
    /* We need to watch that we do not overwrite content below the RxStartPos though. */
    if (pDataParams->pTxBuffer == (pDataParams->pRxBuffer - 1))
    {
        *pTxBuffer = &pDataParams->pTxBuffer[pDataParams->wRxBufStartPos];
        *pTxBufferSize = pDataParams->wTxBufSize - pDataParams->wRxBufStartPos;
    }
    /* Else just return the actual Buffer. */
    else
    {
        *pTxBuffer = pDataParams->pTxBuffer;
        *pTxBufferSize = pDataParams->wTxBufSize;
    }

    /* Return stored length. */
    *pTxBufferLen = pDataParams->wTxBufLen;

    return PH_ERR_SUCCESS;
}

phStatus_t phhalHw_Pn5190_Int_GetRxBuffer(
                                          phhalHw_Pn5190_DataParams_t * pDataParams,
                                          uint8_t ** pRxBuffer,
                                          uint16_t * pRxBufferLen,
                                          uint16_t * pRxBufferSize
                                          )
{
    uint8_t PH_MEMLOC_REM bOffset = 0U;

    if ((pDataParams->bActiveMode != PH_ON) && (pDataParams->bNfcipMode == PH_ON))
    {
        bOffset = 1U;
    }
    /* Update out parameters */
    *pRxBuffer = &pDataParams->pRxBuffer[pDataParams->wRxBufStartPos - bOffset];
    *pRxBufferSize = pDataParams->wRxBufSize - pDataParams->wRxBufStartPos - bOffset;
    *pRxBufferLen = pDataParams->wRxBufLen;

    return PH_ERR_SUCCESS;
}

void phhalHw_Pn5190_Int_Reset(void)
{
#ifndef _WIN32
    /* Send the reset pulse to FE to reset. */
    (void)phDriver_PinWrite(
        PHDRIVER_PIN_RESET,
        RESET_POWERUP_LEVEL);

    /* Delay of ~5 ms */
    (void)phDriver_TimerStart(
        PH_DRIVER_TIMER_MILLI_SECS,
        PHHAL_HW_PN5190_RESET_DELAY_MILLI_SECS,
        NULL);

    (void)phDriver_PinWrite(
      PHDRIVER_PIN_RESET,
      RESET_POWERDOWN_LEVEL);

    /* Delay of ~5 ms */
    (void)phDriver_TimerStart(
        PH_DRIVER_TIMER_MILLI_SECS,
        PHHAL_HW_PN5190_RESET_DELAY_MILLI_SECS,
        NULL);

    (void)phDriver_PinWrite(
      PHDRIVER_PIN_RESET,
      RESET_POWERUP_LEVEL);

    /* Delay of ~5 ms */
    (void)phDriver_TimerStart(
        PH_DRIVER_TIMER_MILLI_SECS,
        PHHAL_HW_PN5190_RESET_DELAY_MILLI_SECS,
        NULL);
#endif /*_WIN32*/
}

phStatus_t phhalHw_Pn5190_Int_TimerStart(
    phhalHw_Pn5190_DataParams_t * pDataParams,
    uint8_t bTimer,
    uint32_t dwStartCond,
    uint32_t dwStopCond,
    uint32_t wPrescaler,
    uint32_t dwLoadValue
    )
{
    phStatus_t  PH_MEMLOC_REM  status;
    uint8_t     PH_MEMLOC_REM  bTmrConfigReg;
    uint8_t     PH_MEMLOC_REM  bTmrRelaodReg;
    uint32_t    PH_MEMLOC_REM  dwEnableMask;
    uint32_t    PH_MEMLOC_REM  dwStartNowMask;

    uint32_t    PH_MEMLOC_REM dwTemp;
    uint8_t     PH_MEMLOC_BUF wRegTypeValueSets[18];
    uint16_t    PH_MEMLOC_REM wSizeOfRegTypeValueSets;
    phOsal_EventBits_t PH_MEMLOC_REM tReceivedEvents = 0U;
    uint8_t * PH_MEMLOC_REM pEvtPayload = NULL;

    /* Populate the timer configure and reload registers */

    switch(bTimer)
    {
    case CLIF_TIMER0_CONFIG:
        bTmrConfigReg = CLIF_TIMER0_CONFIG;
        bTmrRelaodReg = CLIF_TIMER0_RELOAD;
        dwEnableMask = CLIF_TIMER0_CONFIG_T0_ENABLE_MASK;
        dwStartNowMask = CLIF_TIMER0_CONFIG_T0_START_NOW_MASK;
        break;

    case CLIF_TIMER1_CONFIG:
        bTmrConfigReg = CLIF_TIMER1_CONFIG;
        bTmrRelaodReg = CLIF_TIMER1_RELOAD;
        dwEnableMask = CLIF_TIMER1_CONFIG_T1_ENABLE_MASK;
        dwStartNowMask = CLIF_TIMER1_CONFIG_T1_START_NOW_MASK;
        break;

    default:
        return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
    }

    if ((dwStartCond & dwStartNowMask) == dwStartNowMask)
    {
        if (bTimer == CLIF_TIMER0_CONFIG)
        {
            phOsal_EventClear(&pDataParams->HwEventObj.EventHandle, E_OS_EVENT_OPT_NONE, E_PH_OSAL_EVT_SIG, NULL);
        }
        else
        {
            dwEnableMask = 0;
            dwStartCond = 0;
        }
    }

    /*write 0 to stop timer*/
    wSizeOfRegTypeValueSets = 0U;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = bTmrConfigReg;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = PHHAL_HW_PN5190_WRITE_MULTIPLE_TYPE_WRITE;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = 0x00U;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = 0x00U;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = 0x00U;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = 0x00U;

    /*load the timer  reload value*/
    dwTemp = (dwLoadValue & TMR_RELOAD_VALUE_MASK);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = bTmrRelaodReg;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = PHHAL_HW_PN5190_WRITE_MULTIPLE_TYPE_WRITE;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>>8U);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>>16U);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>>24U);

    /* Timer MODE_SEL is defined by ePrescaler, 0x01U enables the timer */
    dwTemp = (dwStartCond | dwStopCond | wPrescaler | dwEnableMask );
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = bTmrConfigReg;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = PHHAL_HW_PN5190_WRITE_MULTIPLE_TYPE_WRITE;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>>8U);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>>16U);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>>24U);

    PH_CHECK_SUCCESS_FCT(status, phhalHw_Pn5190_Instr_WriteRegisterMultiple( pDataParams, wRegTypeValueSets, wSizeOfRegTypeValueSets));

    /* If the timer is started immediately, then wait for Interrupt (Timer Event) from FW */
    if ((bTimer == CLIF_TIMER0_CONFIG) && ((dwStartCond & dwStartNowMask) == dwStartNowMask))
    {
        /*Wait for Timer event*/
        PH_CHECK_SUCCESS_FCT(status, phhalHw_Pn5190_WaitForEvent(
            pDataParams,
            PH_PN5190_EVT_TIMER,
            PHOSAL_MAX_DELAY,
            true,
            &tReceivedEvents,
            &pEvtPayload));
    }

    return PH_ERR_SUCCESS;
}

phStatus_t phhalHw_Pn5190_SetConfig_Int(
    phhalHw_Pn5190_DataParams_t * pDataParams,
    uint16_t wConfig,
    uint16_t wValue
    )
{
    phStatus_t  PH_MEMLOC_REM statusTmp = 0U;
    uint32_t    PH_MEMLOC_REM dwValue = 0U;
    uint8_t     PH_MEMLOC_BUF wRegTypeValueSets[12] = {0};
    uint16_t    PH_MEMLOC_REM wSizeOfRegTypeValueSets = 0U;
    uint32_t    PH_MEMLOC_REM dwTemp = 0U;
    float32_t   PH_MEMLOC_REM fTime = 0.0;
    uint8_t     PH_MEMLOC_REM aRetrieveRfConfig[PHHAL_HW_PN5190_MIN_RF_CONFIGURATION_BUFFER_SIZE] = {0};
    uint16_t    PH_MEMLOC_REM wRxLength = sizeof(aRetrieveRfConfig);
    uint8_t     PH_MEMLOC_REM bRxRMEn_Offset = CLIF_SIGPRO_RM_ENABLES_REG_PROTOCOL_INDEX * RETRIEVE_RF_CONFIG_REG_VALUE_PAIR_LENGTH;

    switch(wConfig)
    {
    case PHHAL_HW_CONFIG_PARITY:

        /* Tx-Parity,  Rx-Parity is OFF */
        if (wValue == PH_OFF)
        {
            dwValue = (uint32_t)~(uint32_t)CLIF_TX_FRAME_CONFIG_TX_PARITY_ENABLE_MASK;
            /* Perform write */
            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegisterAndMask(pDataParams, CLIF_TX_FRAME_CONFIG, dwValue));

            dwValue = (uint32_t)~(uint32_t)CLIF_RX_PROTOCOL_CONFIG_RX_PARITY_ENABLE_MASK;
            /* Perform write */
            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegisterAndMask(pDataParams, CLIF_RX_PROTOCOL_CONFIG, dwValue));
        }
        else
        {
            /* Turn ON Tx-Parity */
            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegisterOrMask(pDataParams,
                    CLIF_TX_FRAME_CONFIG, CLIF_TX_FRAME_CONFIG_TX_PARITY_ENABLE_MASK));

            /* Turn ON Rx-Parity */
            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegisterOrMask(pDataParams, CLIF_RX_PROTOCOL_CONFIG, CLIF_RX_PROTOCOL_CONFIG_RX_PARITY_ENABLE_MASK));
        }

        break;

    case PHHAL_HW_CONFIG_TXCRC:

        if (wValue == PH_OFF)
        {
            /* CRC calculator, your services are not required */
            dwValue = (uint32_t)~(uint32_t)CLIF_CRC_TX_CONFIG_TX_CRC_ENABLE_MASK;

            /* Perform write */
            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegisterAndMask(pDataParams, CLIF_CRC_TX_CONFIG, dwValue));

            if (pDataParams->bCardType == PHHAL_HW_CARDTYPE_ISO14443A)
            {
                PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_RetrieveRfConfiguration(pDataParams,
                    PHHAL_HW_PN5190_RF_RX_ISO14443A_106_MANCH_SUBC,
                    &aRetrieveRfConfig[0],
                    &wRxLength));

                if (aRetrieveRfConfig[bRxRMEn_Offset++] == CLIF_SIGPRO_RM_ENABLES)
                {
                    dwValue = aRetrieveRfConfig[bRxRMEn_Offset++];
                    dwValue |= (aRetrieveRfConfig[bRxRMEn_Offset++] << 8U);
                    dwValue |= (aRetrieveRfConfig[bRxRMEn_Offset++] << 16U);
                    dwValue |= (aRetrieveRfConfig[bRxRMEn_Offset++] << 24U);

                    PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegisterOrMask(pDataParams,
                        CLIF_SIGPRO_RM_ENABLES,
                        (dwValue & CLIF_SIGPRO_RM_ENABLES_RM_OOK_COL_LOW_SLOPE_MASK)));
                }
            }
        }
        else
        {

            wSizeOfRegTypeValueSets = 0U;

            /*Clear the Bits*/
            dwTemp = (uint32_t) ~( CLIF_CRC_TX_CONFIG_TX_CRC_ENABLE_MASK);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = CLIF_CRC_TX_CONFIG;
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = PHHAL_HW_PN5190_WRITE_MULTIPLE_TYPE_WRITE_AND_MASK;
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>> 8U);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>> 16U);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>> 24U);

            /*Set the new value  */
            dwTemp = (uint32_t)(CLIF_CRC_TX_CONFIG_TX_CRC_ENABLE_MASK);
            /* If the card type is other than I18000p3m3, operate the CRC in 16-bit mode */
            if (pDataParams->bCardType == PHHAL_HW_CARDTYPE_I18000P3M3)
            {
                /* Just set the bit for 5-bit mode operation */
                dwTemp |= (uint32_t)CLIF_CRC_TX_CONFIG_TX_CRC_TYPE_MASK;
            }

            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = CLIF_CRC_TX_CONFIG;
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = PHHAL_HW_PN5190_WRITE_MULTIPLE_TYPE_WRITE_OR_MASK;
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>> 8U);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>> 16U);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>> 24U);

            /*Send the array to the IC*/
            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegisterMultiple(pDataParams,  wRegTypeValueSets, wSizeOfRegTypeValueSets));

            if (pDataParams->bCardType == PHHAL_HW_CARDTYPE_ISO14443A)
            {
                PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegisterAndMask(pDataParams, CLIF_SIGPRO_RM_ENABLES, (uint32_t)(~CLIF_SIGPRO_RM_ENABLES_RM_OOK_COL_LOW_SLOPE_MASK)));
            }
        }
        break;

    case PHHAL_HW_CONFIG_RXCRC:

        if (wValue == PH_OFF)
        {
            /* CRC calculator, your services are not required */
            dwValue = (uint32_t)~(uint32_t)CLIF_CRC_RX_CONFIG_RX_CRC_ENABLE_MASK;

            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegisterAndMask(pDataParams, CLIF_CRC_RX_CONFIG, dwValue));
        }
        else
        {

            wSizeOfRegTypeValueSets = 0U;

            /*Clear the Bits */
            dwTemp = (uint32_t) ~( CLIF_CRC_RX_CONFIG_RX_CRC_TYPE_MASK);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = CLIF_CRC_RX_CONFIG;
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = PHHAL_HW_PN5190_WRITE_MULTIPLE_TYPE_WRITE_AND_MASK;
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>> 8U);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>> 16U);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>> 24U);

            /*Set the new value  */
            dwTemp = (uint32_t)CLIF_CRC_RX_CONFIG_RX_CRC_ENABLE_MASK;
            /* If the card type is other than I18000p3m3, operate the CRC in 16-bit mode */
            if (pDataParams->bCardType == PHHAL_HW_CARDTYPE_I18000P3M3)
            {
                /* Just set the bit for 5-bit mode operation */
                dwTemp |= (uint32_t)CLIF_CRC_RX_CONFIG_RX_CRC_TYPE_MASK;
            }

            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = CLIF_CRC_RX_CONFIG;
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = PHHAL_HW_PN5190_WRITE_MULTIPLE_TYPE_WRITE_OR_MASK;
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>> 8U);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>> 16U);
            wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>> 24U);

            /*Send the array to the IC*/
            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegisterMultiple(pDataParams,  wRegTypeValueSets, wSizeOfRegTypeValueSets));

        }
        break;

    case PHHAL_HW_CONFIG_ASK100:
        /* TODO:
         * Presently for ASK100/ASK10 is changed for ISO15693, loading will be
         * done usingthe load_rf_config, But check whether can it be done to
         * change a particular register itself to support this switching.
         */
        if(PHHAL_HW_CARDTYPE_ISO15693 == pDataParams->bCardType )
        {
            /* switch off 100% ASK */
            if (wValue == PH_OFF)
            {
                PH_CHECK_SUCCESS_FCT(statusTmp,phhalHw_Pn5190_Instr_LoadRfConfiguration(pDataParams, PHHAL_HW_PN5190_RF_TX_ISO15693_26_1OF4_ASK10, 0xFF));
            }
            /* switch on 100% ASK */
            else
            {
                PH_CHECK_SUCCESS_FCT(statusTmp,phhalHw_Pn5190_Instr_LoadRfConfiguration(pDataParams, PHHAL_HW_PN5190_RF_TX_ISO15693_26_1OF4_ASK100, 0xFF));
            }
        }
        break;

    case PHHAL_HW_CONFIG_RXWAIT_US:
        /* Set Rx Wait(deaf) bits */
        dwValue = PHHAL_HW_Pn5190_TR_RX_PRESCALAR;
        dwTemp = wValue;
        if(0U != dwTemp)
        {
            fTime = (float32_t)(((float32_t)dwTemp * 13.56) / (float32_t)(dwValue));
            dwTemp = (uint32_t)fTime;
            dwValue |= (uint32_t)dwTemp << CLIF_RX_WAIT_RX_WAIT_VALUE_POS;
            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegister(pDataParams, CLIF_RX_WAIT, dwValue ));
        }
        else
        {
            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegister(pDataParams, CLIF_RX_WAIT, dwValue ));
        }
        break;

    default:
        return PH_ADD_COMPCODE_FIXED(PH_ERR_INTERNAL_ERROR, PH_COMP_HAL);
    }

    return PH_ERR_SUCCESS;
}

phStatus_t phhalHw_Pn5190_Int_SetConfig_FelicaEmdReg(
    phhalHw_Pn5190_DataParams_t * pDataParams
    )
{
    phStatus_t PH_MEMLOC_REM statusTmp;

    if (pDataParams->bOpeMode != RD_LIB_MODE_FELICA)
    {
        /* Clear FeliCa EMD Control Register */
        PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegister(pDataParams, FELICA_EMD_CONTROL, (uint32_t)0x0U));
        /* Reset shadow register */
        pDataParams->dwFelicaEmdReg = (uint32_t)0x0U;
    }
    else
    {
        /* Configure FeliCa EMD Control Register with default value */
        PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegister(pDataParams, FELICA_EMD_CONTROL, PHHAL_HW_PN5190_DEFAULT_FELICA_EMD_REGISTER));
        /* Configure shadow register with default value */
        pDataParams->dwFelicaEmdReg = PHHAL_HW_PN5190_DEFAULT_FELICA_EMD_REGISTER;
    }

    return PH_ERR_SUCCESS;
}

phStatus_t phhalHw_Pn5190_Int_SetConfig_FelicaEmdRegBit(
    phhalHw_Pn5190_DataParams_t * pDataParams,
    uint16_t wValue,
    uint32_t dwMaskValue
    )
{
    phStatus_t  PH_MEMLOC_REM statusTmp;

    if ((wValue != PH_ON) && (wValue != PH_OFF))
    {
        return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
    }

    if(wValue != PH_OFF)
    {
        if (!(pDataParams->dwFelicaEmdReg & dwMaskValue))
        {
            /* Update FELICA_EMD_CONTROL Register */
            PH_CHECK_SUCCESS_FCT(statusTmp,
                phhalHw_Pn5190_Instr_WriteRegisterOrMask(pDataParams, FELICA_EMD_CONTROL, dwMaskValue));
            /* Update shadow register */
            pDataParams->dwFelicaEmdReg |= dwMaskValue;
        }
    }
    else
    {
        if (pDataParams->dwFelicaEmdReg & dwMaskValue)
        {
            /* Update FELICA_EMD_CONTROL Register */
            PH_CHECK_SUCCESS_FCT(statusTmp,
                phhalHw_Pn5190_Instr_WriteRegisterAndMask(pDataParams, FELICA_EMD_CONTROL, (uint32_t)~dwMaskValue));
            /* Update shadow register */
            pDataParams->dwFelicaEmdReg &= (uint32_t)~dwMaskValue;
        }
    }

    return PH_ERR_SUCCESS;
}

phStatus_t phhalHw_Pn5190_Int_SetConfig_FelicaEmdRegByte(
    phhalHw_Pn5190_DataParams_t * pDataParams,
    uint16_t wValue,
    uint8_t bBytePos,
    uint32_t dwMaskValue
    )
{
    phStatus_t  PH_MEMLOC_REM statusTmp;
    uint32_t    PH_MEMLOC_REM dwValue;

    if (wValue & 0xFF00U)
    {
        return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
    }

    if ((uint8_t)(pDataParams->dwFelicaEmdReg >> bBytePos) != (uint8_t)wValue)
    {
        dwValue = pDataParams->dwFelicaEmdReg;
        dwValue &= ((uint32_t)~dwMaskValue);
        dwValue |= ((uint32_t)wValue << bBytePos);
        /* Update FELICA_EMD_CONTROL Register */
        PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Instr_WriteRegister(pDataParams, FELICA_EMD_CONTROL, dwValue));
        /* Update shadow register */
        pDataParams->dwFelicaEmdReg = dwValue;
    }

    return PH_ERR_SUCCESS;
}

phStatus_t phhalHw_Pn5190_Int_SetCardMode(
    phhalHw_Pn5190_DataParams_t * pDataParams,
    uint16_t wTxDataRate,
    uint16_t wRxDataRate,
    uint16_t wSubcarrier )
{
    phStatus_t  PH_MEMLOC_REM statusTmp;
    uint8_t     PH_MEMLOC_REM bTxConfig = 0U;
    uint8_t     PH_MEMLOC_REM bRxConfig = 0U;

    if(pDataParams->wTargetMode == PH_OFF)
    {
        if(wTxDataRate == pDataParams->wCfgShadow[PHHAL_HW_CONFIG_TXDATARATE_FRAMING])
        {
            wTxDataRate = PHHAL_HW_RF_DATARATE_NO_CHANGE;
        }
        if(wRxDataRate == pDataParams->wCfgShadow[PHHAL_HW_CONFIG_RXDATARATE_FRAMING])
        {
            wRxDataRate = PHHAL_HW_RF_DATARATE_NO_CHANGE;
        }
    }

/*
 * In Reader/initiator mode bCardType is set as part fo apply protocol setting and during PSL/PPS bCardType updated.
 * In Passive Card/Target mode bCardType is set to NONE/UNKNOWN in autocoll and is updated during PSL/PPS.
 * In Active Target mode bCardType is set to NONE/UNKNOWN in autocoll, hence below check has been added to hanlde it.
 * NOTE: Active mode Proprietary Baudrate communication is not handled.
 * */

    if ((pDataParams->wTargetMode != PH_OFF) && (pDataParams->bActiveMode == PH_ON))
    {
        switch (wTxDataRate)
        {
            case PHHAL_HW_RF_DATARATE_106:
                bTxConfig = PHHAL_HW_PN5190_RF_TX_NFC_AT_106;
                break;
            case PHHAL_HW_RF_DATARATE_212:
                bTxConfig = PHHAL_HW_PN5190_RF_TX_NFC_AT_212;
                break;
            case PHHAL_HW_RF_DATARATE_424:
                bTxConfig = PHHAL_HW_PN5190_RF_TX_NFC_AT_424;
                break;
            case PHHAL_HW_RF_DATARATE_NO_CHANGE:
                bTxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
                break;
            default:
                return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
        }

        switch (wRxDataRate)
        {
            case PHHAL_HW_RF_DATARATE_106:
                bRxConfig = PHHAL_HW_PN5190_RF_RX_NFC_AT_106;
                break;
            case PHHAL_HW_RF_DATARATE_212:
                bRxConfig = PHHAL_HW_PN5190_RF_RX_NFC_AT_212;
                break;
            case PHHAL_HW_RF_DATARATE_424:
                bRxConfig = PHHAL_HW_PN5190_RF_RX_NFC_AT_424;
                break;
            case PHHAL_HW_RF_DATARATE_NO_CHANGE:
                bRxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
                break;
            default:
                return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
        }
    }
    else
    {
       switch (pDataParams->bCardType)
       {
       case PHHAL_HW_CARDTYPE_ISO14443A:

           /* Check if Target is activated and perform required change to switch BaudRate. */
           if (pDataParams->wTargetMode != PH_OFF)
           {
               switch (wTxDataRate)
               {
               case PHHAL_HW_RF_DATARATE_106:
                   bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO14443A_PICC_106_MANCH_SUBC;
                   /*pTxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_106;*/
                   break;
               case PHHAL_HW_RF_DATARATE_212:
                   bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO14443A_PICC_212_BPSK;
                   /*pTxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_212;*/
                   break;
               case PHHAL_HW_RF_DATARATE_424:
                   bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO14443A_PICC_424_BPSK;
                   /* pTxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_424;*/
                   break;
               case PHHAL_HW_RF_DATARATE_848:
                   bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO14443A_PICC_848_BPSK;
                   /* pTxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_848;*/
                   break;
               case PHHAL_HW_RF_DATARATE_NO_CHANGE:
                   bTxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
                   break;
               default:
                   return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
               }

               switch (wRxDataRate)
               {
               case PHHAL_HW_RF_DATARATE_106:
                   bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO14443A_PICC_106_MILLER;
                   /*pRxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_106;*/
                   break;
               case PHHAL_HW_RF_DATARATE_212:
                   bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO14443A_PICC_212_MILLER;
                   /*pRxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_212;*/
                   break;
               case PHHAL_HW_RF_DATARATE_424:
                   bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO14443A_PICC_424_MILLER;
                   /* pRxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_424;*/
                   break;
               case PHHAL_HW_RF_DATARATE_848:
                   bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO14443A_PICC_848_MILLER;
                   /* pRxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_848;*/
                   break;
               case PHHAL_HW_RF_DATARATE_NO_CHANGE:
                   bRxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
                   break;
               default:
                   return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
               }
           }
           else
           {
               switch (wTxDataRate)
               {
               case PHHAL_HW_RF_DATARATE_106:
                   bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO14443A_106_MILLER;
                   /*pTxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_106;*/
                   break;
               case PHHAL_HW_RF_DATARATE_212:
                   bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO14443A_212_MILLER;
                   /*pTxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_212;*/
                   break;
               case PHHAL_HW_RF_DATARATE_424:
                   bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO14443A_424_MILLER;
                   /* pTxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_424;*/
                   break;
               case PHHAL_HW_RF_DATARATE_848:
                   bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO14443A_848_MILLER;
                   /* pTxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_848;*/
                   break;
               case PHHAL_HW_RF_DATARATE_NO_CHANGE:
                   bTxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
                   break;
               default:
                   return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
               }
               switch (wRxDataRate)
               {
               case PHHAL_HW_RF_DATARATE_106:
                   bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO14443A_106_MANCH_SUBC;
                   /*pRxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_106;*/
                   break;
               case PHHAL_HW_RF_DATARATE_212:
                   bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO14443A_212_BPSK;
                   /*pRxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_212;*/
                   break;
               case PHHAL_HW_RF_DATARATE_424:
                   bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO14443A_424_BPSK;
                   /* pRxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_424;*/
                   break;
               case PHHAL_HW_RF_DATARATE_848:
                   bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO14443A_848_BPSK;
                   /* pRxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443A_848;*/
                   break;
               case PHHAL_HW_RF_DATARATE_NO_CHANGE:
                   bRxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
                   break;
               default:
                   return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
               }
           }
           break;

       case PHHAL_HW_CARDTYPE_ISO14443B:

           switch (wTxDataRate)
           {
           case PHHAL_HW_RF_DATARATE_106:
               bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO14443B_106_NRZ;
               /* pTxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443B_106;*/
               break;
           case PHHAL_HW_RF_DATARATE_212:
               bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO14443B_212_NRZ;
               /* pTxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443B_212;*/
               break;
           case PHHAL_HW_RF_DATARATE_424:
               bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO14443B_424_NRZ;
               /*pTxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443B_424;*/
               break;
           case PHHAL_HW_RF_DATARATE_848:
               bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO14443B_848_NRZ;
               /*pTxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443B_848;*/
               break;
           case PHHAL_HW_RF_DATARATE_NO_CHANGE:
               bTxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
               break;
           default:
               return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
           }

           switch (wRxDataRate)
           {
           case PHHAL_HW_RF_DATARATE_106:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO14443B_106_BPSK;
               /* pRxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443B_106;*/
               break;
           case PHHAL_HW_RF_DATARATE_212:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO14443B_212_BPSK;
               /*pRxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443B_212;*/
               break;
           case PHHAL_HW_RF_DATARATE_424:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO14443B_424_BPSK;
               /* pRxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443B_424;*/
               break;
           case PHHAL_HW_RF_DATARATE_848:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO14443B_848_BPSK;
               /*  pRxRegisterSet = (const uint8_t*)gkphhalHw_Pn5190_I14443B_848;*/
               break;
           case PHHAL_HW_RF_DATARATE_NO_CHANGE:
               bRxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
               break;
           default:
               return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
           }
           break;

       case PHHAL_HW_CARDTYPE_FELICA_212:
       case PHHAL_HW_CARDTYPE_FELICA_424:

           if (pDataParams->wTargetMode != PH_OFF)
           {
               switch (wTxDataRate)
               {
               case PHHAL_HW_RF_DATARATE_212:
                   bTxConfig = PHHAL_HW_PN5190_RF_TX_NFC_PT_212;
                   break;

               case PHHAL_HW_RF_DATARATE_424:
                   bTxConfig = PHHAL_HW_PN5190_RF_TX_NFC_PT_424;
                   break;
               case PHHAL_HW_RF_DATARATE_NO_CHANGE:
                   bTxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
                   break;
               default:
                   return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
               }

               switch (wRxDataRate)
               {
               case PHHAL_HW_RF_DATARATE_212:
                   bRxConfig = PHHAL_HW_PN5190_RF_RX_NFC_PT_212;
                   break;

               case PHHAL_HW_RF_DATARATE_424:
                   bRxConfig = PHHAL_HW_PN5190_RF_RX_NFC_PT_424;
                   break;
               case PHHAL_HW_RF_DATARATE_NO_CHANGE:
                   bRxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
                   break;
               default:
                   return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
               }
           }
           else
           {
               switch (wTxDataRate)
               {
               case PHHAL_HW_RF_DATARATE_212:
                   bTxConfig = PHHAL_HW_PN5190_RF_TX_FELICA_212;
                   break;

               case PHHAL_HW_RF_DATARATE_424:
                   bTxConfig = PHHAL_HW_PN5190_RF_TX_FELICA_424;
                   break;

               case PHHAL_HW_RF_DATARATE_NO_CHANGE:
                   bTxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
                   break;

               default:
                   return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
               }

               switch (wRxDataRate)
               {
               case PHHAL_HW_RF_DATARATE_212:
                   bRxConfig = PHHAL_HW_PN5190_RF_RX_FELICA_212;
                   break;

               case PHHAL_HW_RF_DATARATE_424:
                   bRxConfig = PHHAL_HW_PN5190_RF_RX_FELICA_424;
                   break;

               case PHHAL_HW_RF_DATARATE_NO_CHANGE:
                   bRxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
                   break;

               default:
                   return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
               }

           }
           break;

       case PHHAL_HW_CARDTYPE_ISO15693:
           switch (wTxDataRate)
           {
           case PHHAL_HW_RF_TX_DATARATE_1_OUT_OF_4:
               bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO15693_26_1OF4_ASK10;
               break;

           case PHHAL_HW_RF_DATARATE_NO_CHANGE:
               bTxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
               break;

           default:
               return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
           }

           switch (wRxDataRate)
           {
           case PHHAL_HW_RF_RX_DATARATE_LOW:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO15693_6P6_SC;
               break;

           case PHHAL_HW_RF_RX_DATARATE_HIGH:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO15693_26_SC;
               break;

           case PHHAL_HW_RF_RX_DATARATE_FAST_HIGH:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO15693_53_SC;
               break;

           case PHHAL_HW_RF_DATARATE_106:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO15693_106;
               break;

           case PHHAL_HW_RF_DATARATE_212:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO15693_212;
               break;

           case PHHAL_HW_RF_RX_DATARATE_FAST_LOW:
               return PH_ADD_COMPCODE_FIXED(PH_ERR_UNSUPPORTED_PARAMETER, PH_COMP_HAL);

           case PHHAL_HW_RF_DATARATE_NO_CHANGE:
               bRxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
               break;

           default:
               return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
           }
           break;

       case PHHAL_HW_CARDTYPE_I18000P3M3:
           switch (wTxDataRate)
           {
           case PHHAL_HW_RF_TX_DATARATE_I18000P3M3:
               /* Subcarrier check */
               if ((wSubcarrier != PHHAL_HW_SUBCARRIER_DUAL) &&
                   (wSubcarrier != PHHAL_HW_SUBCARRIER_QUAD))
               {
                   return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
               }
               bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO180003M3_TARI_9_44_ASK;
               break;

           case PHHAL_HW_RF_TX_DATARATE_I18000P3M3_TARI1888:
               /* Subcarrier check */
               if ((wSubcarrier != PHHAL_HW_SUBCARRIER_DUAL) &&
                   (wSubcarrier != PHHAL_HW_SUBCARRIER_QUAD))
               {
                   return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
               }
               bTxConfig = PHHAL_HW_PN5190_RF_TX_ISO180003M3_TARI_18_88_ASK;
               break;

           case PHHAL_HW_RF_DATARATE_NO_CHANGE:
               bTxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
               break;

           default:
               return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
           }

           switch (wRxDataRate)
           {
           case PHHAL_HW_RX_I18000P3M3_FL_423_MAN2:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO180003M3_MANCH424_2_PERIOD;
               break;
           case PHHAL_HW_RX_I18000P3M3_FL_423_MAN4:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO180003M3_MANCH424_4_PERIOD;
               break;
           case PHHAL_HW_RX_I18000P3M3_FL_847_MAN2:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO180003M3_MANCH848_2_PERIOD;
               break;
           case PHHAL_HW_RX_I18000P3M3_FL_847_MAN4:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_ISO180003M3_MANCH848_4_PERIOD;
               break;
           case PHHAL_HW_RF_DATARATE_NO_CHANGE:
               bRxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
               break;
           default:
               return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
           }
           break;

       case PHHAL_HW_CARDTYPE_I18092M_ACTIVE_106:
       case PHHAL_HW_CARDTYPE_I18092M_ACTIVE_212:
       case PHHAL_HW_CARDTYPE_I18092M_ACTIVE_424:
           switch (wTxDataRate)
           {
           case PHHAL_HW_RF_DATARATE_106:
               bTxConfig = PHHAL_HW_PN5190_RF_TX_NFC_AI_106_106;
               break;

           case PHHAL_HW_RF_DATARATE_212:
               bTxConfig = PHHAL_HW_PN5190_RF_TX_NFC_AI_212_212;
           break;

           case PHHAL_HW_RF_DATARATE_424:
               bTxConfig = PHHAL_HW_PN5190_RF_TX_NFC_AI_424_424;
               break;

           case PHHAL_HW_RF_DATARATE_NO_CHANGE:
               bTxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
               break;

           default:
               return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
           }

           switch (wRxDataRate)
           {
           case PHHAL_HW_RF_DATARATE_106:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_NFC_AI_106;
               break;

           case PHHAL_HW_RF_DATARATE_212:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_NFC_AI_212;
           break;

           case PHHAL_HW_RF_DATARATE_424:
               bRxConfig = PHHAL_HW_PN5190_RF_RX_NFC_AI_424;
               break;

           case PHHAL_HW_RF_DATARATE_NO_CHANGE:
               bRxConfig = PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX;
               break;

           default:
               return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
           }

           break;

       case PHHAL_HW_CARDTYPE_ICODEEPCUID:
       default:
           return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
       }
    }

    if((PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX != bTxConfig) || (PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX != bRxConfig))
    {
        PH_CHECK_SUCCESS_FCT(statusTmp,
            phhalHw_Pn5190_Instr_LoadRfConfiguration(
            pDataParams,
            (uint8_t) bTxConfig,
            (uint8_t) bRxConfig ));
    }

    /* If Datarate is changed, then Update Data-rate in shadow for parity setting */
    if(wTxDataRate != PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX)
    {
        pDataParams->wCfgShadow[PHHAL_HW_CONFIG_TXDATARATE_FRAMING] = wTxDataRate;
    }
    if(wRxDataRate != PHHAL_HW_PN5190_CURRENT_RF_CONFIGURATION_INDEX)
    {
        pDataParams->wCfgShadow[PHHAL_HW_CONFIG_RXDATARATE_FRAMING] = wRxDataRate;
    }

    return PH_ERR_SUCCESS;
}

phStatus_t phhalHw_Pn5190_Int_SetTxDataRateFraming(phhalHw_Pn5190_DataParams_t * pDataParams,uint16_t wConfig,uint16_t wValue)
{
    uint16_t   PH_MEMLOC_REM wFraming = 0U;
    phStatus_t statusTmp = PH_ERR_SUCCESS;

    if ((pDataParams->wCfgShadow[wConfig] & PHHAL_HW_RF_DATARATE_OPTION_MASK) != (wValue & PHHAL_HW_RF_DATARATE_OPTION_MASK))
    {
        /* Update the framing, based on the Higher byte */
        wFraming = wValue & PHHAL_HW_RF_FRAMING_OPTION_MASK;
        wFraming = wFraming >> 0x08U;

        if((wFraming != PHHAL_HW_CARDTYPE_CURRENT) && (wFraming != pDataParams->bCardType))
        {
            if((wValue & PHHAL_HW_RF_FRAMING_OPTION_MASK) != PHHAL_HW_RF_TYPE_ACTIVE_FRAMING)
            {
                pDataParams->bCardType = (uint8_t)wFraming;
            }
        }

        /* Update the Baudrate based on the lower byte */
        wValue = wValue & PHHAL_HW_RF_DATARATE_OPTION_MASK;

        if(pDataParams->bCardType != PHHAL_HW_CARDTYPE_ISO15693)
        {
            /* Evaluate hardware settings */
            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Int_SetCardMode(
                pDataParams,
                wValue,
                PHHAL_HW_RF_DATARATE_NO_CHANGE,
                pDataParams->wCfgShadow[PHHAL_HW_CONFIG_SUBCARRIER]));
        }

        if(pDataParams->bCardType == PHHAL_HW_CARDTYPE_ISO15693)
        {
            if(wValue == PHHAL_HW_RF_TX_DATARATE_1_OUT_OF_4)
            {
                /* clear the 15693 datarate mask */
                PH_CHECK_SUCCESS_FCT(statusTmp,  phhalHw_Pn5190_Instr_WriteRegisterAndMask( pDataParams, SYSTEM_CONFIG, (uint32_t) (~SYSTEM_CONFIG_15693_CHANGE_DATARATE_MASK)));
                /* set the value */
                PH_CHECK_SUCCESS_FCT(statusTmp,  phhalHw_Pn5190_Instr_WriteRegisterOrMask( pDataParams, SYSTEM_CONFIG, (uint32_t) (PHHAL_HW_15693_26KBPS)));
            }
            else if(wValue == PHHAL_HW_RF_I15693_53KBPS_DATARATE)
            {
                /* clear the 15693 datarate mask */
                PH_CHECK_SUCCESS_FCT(statusTmp,  phhalHw_Pn5190_Instr_WriteRegisterAndMask( pDataParams, SYSTEM_CONFIG, (uint32_t) (~SYSTEM_CONFIG_15693_CHANGE_DATARATE_MASK)));
                /* set the value */
                PH_CHECK_SUCCESS_FCT(statusTmp,  phhalHw_Pn5190_Instr_WriteRegisterOrMask( pDataParams, SYSTEM_CONFIG, (uint32_t) (PHHAL_HW_15693_53KBPS)));
            }
            else if(wValue == PHHAL_HW_RF_DATARATE_106)
            {
                /* clear the 15693 datarate mask */
                PH_CHECK_SUCCESS_FCT(statusTmp,  phhalHw_Pn5190_Instr_WriteRegisterAndMask( pDataParams, SYSTEM_CONFIG, (uint32_t) (~SYSTEM_CONFIG_15693_CHANGE_DATARATE_MASK)));
                /* set the value */
                PH_CHECK_SUCCESS_FCT(statusTmp,  phhalHw_Pn5190_Instr_WriteRegisterOrMask( pDataParams, SYSTEM_CONFIG, (uint32_t) (PHHAL_HW_15693_106KBPS)));
            }
            else if(wValue == PHHAL_HW_RF_DATARATE_212)
            {
                /* clear the 15693 datarate mask */
                PH_CHECK_SUCCESS_FCT(statusTmp,  phhalHw_Pn5190_Instr_WriteRegisterAndMask( pDataParams, SYSTEM_CONFIG, (uint32_t) (~SYSTEM_CONFIG_15693_CHANGE_DATARATE_MASK)));
                /* set the value */
                PH_CHECK_SUCCESS_FCT(statusTmp,  phhalHw_Pn5190_Instr_WriteRegisterOrMask( pDataParams, SYSTEM_CONFIG, (uint32_t) (PHHAL_HW_15693_212KBPS)));
            }
            else
            {
                /* To avoid the warning */
                return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);    /* return the invalid parameter */
            }
        }

        /* Write config data into shadow */
        pDataParams->wCfgShadow[wConfig] = (wValue | (wFraming << 0x08U)) ;
    }

    return PH_ERR_SUCCESS;
}

phStatus_t phhalHw_Pn5190_Int_SetRxDataRateFraming(phhalHw_Pn5190_DataParams_t * pDataParams,uint16_t wConfig,uint16_t wValue)
{
    uint16_t   PH_MEMLOC_REM wFraming = 0U;
    phStatus_t statusTmp = PH_ERR_SUCCESS;

    if ((pDataParams->wCfgShadow[wConfig] & PHHAL_HW_RF_DATARATE_OPTION_MASK) != (wValue & PHHAL_HW_RF_DATARATE_OPTION_MASK))
    {
        /* Update teh framing, based on the Higher byte */
        wFraming = wValue & PHHAL_HW_RF_FRAMING_OPTION_MASK;
        wFraming = wFraming >> 0x08U;

        if((wFraming != PHHAL_HW_CARDTYPE_CURRENT) && (wFraming != pDataParams->bCardType))
        {
            if((wValue & PHHAL_HW_RF_FRAMING_OPTION_MASK) != PHHAL_HW_RF_TYPE_ACTIVE_FRAMING)
            {
                pDataParams->bCardType = (uint8_t)wFraming;
            }
        }

        /* Update the Baudrate based on the lower byte */
        wValue = wValue & PHHAL_HW_RF_DATARATE_OPTION_MASK;

        /* Evaluate hardware settings */
        PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_Int_SetCardMode(
            pDataParams,
            PHHAL_HW_RF_DATARATE_NO_CHANGE,
            wValue,
            pDataParams->wCfgShadow[PHHAL_HW_CONFIG_SUBCARRIER]));

        /* Write config data into shadow */
        pDataParams->wCfgShadow[wConfig] = (wValue | (wFraming << 0x08U));
    }
    return PH_ERR_SUCCESS;
}

phStatus_t phhalHw_Pn5190_Int_SetMinFdt(
                                        phhalHw_Pn5190_DataParams_t * pDataParams,
                                        uint16_t wValue
                                        )
{
    phStatus_t PH_MEMLOC_REM statusTmp = 0U;
    uint16_t   PH_MEMLOC_REM wTimer = 0U;
    uint16_t   PH_MEMLOC_REM wTxRate = 0U;

    if (wValue == PH_ON)
    {
        /* Get current timeout value */
        PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_GetConfig(
            pDataParams,
            PHHAL_HW_CONFIG_TIMEOUT_VALUE_MS,
            &wTimer));

        /* Backup current value */
        pDataParams->dwFdtPc = wTimer;

        /* Get current data rate */
        PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_GetConfig(
            pDataParams,
            PHHAL_HW_CONFIG_TXDATARATE_FRAMING,
            &wTxRate));

        /* Select timeout value based on data rate */
        switch(wTxRate & PHHAL_HW_RF_DATARATE_OPTION_MASK)
        {
            case PHHAL_HW_RF_DATARATE_106:
                wTimer = PHHAL_HW_MINFDT_106_US;
                break;

            case PHHAL_HW_RF_DATARATE_212:
                wTimer = PHHAL_HW_MINFDT_212_US;
                break;

            case PHHAL_HW_RF_DATARATE_424:
                wTimer = PHHAL_HW_MINFDT_424_US;
                break;

            case PHHAL_HW_RF_DATARATE_848:
                wTimer = PHHAL_HW_MINFDT_848_US;
                break;

            default:
                break;
        }

        /* Set timeout value */
        PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_SetConfig(
            pDataParams,
            PHHAL_HW_CONFIG_TIMEOUT_VALUE_US,
            wTimer));
    }
    else if (wValue == PH_OFF)
    {
        /* Restore the timeout value */
        PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Pn5190_SetConfig(
            pDataParams,
            PHHAL_HW_CONFIG_TIMEOUT_VALUE_MS,
            pDataParams->dwFdtPc));
    }
    else
    {
        /* If option is not #PH_OFF or #PH_ON */
        return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
    }

    return PH_ERR_SUCCESS;
}

phStatus_t phhalHw_Pn5190_Int_SetTmo(
    phhalHw_Pn5190_DataParams_t *pDataParams,
    uint16_t wTimeout,
    uint8_t  bUnit
    )
{
    phStatus_t  PH_MEMLOC_REM statusTmp;
    uint32_t    PH_MEMLOC_REM wPrescaler;
    uint32_t    PH_MEMLOC_REM dwLoadValue;

    /* Parameter check */
    if ((bUnit != PHHAL_HW_TIME_MICROSECONDS) && (bUnit != PHHAL_HW_TIME_MILLISECONDS))
    {
        return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_HAL);
    }

    if(wTimeout == 0U)
    {
        return PH_ERR_SUCCESS;
    }

    if (bUnit == PHHAL_HW_TIME_MICROSECONDS)
    {
        /* here wTimeout will be in uS */
        wPrescaler = 0x00U;
        /*Reducing the division by 2 digits to retain the 2 digit decimal places which were getting wiped out*/
        dwLoadValue =(uint32_t) ( PHHAL_HW_PN5190_MAX_FREQ / (PHHAL_HW_PN5190_CONVERSION_US_SEC/100));
        /*Restoring the division done in the earlier step*/
        dwLoadValue =(uint32_t) ((wTimeout * dwLoadValue)/100);
    }
    else
    {
        wPrescaler = 0x3CU;
        if(wTimeout <= PHHAL_HW_PN5190_MAX_TIME_DELAY_MS)
        {
            dwLoadValue =(uint32_t) (  wTimeout * ( PHHAL_HW_PN5190_MIN_FREQ  / PHHAL_HW_PN5190_CONVERSION_MS_SEC) );
        }
        else
        {
            return PH_ADD_COMPCODE_FIXED(PH_ERR_PARAMETER_OVERFLOW, PH_COMP_HAL);
        }
    }

    /* Start the FDT/FWT Timer immediately before executing Exchange CMD to skip Transmit cycle. */
    if (pDataParams->bFeliCaRxOnly != 0)
    {
        PH_CHECK_SUCCESS_FCT(statusTmp,
            phhalHw_Pn5190_Int_TimerStart(
            pDataParams,
            CLIF_TIMER1_CONFIG,
            CLIF_TIMER1_CONFIG_T1_START_NOW_MASK,
            CLIF_TIMER1_CONFIG_T1_STOP_ON_RX_STARTED_MASK,
            wPrescaler,
            dwLoadValue
            ));
    }
    else
    {
        PH_CHECK_SUCCESS_FCT(statusTmp,
            phhalHw_Pn5190_Int_TimerStart(
            pDataParams,
            CLIF_TIMER1_CONFIG,
            CLIF_TIMER1_CONFIG_T1_START_ON_TX_ENDED_MASK,
            CLIF_TIMER1_CONFIG_T1_STOP_ON_RX_STARTED_MASK,
            wPrescaler,
            dwLoadValue
            ));
    }

    return PH_ERR_SUCCESS;
}

phStatus_t phhalHw_Pn5190_Int_Retrieve_Data(
        phhalHw_Pn5190_DataParams_t * pDataParams,
        uint8_t ** ppRxBuffer,
        uint16_t * pRxLength
        )
{
    phStatus_t  PH_MEMLOC_REM status = PH_ERR_SUCCESS;
    uint16_t wTmpRxBufferLen = 0U;
    uint16_t wTmpRxBufferSize = 0U;
    uint16_t wNumByteReceived = 0U;
    uint32_t dwValue = 0U;
    uint32_t dwColPos = 0U;
    uint32_t dwTemp = 0U;
    uint32_t dwRx_Status = 0U;
    uint32_t dwRf_Status = 0U;
    uint32_t dwEvt_Status = 0U;
    uint8_t  bBackup = 0U;
    uint8_t * pTmpRxBuffer = 0U;
    uint8_t * pRxData = 0U;

    pRxData = * ppRxBuffer;

    dwRx_Status = (uint32_t)pRxData[0];
    dwRx_Status |= (uint32_t)pRxData[1] << 8U;
    dwRx_Status |= (uint32_t)pRxData[2] << 16U;
    dwRx_Status |= (uint32_t)pRxData[3] << 24U;

    dwRf_Status = (uint32_t)pRxData[4];
    dwRf_Status |= (uint32_t)pRxData[5] << 8U;
    dwRf_Status |= (uint32_t)pRxData[6] << 16U;
    dwRf_Status |= (uint32_t)pRxData[7] << 24U;

    dwEvt_Status = (uint32_t)pRxData[8];
    dwEvt_Status |= (uint32_t)pRxData[9] << 8U;
    dwEvt_Status |= (uint32_t)pRxData[10] << 16U;
    dwEvt_Status |= (uint32_t)pRxData[11] << 24U;

    /* Success; check data received */
    pDataParams->wRxBufLen = 0U;

    phhalHw_Pn5190_Int_GetRxBuffer(
            pDataParams,
            &pTmpRxBuffer,
            &wTmpRxBufferLen,
            &wTmpRxBufferSize);

    if ((pDataParams->bActiveMode != PH_ON) && (pDataParams->bNfcipMode == PH_ON))
    {
        bBackup = pTmpRxBuffer[0];
    }

    if(pDataParams->bRxMultiple == PH_ON)
    {
        wNumByteReceived = (uint16_t)((dwRx_Status & CLIF_RX_STATUS_RX_NUM_BYTES_RECEIVED_MASK) |
            (((dwRx_Status & CLIF_RX_STATUS_RX_NUM_FRAMES_RECEIVED_MASK) >>
                    CLIF_RX_STATUS_RX_NUM_FRAMES_RECEIVED_POS) * 32U));
    }
    else
    {
        wNumByteReceived = (uint16_t)(dwRx_Status & CLIF_RX_STATUS_RX_NUM_BYTES_RECEIVED_MASK);
    }

    /* Check if the buffer size is sufficient*/
    if (wTmpRxBufferSize < wNumByteReceived)
    {
        return PH_ADD_COMPCODE_FIXED(PH_ERR_BUFFER_OVERFLOW, PH_COMP_HAL);
    }

    if(wNumByteReceived != 0)
    {
        PH_CHECK_SUCCESS_FCT(status, phhalHw_Pn5190_Instr_RetrieveRFData(pDataParams, ppRxBuffer, pRxLength));

        if(*pRxLength != wNumByteReceived)
        {
            return PH_ADD_COMPCODE_FIXED(PH_ERR_INTERNAL_ERROR, PH_COMP_HAL);
        }
    }

    /* Extract RX last bits */
    dwTemp = dwRx_Status & CLIF_RX_STATUS_RX_NUM_LAST_BITS_MASK;
    dwTemp = dwTemp >> CLIF_RX_STATUS_RX_NUM_LAST_BITS_POS;

    /* Set RX last bits */
    pDataParams->wAdditionalInfo = (uint16_t)dwTemp;

    if (0U != (dwRf_Status & CLIF_RX_STATUS_ERROR_RX_COLLISION_DETECTED_MASK))
    {
        if ((wNumByteReceived == 0U) && ((dwRx_Status & CLIF_RX_STATUS_RX_COLL_POS_MASK) == 0U))
        {
            /* No data received */
            status = PH_ERR_IO_TIMEOUT;
        }
        else
        {
            status = PH_ERR_COLLISION_ERROR;

            /* Retrieve collision position */
            dwColPos = dwRx_Status & CLIF_RX_STATUS_RX_COLL_POS_MASK;
            dwColPos = dwColPos >> CLIF_RX_STATUS_RX_COLL_POS_POS;

            /* Restore RX buffer size */
            wTmpRxBufferSize = wTmpRxBufferSize + wNumByteReceived;
            if (dwColPos > wTmpRxBufferSize)
            {
                wNumByteReceived = wTmpRxBufferSize;
                status = PH_ERR_BUFFER_OVERFLOW;
            }
            else
            {
                wNumByteReceived = ((uint16_t)dwColPos >> 3U);
            }

            /* Extracting valid bits from Collision position */
            if(dwColPos >= 8U)
            {
                dwValue = dwColPos % 8U ;
            }
            else
            {
                dwValue = dwColPos;
            }

            if (0U != (dwValue))
            {
                /* If it is non zero, some valid bits are received */
                if (wNumByteReceived == wTmpRxBufferSize)
                {
                    status = PH_ERR_BUFFER_OVERFLOW;
                }
                else
                {
                    ++wNumByteReceived;
                }
            }

            /* Store valid bits of last byte */
            pDataParams->wAdditionalInfo = (uint16_t)dwValue;
        }
    }
    else if (0U != (dwRf_Status & CLIF_RX_STATUS_ERROR_RX_DATA_INTEGRITY_ERROR_MASK))
    {
        status = PH_ERR_INTEGRITY_ERROR;
    }
    else if (dwTemp != 0x00U && dwTemp != 0x08U)
    {
        /* Set incomplete byte status if applicable */
        status =  PH_ERR_SUCCESS_INCOMPLETE_BYTE;
    }
    else if (0U != (dwRf_Status & CLIF_RX_STATUS_ERROR_RX_PROTOCOL_ERROR_MASK))
    {
        status = PH_ERR_PROTOCOL_ERROR;
    }
    else
    {
        /* For QAC */
    }

    if ((pDataParams->bActiveMode != PH_ON) && (pDataParams->bNfcipMode == PH_ON) && (wNumByteReceived != 0U))
    {
        if(*ppRxBuffer[0] != 0xF0U)
        {
            return PH_ADD_COMPCODE_FIXED(PH_ERR_IO_TIMEOUT, PH_COMP_HAL);
        }
        pTmpRxBuffer[0] = bBackup;
        (*ppRxBuffer)++;
        wNumByteReceived--;
    }

    /* Store received data length in dataparams */
    pDataParams->wRxBufLen = pDataParams->wRxBufStartPos + wNumByteReceived;

    *ppRxBuffer = pDataParams->pRxBuffer;
    *pRxLength = pDataParams->wRxBufLen;

    return PH_ADD_COMPCODE(status, PH_COMP_HAL);
}

phStatus_t phhalHw_Pn5190_Int_PCD_GetExchgFdt(
        phhalHw_Pn5190_DataParams_t * pDataParams,
        uint32_t *pFdtUs
)
{
    phStatus_t  PH_MEMLOC_REM status = PH_ERR_SUCCESS;
    uint32_t dwTimer1Output = 0;
    uint32_t dwTimer1Reload = 0;
    uint32_t dwTimer1Config = 0;
    uint8_t bDigitalDelay;

    uint32_t dwTmp;
    float32_t fTime = 0;

    /* Read Timer1 output register to get 20bit counter value */
    PH_CHECK_SUCCESS_FCT(status, phhalHw_Pn5190_Instr_ReadRegister(pDataParams, CLIF_TIMER1_OUTPUT, &dwTimer1Output));
    dwTimer1Output = dwTimer1Output & CLIF_TIMER1_OUTPUT_T1_VALUE_MASK;

    /* Get T1 reload value */
    PH_CHECK_SUCCESS_FCT(status, phhalHw_Pn5190_Instr_ReadRegister(pDataParams, CLIF_TIMER1_RELOAD, &dwTimer1Reload));
    dwTimer1Reload = dwTimer1Reload & CLIF_TIMER1_RELOAD_T1_RELOAD_VALUE_MASK;

    /* Subtract reload and counter values */
    dwTmp = dwTimer1Reload - dwTimer1Output;

    /* Get digital delay for Type A */
    if(pDataParams->bCardType == PHHAL_HW_CARDTYPE_ISO14443A)
    {
        PH_CHECK_SUCCESS_FCT(status, phhalHw_Pn5190_Instr_ReadE2Prom(pDataParams, 0x553, &bDigitalDelay, 1));
    }
    /* Get digital delay for Type V */
    else if(pDataParams->bCardType == PHHAL_HW_CARDTYPE_ISO15693)
    {
        PH_CHECK_SUCCESS_FCT(status, phhalHw_Pn5190_Instr_ReadE2Prom(pDataParams, 0x556, &bDigitalDelay, 1));
    }
    else
    {
        /* Do Nothing */
    }

    /* Read T1 Config register to get the prescaler value */
    PH_CHECK_SUCCESS_FCT(status, phhalHw_Pn5190_Instr_ReadRegister(pDataParams, CLIF_TIMER1_CONFIG, &dwTimer1Config));

    if((dwTimer1Config & CLIF_TIMER1_CONFIG_T1_PRESCALE_SEL_MASK) == 0)
    {
        /* Add digital delay */
        dwTmp += ((bDigitalDelay * 8 * 13885U) >> 10U);
    }

    if(pDataParams->bTimeoutUnit == PHHAL_HW_TIME_MICROSECONDS)
    {
        fTime = ((dwTmp / 13.56) - 9.44 /* 1 ETU */);
    }
    else
    {
        fTime = (PHHAL_HW_PN5190_MIN_FREQ  / PHHAL_HW_PN5190_CONVERSION_MS_SEC);

        fTime = ((dwTmp / (fTime / 1000U)));

        /* Subtract obtained PICC response time with platform timer deviation percentage */
        fTime -= ((fTime * pDataParams->bTimerDeviation) / 100);
    }

    /* Return the value */
    *pFdtUs = (uint32_t)fTime;
    if ((float32_t)*pFdtUs < fTime)
    {
        ++(*pFdtUs);
    }

    return PH_ADD_COMPCODE(status, PH_COMP_HAL);
}

phStatus_t phhalHw_Pn5190_Int_ClearNSetRegFields(phhalHw_Pn5190_DataParams_t * pDataParams,
        uint8_t bRegister,uint32_t dwMask,uint32_t dwPos,uint16_t wValue)
{
    uint8_t     PH_MEMLOC_BUF wRegTypeValueSets[12] = {0};
    uint16_t    PH_MEMLOC_REM wSizeOfRegTypeValueSets = 0U;
    uint32_t    PH_MEMLOC_REM dwTemp = 0U;
    phStatus_t  PH_MEMLOC_REM status = PH_ERR_SUCCESS;

    dwTemp = (uint32_t) ~(dwMask);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = bRegister;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = PHHAL_HW_PN5190_WRITE_MULTIPLE_TYPE_WRITE_AND_MASK;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>>8U);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>>16U);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>>24U);

    dwTemp = (uint32_t) ((wValue << dwPos) & dwMask);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = bRegister;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = PHHAL_HW_PN5190_WRITE_MULTIPLE_TYPE_WRITE_OR_MASK;
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>>8U);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>>16U);
    wRegTypeValueSets[wSizeOfRegTypeValueSets++] = (uint8_t)(dwTemp>>24U);

    PH_CHECK_SUCCESS_FCT(status, phhalHw_Pn5190_Instr_WriteRegisterMultiple(pDataParams,  wRegTypeValueSets, wSizeOfRegTypeValueSets));

    return PH_ERR_SUCCESS;
}

/**
 * For PN5190, In ISR read part of response and post
 * appropriate Event depending upon the response
 *
 *  aISRReadBuf[0] = SPI header
 *  aISRReadBuf[1] = Type
 *  aISRReadBuf[2] = Length MSB
 *  aISRReadBuf[3] = Length
 *  aISRReadBuf[4] = Value[0]
 */
void phhalHw_Pn5190_ISR_Callback(void * pDataParams)
{
#ifndef _WIN32
    uint32_t dwEventStatusReg = 0x0U;
    phhalHw_Pn5190_DataParams_t * pPn5190DataParams = NULL;
    pPn5190DataParams = (phhalHw_Pn5190_DataParams_t *) pDataParams;

    if( pPn5190DataParams->sIrqResp.pHandlerModeBuffPtr[1] == PH_PN5190_EVT_RSP )
    {
        dwEventStatusReg = (uint32_t) pPn5190DataParams->sIrqResp.pIsrEvtBuffPtr[4];
        dwEventStatusReg |= ((uint32_t) pPn5190DataParams->sIrqResp.pIsrEvtBuffPtr[5]) << 8U ;
        dwEventStatusReg |= ((uint32_t) pPn5190DataParams->sIrqResp.pIsrEvtBuffPtr[6]) << 16U;
        dwEventStatusReg |= ((uint32_t) pPn5190DataParams->sIrqResp.pIsrEvtBuffPtr[7]) << 24U;

        if ((dwEventStatusReg & (PH_PN5190_EVT_TX_OVERCURRENT_ERROR | PH_PN5190_EVT_CTS)) == 0U)
        {
            /* Received Event can be processed by RdLib */
            (void)phOsal_EventPost(
                &(((phhalHw_Pn5190_DataParams_t *)(pDataParams))->HwEventObj.EventHandle),
                E_OS_EVENT_OPT_POST_ISR,
                E_PH_OSAL_EVT_SIG,
                NULL);
        }
    }
    else
    {
        /*Parse the response and accordingly send event*/
        (void)phOsal_EventPost(
            &(((phhalHw_Pn5190_DataParams_t *)(pDataParams))->HwEventObj.EventHandle),
            E_OS_EVENT_OPT_POST_ISR,
            E_PH_OSAL_EVT_RF,
            NULL);
    }

#else
    /* satisfy compiler */
    if (0U != (pDataParams));
#endif
}

void phhalHw_Pn5190_Int_GuardTimeCallBck(void)
{
#ifndef _WIN32
    if(xEventHandle != NULL)
    {
        (void)phOsal_EventPost(
            &xEventHandle,
            E_OS_EVENT_OPT_POST_ISR,
            E_PH_OSAL_EVT_GT_EXP,
            NULL);
    }
#endif
}

void phhalHw_Pn5190_Int_WriteSSEL(
                                  void *pBalDataParams,
                                  uint8_t bValue
                                  )
{
#ifndef _WIN32
    if (((phbalReg_Type_t *)pBalDataParams)->bBalType == PHBAL_REG_TYPE_SPI)
    {
        phDriver_PinWrite(PHDRIVER_PIN_SSEL, bValue);
    }
#endif
}

phStatus_t phhalHw_Pn5190_Int_UserAbort(
                                        phhalHw_Pn5190_DataParams_t * pDataParams,
                                        uint8_t                     * pCMDAborted
                                        )
{
#ifndef _WIN32
    phStatus_t PH_MEMLOC_REM status = PH_ERR_SUCCESS;
    uint8_t    PH_MEMLOC_REM aCmd[4] = {0};

    aCmd[0]  = PHHAL_HW_PN5190_INT_SPI_WRITE;
    aCmd[1]  = 0x20;
    if (pDataParams->bAbortCMD == PH_PN5190_SWITCH_MODE_NORMAL_2_1)
    {
        aCmd[2]  = 0x00;
    }
    else
    {
        aCmd[2]  = 0x02;
    }
    aCmd[3]  = 0x00;

    phDriver_EnterCriticalSection();

    if (phDriver_IRQPinRead(PHDRIVER_PIN_IRQ) == 0)
    {
#ifndef PH_OSAL_LINUX
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-value"
        __DSB;
        __ISB;
#pragma GCC diagnostic pop
#endif /* PH_OSAL_LINUX */
        status = phhalHw_Pn5190_Send(pDataParams,&aCmd[0], 4, PH_OFF);

        *pCMDAborted = 1;
    }
    phDriver_ExitCriticalSection();

    return status;
#else
    /* satisfy compiler */
    if (0U != (pDataParams));
    return PH_ADD_COMPCODE_FIXED(PH_ERR_UNSUPPORTED_COMMAND, PH_COMP_HAL);
#endif
}

phStatus_t phhalHw_Pn5190_Int_EventWait(
    phhalHw_Pn5190_DataParams_t * pDataParams,
    uint32_t dwExpectedEvents,
    uint32_t dwEventTimeOut,
    uint8_t bPayLoadPresent,
    uint32_t * dwEventReceived)
{
    phStatus_t PH_MEMLOC_REM status;
    uint8_t * PH_MEMLOC_REM pEvtPayload = NULL;
    uint32_t PH_MEMLOC_REM dwEvtReceived = 0x0U;

    status = phhalHw_Pn5190_WaitForEvent(
        pDataParams,
        dwExpectedEvents,
        dwEventTimeOut,
        bPayLoadPresent,
        &dwEvtReceived,
        &pEvtPayload);
    if((status & PH_ERR_MASK) == PH_ERR_SUCCESS)
    {
        if(pEvtPayload != NULL)
        {
            pDataParams->dwEventStatus = (uint32_t) pEvtPayload[0];
            pDataParams->dwEventStatus |= ((uint32_t) pEvtPayload[1]) << 8U ;
            pDataParams->dwEventStatus |= ((uint32_t) pEvtPayload[2]) << 16U;
            pDataParams->dwEventStatus |= ((uint32_t) pEvtPayload[3]) << 24U;

            if(dwEvtReceived & PH_PN5190_EVT_GENERAL_ERROR)
            {
                pDataParams->dwGenError = (uint32_t) pEvtPayload[4];
                pDataParams->dwGenError |= ((uint32_t) pEvtPayload[5]) << 8U ;
                pDataParams->dwGenError |= ((uint32_t) pEvtPayload[6]) << 16U;
                pDataParams->dwGenError |= ((uint32_t) pEvtPayload[7]) << 24U;

                status = PH_ERR_INTERNAL_ERROR;
            }
        }
    }
    else
    {
        if((status & PH_ERR_MASK) == PH_OSAL_IO_TIMEOUT)
        {
            status = PH_ERR_IO_TIMEOUT;
        }
    }

    if(dwEventReceived != NULL)
    {
        * dwEventReceived = dwEvtReceived;
    }
    return PH_ADD_COMPCODE(status, PH_COMP_HAL);
}

#endif  /* NXPBUILD__PHHAL_HW_PN5190 */
