/*----------------------------------------------------------------------------*/
/* Copyright 2009-2020,2022, 2024 NXP                                         */
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
* Generic MIFARE(R) Ultralight contactless IC Application Component of Reader Library Framework.
* $Author$
* $Revision$ (v07.13.00)
* $Date$
*
*/

#include <ph_Status.h>
#include <phalMful.h>
#include <phhalHw.h>
#include <ph_RefDefs.h>

#ifdef NXPBUILD__PHAL_MFUL_SW
#include "Sw/phalMful_Sw.h"
#endif /* NXPBUILD__PHAL_MFUL_SW */

#ifdef NXPBUILD__PHAL_MFUL

#ifndef NXPRDLIB_REM_GEN_INTFS

phStatus_t phalMful_UlcAuthenticate(void * pDataParams, uint8_t bOption, uint16_t wKeyNo, uint16_t wKeyVer, uint8_t * pDivInput,
    uint8_t bDivInputLen)
{
    phStatus_t PH_MEMLOC_REM status;

    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_UlcAuthenticate");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bOption);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(wKeyNo);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(wKeyVer);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pDivInput);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bDivInputLen);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);

    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bOption_log, &bOption);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_DEBUG, wKeyNo_log, &wKeyNo);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_DEBUG, wKeyVer_log, &wKeyVer);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pDivInput_log, pDivInput, bDivInputLen);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bDivInputLen_log, &bDivInputLen);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);

    /* Validate the parameters. */
    PH_ASSERT_NULL_DATA_PARAM (pDataParams, PH_COMP_AL_MFUL);
    if (bDivInputLen) PH_ASSERT_NULL_PARAM (pDivInput, PH_COMP_AL_MFUL);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* Perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_UlcAuthenticate((phalMful_Sw_DataParams_t *)pDataParams, wKeyNo, wKeyVer);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }

    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_Write(
                          void * pDataParams,
                          uint8_t bAddress,
                          uint8_t * pData
                          )
{
    phStatus_t PH_MEMLOC_REM status;

    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_Write");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bAddress);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pData);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bAddress_log, &bAddress);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pData_log, pData, PHAL_MFUL_WRITE_BLOCK_LENGTH);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    PH_ASSERT_NULL (pData);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_Write((phalMful_Sw_DataParams_t *)pDataParams, bAddress, pData);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_FastWrite(
                              void * pDataParams,
                              uint8_t * pData
                              )
{
    phStatus_t PH_MEMLOC_REM status;

    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_FastWrite");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pData);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pData_log, pData, 16U * PHAL_MFUL_WRITE_BLOCK_LENGTH);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    PH_ASSERT_NULL (pData);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_FastWrite((phalMful_Sw_DataParams_t *)pDataParams, pData);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_Read(
                         void * pDataParams,
                         uint8_t bAddress,
                         uint8_t * pData
                         )
{
    phStatus_t PH_MEMLOC_REM status;

    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_Read");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bAddress);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pData);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bAddress_log, &bAddress);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    PH_ASSERT_NULL (pData);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_Read((phalMful_Sw_DataParams_t *)pDataParams, bAddress, pData);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
#ifdef NXPBUILD__PH_LOG
    if ( (status & PH_ERR_MASK) == PH_ERR_SUCCESS)
    {
        PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pData_log, pData, PHAL_MFUL_READ_BLOCK_LENGTH);
    }
#endif
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_CompatibilityWrite(
                                       void * pDataParams,
                                       uint8_t bAddress,
                                       uint8_t * pData
                                       )
{
    phStatus_t PH_MEMLOC_REM status;

    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_CompatibilityWrite");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bAddress);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pData);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bAddress_log, &bAddress);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pData_log, pData, PHAL_MFUL_COMPWRITE_BLOCK_LENGTH);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    PH_ASSERT_NULL (pData);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_CompatibilityWrite((phalMful_Sw_DataParams_t *)pDataParams, bAddress, pData);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_IncrCnt(
                            void * pDataParams,
                            uint8_t bCntNum,
                            uint8_t * pCnt
                            )
{
    phStatus_t PH_MEMLOC_REM status = 0;
    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_IncrCnt");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bCntNum);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pCnt);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bCntNum_log, &bCntNum);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pCnt_log, pCnt, PHAL_MFUL_COUNTER_WR_VALUE_LENGTH);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    PH_ASSERT_NULL (pCnt);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_IncrCnt((phalMful_Sw_DataParams_t *)pDataParams, bCntNum, pCnt);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_ReadCnt(
                            void * pDataParams,
                            uint8_t bCntNum,
                            uint8_t * pCntValue
                            )
{
    phStatus_t PH_MEMLOC_REM status = 0;
    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_ReadCnt");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bCntNum);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pCntValue);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bCntNum_log, &bCntNum);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    PH_ASSERT_NULL (pCntValue);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_ReadCnt((phalMful_Sw_DataParams_t *)pDataParams, bCntNum, pCntValue);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pCntValue_log, pCntValue, PHAL_MFUL_COUNTER_RD_VALUE_LENGTH);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);
    return status;
}

phStatus_t phalMful_PwdAuth(void * pDataParams, uint8_t bOption, uint8_t bKeyNo, uint8_t bKeyVer, uint8_t * pDivInput,
    uint8_t bDivInputLen, uint8_t * pPwd, uint8_t * pPack)
{
    phStatus_t PH_MEMLOC_REM status = 0;

    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_PwdAuth");

    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bOption);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bKeyNo);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bKeyVer);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pDivInput);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bDivInputLen);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pPwd);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pPack);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);

    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bOption_log, &bOption);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bKeyNo_log, &bKeyNo);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bKeyVer_log, &bKeyVer);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pDivInput_log, pDivInput, bDivInputLen);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bDivInputLen_log, &bDivInputLen);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pPwd_log, pPwd, 4);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);

    /* Validate parameters. */
    PH_ASSERT_NULL_DATA_PARAM (pDataParams, PH_COMP_AL_MFUL);
    if (bDivInputLen) PH_ASSERT_NULL_PARAM (pDivInput, PH_COMP_AL_MFUL);
    PH_ASSERT_NULL_PARAM (pPwd, PH_COMP_AL_MFUL);
    PH_ASSERT_NULL_PARAM (pPack, PH_COMP_AL_MFUL);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_PwdAuth((phalMful_Sw_DataParams_t *)pDataParams, pPwd, pPack);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pPack_log, pPack, PHAL_MFUL_PACK_LENGTH);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_GetVersion(
                               void * pDataParams,
                               uint8_t * pVersion
                               )
{
    phStatus_t PH_MEMLOC_REM status = 0;
    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_GetVersion");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pVersion);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    PH_ASSERT_NULL (pVersion);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_GetVersion((phalMful_Sw_DataParams_t *)pDataParams, pVersion);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pVersion_log, pVersion, PHAL_MFUL_VERSION_LENGTH);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);
    return status;
}

phStatus_t phalMful_FastRead(
                             void * pDataParams,
                             uint8_t  bStartAddr,
                             uint8_t bEndAddr,
                             uint8_t ** ppData,
                             uint16_t * pNumBytes
                             )
{
    phStatus_t PH_MEMLOC_REM status;
#ifdef NXPBUILD__PH_LOG
    uint8_t PH_MEMLOC_REM bBytes;
#endif
    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_FastRead");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bStartAddr);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bEndAddr);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(ppData);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pNumBytes);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bBytes);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bStartAddr_log, &bStartAddr);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bEndAddr_log, &bEndAddr);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    PH_ASSERT_NULL (ppData);
    PH_ASSERT_NULL (pNumBytes);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_FastRead((phalMful_Sw_DataParams_t *)pDataParams, bStartAddr, bEndAddr,
            ppData, pNumBytes);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
#ifdef NXPBUILD__PH_LOG
    bBytes = (bEndAddr - bStartAddr + 1U) * PHAL_MFUL_READ_BLOCK_LENGTH;

    if ( (status & PH_ERR_MASK) == PH_ERR_SUCCESS)
    {
        PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bBytes_log, &bBytes);
        PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, ppData_log, *ppData, bBytes);
        PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pNumBytes_log, pNumBytes, 1);
    }
#endif
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_SectorSelect(
                                 void * pDataParams,
                                 uint8_t bSecNo
                                 )
{
    phStatus_t PH_MEMLOC_REM status;
    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_SectorSelect");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);

    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_SectorSelect((phalMful_Sw_DataParams_t *)pDataParams, bSecNo);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_ReadSign(
                             void * pDataParams,
                             uint8_t bAddr,
                             uint8_t ** pSignature
                             )
{
    phStatus_t PH_MEMLOC_REM status = 0;
    uint16_t   PH_MEMLOC_REM wDataLen = 0;
    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_ReadSign");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bAddr);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pSignature);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bAddr_log, &bAddr);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    PH_ASSERT_NULL (pSignature);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_ReadSign((phalMful_Sw_DataParams_t *)pDataParams, bAddr, pSignature, &wDataLen );
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pSignature_log, *pSignature, wDataLen);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_ChkTearingEvent(
                                    void * pDataParams,
                                    uint8_t bCntNum,
                                    uint8_t * pValidFlag
                                    )
{
    phStatus_t PH_MEMLOC_REM status = 0;
    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_ChkTearingEvent");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bCntNum);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pValidFlag);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bCntNum_log, &bCntNum);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    PH_ASSERT_NULL (pValidFlag);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_ChkTearingEvent((phalMful_Sw_DataParams_t *)pDataParams, bCntNum, pValidFlag);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pValidFlag_log, pValidFlag, 1);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);
    return status;
}

phStatus_t phalMful_WriteSign(
                              void * pDataParams,
                              uint8_t bAddress,
                              uint8_t * pSignature
                              )
{
    phStatus_t PH_MEMLOC_REM status = 0;
    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_WriteSign");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bAddress);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pSignature);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bAddress_log, &bAddress);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pSignature_log, pSignature, PHAL_MFUL_WRITE_BLOCK_LENGTH);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    PH_ASSERT_NULL (pSignature);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_WriteSign((phalMful_Sw_DataParams_t *)pDataParams, bAddress, pSignature);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);
    return status;
}

phStatus_t phalMful_LockSign(
                             void * pDataParams,
                             uint8_t bLockMode
                             )
{
    phStatus_t PH_MEMLOC_REM status = 0;
    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_LockSign");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bLockMode);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bLockMode_log, &bLockMode);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_LockSign((phalMful_Sw_DataParams_t *)pDataParams, bLockMode);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);
    return status;
}

phStatus_t phalMful_VirtualCardSelect(
                                      void * pDataParams,
                                      uint8_t * pVCIID,
                                      uint8_t bVCIIDLen,
                                      uint8_t * pVCTID
                                      )
{
    phStatus_t PH_MEMLOC_REM status = 0;
    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_VirtualCardSelect");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pVCIID);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pVCTID);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pVCIID_log, pVCIID, bVCIIDLen);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    PH_ASSERT_NULL (pVCTID);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_VirtualCardSelect((phalMful_Sw_DataParams_t *)pDataParams, pVCIID, bVCIIDLen, pVCTID);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }

    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pVCTID_log, pVCTID, 1);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);
    return status;
}

phStatus_t phalMful_ReadTTStatus(void * pDataParams, uint8_t  bAddr, uint8_t * pData)
{
    phStatus_t PH_MEMLOC_REM status = 0;

    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_ReadTTStatus");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bAddr);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pData);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);

    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bAddr_log, &bAddr);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);

    /* Validate the parameters. */
    PH_ASSERT_NULL_DATA_PARAM (pDataParams, PH_COMP_AL_MFUL);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* Perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_ReadTTStatus((phalMful_Sw_DataParams_t *)pDataParams, bAddr, pData);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
        break;
    }
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pData_log, pData, 5);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

#ifdef NXPBUILD__PH_CRYPTOSYM
phStatus_t phalMful_GetConfig(void * pDataParams, uint16_t wConfig, uint16_t * pValue)
{
    phStatus_t PH_MEMLOC_REM status = 0;

    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_GetConfig");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(wConfig);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pValue);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_DEBUG, wConfig_log, &wConfig);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);

    /* Validate the parameters */
    PH_ASSERT_NULL(pDataParams);
    PH_ASSERT_NULL(pValue);

    /* Check data parameters */
    if(PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);
        return PH_ADD_COMPCODE(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
    }

    /* Perform operation on active layer */
    switch(PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
        case PHAL_MFUL_SW_ID:
            status = phalMful_Sw_GetConfig((phalMful_Sw_DataParams_t *) pDataParams, wConfig, pValue);
            break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

        default:
            status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
            break;
    }

    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
#ifdef NXPBUILD__PH_LOG
    if((status & PH_ERR_MASK) == PH_ERR_SUCCESS)
    {
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_DEBUG, pValue_log, pValue);
    }
#endif
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_SetConfig(void * pDataParams, uint16_t wConfig, uint16_t wValue)
{
    phStatus_t PH_MEMLOC_REM status = 0;

    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_SetConfig");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(wConfig);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(wValue);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_DEBUG, wConfig_log, &wConfig);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_DEBUG, wValue_log, &wValue);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);

    /* Validate the parameters */
    PH_ASSERT_NULL_DATA_PARAM(pDataParams, PH_COMP_AL_MFUL);

    /* Check data parameters */
    if(PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);
        return PH_ADD_COMPCODE(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
    }

    /* Perform operation on active layer */
    switch(PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
        case PHAL_MFUL_SW_ID:
            status = phalMful_Sw_SetConfig((phalMful_Sw_DataParams_t *) pDataParams, wConfig, wValue);
            break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

        default:
            status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
            break;
    }

    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);

    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_AuthenticateAES(void *pDataParams, uint8_t bOption, uint16_t wKeyNo, uint16_t wKeyVer, uint8_t bRamKeyNo, uint8_t bRamKeyVer, uint8_t bKeyNoCard,
    uint8_t * pDivInput, uint8_t bDivLen)
{
    phStatus_t PH_MEMLOC_REM status;

    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_AuthenticatAES");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bOption);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(wKeyNo);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(wKeyVer);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(bKeyNoCard);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pDivInput);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bOption_log, &bOption);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_DEBUG, wKeyNo_log, &wKeyNo);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_DEBUG, wKeyVer_log, &wKeyVer);
    PH_LOG_HELPER_ADDPARAM_UINT8(PH_LOG_LOGTYPE_DEBUG, bKeyNoCard_log, &bKeyNoCard);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pDivInput_log, pDivInput, bDivLen);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);

    /* Verify the parameters */
    PH_ASSERT_NULL_DATA_PARAM(pDataParams, PH_COMP_AL_MFUL);
    if (0U != (bOption)) PH_ASSERT_NULL_PARAM(pDivInput, PH_COMP_AL_MFUL);

    /* Parameter Validation */
    if(PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return PH_ADD_COMPCODE(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
    }

    /* Perform operation on active layer */
    switch(PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
        case PHAL_MFUL_SW_ID:
			status = phalMful_Sw_AuthenticateAES((phalMful_Sw_DataParams_t *)pDataParams, bOption, wKeyNo, wKeyVer, bKeyNoCard, pDivInput,
				bDivLen);
            break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

        default:
            status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
            break;
    }

    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}

phStatus_t phalMful_CalculateSunCMAC(void * pDataParams, uint16_t wKeyNo, uint16_t wKeyVer, uint8_t * pInData, uint16_t wInDataLen, uint8_t * pRespMac)
{

    phStatus_t PH_MEMLOC_REM status;

    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phalMful_CalculateSunCMAC");
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(wKeyNo);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(wKeyVer);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pInData);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(wInDataLen);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pRespMac);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);

    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_DEBUG, wKeyNo_log, &wKeyNo);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_DEBUG, wKeyVer_log, &wKeyVer);

    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_DEBUG, wInDataLen_log, &wInDataLen);
    if (pInData != NULL) PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pInData_log, pInData, wInDataLen);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);

    /* Validate the parameters */
    PH_ASSERT_NULL_DATA_PARAM(pDataParams, PH_COMP_AL_MFUL);
    PH_ASSERT_NULL_PARAM(pInData, PH_COMP_AL_MFUL);

    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_AL_MFUL)
    {
        status = PH_ADD_COMPCODE(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* Perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHAL_MFUL_SW
    case PHAL_MFUL_SW_ID:
        status = phalMful_Sw_CalculateSunCMAC((phalMful_Sw_DataParams_t *)pDataParams,wKeyNo,  wKeyVer,  pInData, wInDataLen, pRespMac);
        break;
#endif /* NXPBUILD__PHAL_MFUL_SW */

        default:
            status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_AL_MFUL);
            break;
    }

    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pRespMac_log, pRespMac, 8);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}
#endif /* NXPBUILD__PH_CRYPTOSYM */
#endif /* NXPRDLIB_REM_GEN_INTFS */
#endif /* NXPBUILD__PHAL_MFUL */
