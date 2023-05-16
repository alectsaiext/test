// ****************************************************************************
//
//  amdtp_main.c
//! @file
//!
//! @brief Ambiq Micro's demonstration of AMDTP service.
//!
//! @{
//
// ****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2018, Ambiq Micro
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
// 
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision v1.2.12-1330-gad755f6e5 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <string.h>
#include "wsf_types.h"
#include "bstream.h"
#include "wsf_msg.h"
#include "hci_api.h"
#include "dm_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "app_db.h"
#include "app_hw.h"
#include "svc_ch.h"
#include "svc_core.h"
#include "svc_dis.h"
#include "amdtp_api.h"
#include "amdtps_api.h"
#include "svc_amdtp.h"

#include "am_util.h"
#include "assert.h"
/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! WSF message event starting value */
#define AMDTP_MSG_START               0xA0

/*! WSF message event enumeration */
enum
{
  AMDTP_TIMER_IND = AMDTP_MSG_START,  /*! AMDTP tx timeout timer expired */
};

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! Application message type */
typedef union
{
  wsfMsgHdr_t     hdr;
  dmEvt_t         dm;
  attsCccEvt_t    ccc;
  attEvt_t        att;
} amdtpMsg_t;

static void amdtpClose(amdtpMsg_t *pMsg);
/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static const appAdvCfg_t amdtpAdvCfg =
{
  {0, 0, 0},                  /*! Advertising durations in ms */
  {0, 0, 0}                   /*! Advertising intervals in 0.625 ms units */
};

/*! configurable parameters for slave */
static const appSlaveCfg_t amdtpSlaveCfg =
{
  AMDTP_CONN_MAX,                           /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t amdtpSecCfg =
{
  DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG,    /*! Authentication and bonding flags */
  0,                                      /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK,                        /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  FALSE                                   /*! TRUE to initiate security upon connection */
};

/*! configurable parameters for AMDTP connection parameter update */
static const appUpdateCfg_t amdtpUpdateCfg =
{
  0,                                      /*! Connection idle period in ms before attempting
                                              connection parameter update; set to zero to disable */
  6,
  15,
  0,                                      /*! Connection latency */
  600,                                    /*! Supervision timeout in 10ms units */
  5                                       /*! Number of update attempts before giving up */
};

/*! SMP security parameter configuration */
static const smpCfg_t amdtpSmpCfg =
{
  3000,                                   /*! 'Repeated attempts' timeout in msec */
  SMP_IO_NO_IN_NO_OUT,                    /*! I/O Capability */
  7,                                      /*! Minimum encryption key length */
  16,                                     /*! Maximum encryption key length */
  3,                                      /*! Attempts to trigger 'repeated attempts' timeout */
  0,                                      /*! Device authentication requirements */
};

/*! AMDTPS configuration */
static const AmdtpsCfg_t amdtpAmdtpsCfg =
{
    0
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  AMDTP_GATT_SC_CCC_IDX,                  /*! GATT service, service changed characteristic */
  AMDTP_AMDTPS_TX_CCC_IDX,                /*! AMDTP service, tx characteristic */
  AMDTP_AMDTPS_RX_ACK_CCC_IDX,            /*! AMDTP service, rx ack characteristic */
  AMDTP_NUM_CCC_IDX
};

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t amdtpCccSet[AMDTP_NUM_CCC_IDX] =
{
  /* cccd handle                value range               security level */
  {GATT_SC_CH_CCC_HDL,          ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},   /* AMDTP_GATT_SC_CCC_IDX */
  {AMDTPS_TX_CH_CCC_HDL,        ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* AMDTP_AMDTPS_TX_CCC_IDX */
  {AMDTPS_ACK_CH_CCC_HDL,       ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE}    /* AMDTP_AMDTPS_RX_ACK_CCC_IDX */
};

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! WSF handler ID */
wsfHandlerId_t amdtpHandlerId;

/*************************************************************************************************/
/*!
 *  \fn     amdtpDmCback
 *
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amdtpDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t *pMsg;
  uint16_t len;

  len = DmSizeOfEvt(pDmEvt);

  if ((pMsg = WsfMsgAlloc(len)) != NULL)
  {
    memcpy(pMsg, pDmEvt, len);
    WsfMsgSend(amdtpHandlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     amdtpAttCback
 *
 *  \brief  Application ATT callback.
 *
 *  \param  pEvt    ATT callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amdtpAttCback(attEvt_t *pEvt)
{
  attEvt_t *pMsg;

  if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attEvt_t));
    pMsg->pValue = (uint8_t *) (pMsg + 1);
    memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
    WsfMsgSend(amdtpHandlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     amdtpCccCback
 *
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amdtpCccCback(attsCccEvt_t *pEvt)
{
  attsCccEvt_t  *pMsg;
  appDbHdl_t    dbHdl;

  /* if CCC not set from initialization and there's a device record */
  if ((pEvt->handle != ATT_HANDLE_NONE) &&
      ((dbHdl = AppDbGetHdl((dmConnId_t) pEvt->hdr.param)) != APP_DB_HDL_NONE))
  {
    /* store value in device database */
    AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
  }

  if ((pMsg = WsfMsgAlloc(sizeof(attsCccEvt_t))) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attsCccEvt_t));
    WsfMsgSend(amdtpHandlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     amdtpProcCccState
 *
 *  \brief  Process CCC state change.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
//static bool bPairingCompleted = false;
static void amdtpProcCccState(amdtpMsg_t *pMsg)
{
  //logp("ccc state ind value:%d handle:%d idx:%d\n", pMsg->ccc.value, pMsg->ccc.handle, pMsg->ccc.idx);

  /* AMDTPS TX CCC */
  if (pMsg->ccc.idx == AMDTP_AMDTPS_TX_CCC_IDX)
  {
    if (pMsg->ccc.value == ATT_CLIENT_CFG_NOTIFY)
    {
      // notify enabled
      amdtps_start((dmConnId_t)pMsg->ccc.hdr.param, AMDTP_TIMER_IND, AMDTP_AMDTPS_TX_CCC_IDX);
    }
    else
    {
      // notify disabled
      amdtpClose(pMsg);
      amdtps_stop((dmConnId_t)pMsg->ccc.hdr.param);
    }
    return;
  }
}

/*************************************************************************************************/
/*!
 *  \fn     amdtpClose
 *
 *  \brief  Perform UI actions on connection close.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amdtpClose(amdtpMsg_t *pMsg)
{
}

/*************************************************************************************************/
/*!
 *  \fn     amdtpProcMsg
 *
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amdtpProcMsg(amdtpMsg_t *pMsg)
{
  //uint8_t uiEvent = APP_UI_NONE;

  switch(pMsg->hdr.event)
  {
    case AMDTP_TIMER_IND:
      amdtps_proc_msg(&pMsg->hdr);
      break;

    case ATTS_HANDLE_VALUE_CNF:
      //AMDTP_TRACE("ATTS_HANDLE_VALUE_CNF, status = %d", pMsg->att.hdr.status);
      
      amdtps_proc_msg(&pMsg->hdr);
      break;

    case ATTS_CCC_STATE_IND:
      amdtpProcCccState(pMsg);
      break;

    case ATT_MTU_UPDATE_IND:
      //logp("Negotiated MTU %d\n", ((attEvt_t *)pMsg)->mtu);
      break;

    case DM_CONN_DATA_LEN_CHANGE_IND:
      //logp("DM_CONN_DATA_LEN_CHANGE_IND, Tx=%d, Rx=%d\n",((hciLeDataLenChangeEvt_t*)pMsg)->maxTxOctets,((hciLeDataLenChangeEvt_t*)pMsg)->maxRxOctets);
      break;
    case DM_RESET_CMPL_IND:
      //DmSecGenerateEccKeyReq();
      //uiEvent = APP_UI_RESET_CMPL;
      break;

    case DM_ADV_START_IND:
      //uiEvent = APP_UI_ADV_START;
      break;

    case DM_ADV_STOP_IND:
      //uiEvent = APP_UI_ADV_STOP;
      break;

    case DM_CONN_OPEN_IND:
      amdtps_proc_msg(&pMsg->hdr);
      break;

    case DM_CONN_CLOSE_IND:
      amdtpClose(pMsg);
      amdtps_proc_msg(&pMsg->hdr);

      //AMDTP_INFO(logp("DM_CONN_CLOSE_IND, reason = 0x%x\n", pMsg->dm.connClose.reason));

      //uiEvent = APP_UI_CONN_CLOSE;
      break;

    case DM_CONN_UPDATE_IND:
      amdtps_proc_msg(&pMsg->hdr);
      break;

    case DM_SEC_PAIR_CMPL_IND:
      //uiEvent = APP_UI_SEC_PAIR_CMPL;

      //bPairingCompleted = true;

      break;

    case DM_SEC_PAIR_FAIL_IND:
      //AMDTP_INFO(logp("DM_SEC_PAIR_FAIL_IND, status = 0x%x\n", pMsg->dm.pairCmpl.hdr.status));
      //bPairingCompleted = false;

      //uiEvent = APP_UI_SEC_PAIR_FAIL;
      break;

    case DM_SEC_ENCRYPT_IND:
      //uiEvent = APP_UI_SEC_ENCRYPT;
      break;

    case DM_SEC_ENCRYPT_FAIL_IND:
      //uiEvent = APP_UI_SEC_ENCRYPT_FAIL;
      break;

    case DM_SEC_AUTH_REQ_IND:
      AppHandlePasskey(&pMsg->dm.authReq);
      break;

    case DM_SEC_ECC_KEY_IND:
      DmSecSetEccKey(&pMsg->dm.eccMsg.data.key);
      //amdtpSetup(pMsg);
      break;

    case DM_SEC_COMPARE_IND:
      AppHandleNumericComparison(&pMsg->dm.cnfInd);
      break;

    default:
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \fn     AmdtpHandlerInit
 *
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmdtpHandlerInit(wsfHandlerId_t handlerId, 
                      amdtpRecvCback_t recvCback, 
                      amdtpTransCback_t transCback, 
                      amdtpNotfiyEnabledCback_t notifyCback
                      )
{
  //connet app callback interface to callback passed in by developer
  assert(notifyCback && transCback && recvCback);

   /* store handler ID */
  amdtpHandlerId = handlerId;

  /* Set configuration pointers */
  pAppAdvCfg = (appAdvCfg_t *) &amdtpAdvCfg;
  pAppSlaveCfg = (appSlaveCfg_t *) &amdtpSlaveCfg;
  pAppSecCfg = (appSecCfg_t *) &amdtpSecCfg;
  pAppUpdateCfg = (appUpdateCfg_t *) &amdtpUpdateCfg;

  /* Initialize application framework */
  AppSlaveInit();

  /* Set stack configuration pointers */
  pSmpCfg = (smpCfg_t *) &amdtpSmpCfg;

  /* initialize amdtp service server */
  amdtps_init(handlerId, (AmdtpsCfg_t *) &amdtpAmdtpsCfg, recvCback, transCback, notifyCback);
}

/*************************************************************************************************/
/*!
 *  \fn     AmdtpHandler
 *
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmdtpHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  if (pMsg != NULL)
  {
    //AMDTP_TRACE(logp("Amdtp got evt 0x%02X\n", pMsg->event));

    if (pMsg->event >= DM_CBACK_START && pMsg->event <= DM_CBACK_END)
    {
      /* process advertising and connection-related messages */
      AppSlaveProcDmMsg((dmEvt_t *) pMsg);

      /* process security-related messages */
      AppSlaveSecProcDmMsg((dmEvt_t *) pMsg);
    }

    /* perform profile and user interface-related operations */
    amdtpProcMsg((amdtpMsg_t *) pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     AmdtpStart
 *
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmdtpStart(void)
{
  /* Register for stack callbacks */
  DmRegister(amdtpDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, amdtpDmCback);
  AttRegister(amdtpAttCback);
  AttConnRegister(AppServerConnCback);
  AttsCccRegister(AMDTP_NUM_CCC_IDX, (attsCccSet_t *) amdtpCccSet, amdtpCccCback);

  /* Initialize attribute server database */
  SvcCoreAddGroup();
  SvcDisAddGroup();
  SvcAmdtpsCbackRegister(NULL, amdtps_write_cback);
  SvcAmdtpsAddGroup();

  /* Reset the device */
  DmDevReset();
}
