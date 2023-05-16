//*****************************************************************************
//
//  amdtps_main.c
//! @file
//!
//! @brief This file provides the main application for the AMDTP service.
//!
//
//*****************************************************************************

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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "wsf_buf.h"    //for WsfBufAlloc and WsfBufFree
#include "bstream.h"
#include "att_api.h"
#include "svc_ch.h"
#include "svc_amdtp.h"
#include "amdtps_api.h"
#include "am_util_debug.h"
#include "crc32.h"

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"


//*****************************************************************************
//
// Global variables
//
//*****************************************************************************

uint8_t rxPktBuf[AMDTP_PACKET_SIZE];
uint8_t txPktBuf[AMDTP_PACKET_SIZE];
uint8_t ackPktBuf[20];

extern bool to_swap;

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************

/* Control block */
static struct
{
    amdtpsConn_t            conn[DM_CONN_MAX];      // connection control block
    bool_t                  txReady;                // TRUE if ready to send notifications
    wsfHandlerId_t          appHandlerId;
    AmdtpsCfg_t             cfg;                    // configurable parameters
    amdtpCb_t               core;
} amdtpsCb;

//*****************************************************************************
//
// Connection Open event
//
//*****************************************************************************
static void amdtps_conn_open(dmEvt_t *pMsg)
{
    /*
    hciLeConnCmplEvt_t *evt = (hciLeConnCmplEvt_t*) pMsg;
    AMDTP_INFO(logp("connection opened\n"));
    AMDTP_INFO(logp("role = 0x%x\n", evt->role));
    AMDTP_INFO(logp("addrMSB = %02x%02x%02x%02x%02x%02x\n",evt->peerAddr[0], evt->peerAddr[1], evt->peerAddr[2]));
    AMDTP_INFO(logp("addrLSB = %02x%02x%02x%02x%02x%02x\n",evt->peerAddr[3], evt->peerAddr[4], evt->peerAddr[5]));
    logp("OPN handle = 0x%x,\tconnInterval = 0x%x\n",evt->handle, evt->connInterval);
    logp("OPN connLatency = 0x%x,\tsupTimeout = 0x%x\n",evt->connLatency, evt->supTimeout);
    */
}

//*****************************************************************************
//
// Connection Update event
//
//*****************************************************************************
static void amdtps_conn_update(dmEvt_t *pMsg)
{
    /*
    hciLeConnUpdateCmplEvt_t *evt = (hciLeConnUpdateCmplEvt_t*) pMsg;
    logp("UPDT connection update status = 0x%x\n", evt->status);
    logp("UPDT handle = 0x%x,\tconnInterval = 0x%x\n", evt->handle, evt->connInterval);
    logp("UPDT connLatency = 0x%x,\tsupTimeout = 0x%x\n", evt->connLatency, evt->supTimeout);
    */
}

static void amdtpsSetupToSend(void)
{
    amdtpsConn_t    *pConn = amdtpsCb.conn;
    uint8_t       i;

    for (i = 0; i < DM_CONN_MAX; i++, pConn++)
    {
        if (pConn->connId != DM_CONN_ID_NONE)
        {
            pConn->amdtpToSend = TRUE;
        }
    }
}

//*****************************************************************************
//
// Find Next Connection to Send on
//
//*****************************************************************************
static amdtpsConn_t* amdtps_find_next2send(void)
{
    amdtpsConn_t *pConn = amdtpsCb.conn;
    return pConn;
}

//*****************************************************************************
//
// Send Notification to Client
//
//*****************************************************************************
static void amdtpsSendData(uint8_t *buf, uint16_t len)
{
    amdtpsSetupToSend();
    amdtpsConn_t *pConn = amdtps_find_next2send();
    /* send notification */
    if (pConn)
    {   
        // AMDTP_TEST_ACK
        if(to_swap)
        {
            // Swap bytes to break CRC.
            buf[10] ^= buf[9];
            buf[9]  ^= buf[10];
            buf[10] ^= buf[9];
        }
        AttsHandleValueNtf(pConn->connId, AMDTPS_TX_HDL, len, buf);
        if(to_swap)
        {
            // Swap back to its original.
            buf[10] ^= buf[9];
            buf[9]  ^= buf[10];
            buf[10] ^= buf[9];
            to_swap = false;
        }

        pConn->amdtpToSend = false;
        amdtpsCb.txReady = false;
    }
}

static eAmdtpStatus_t amdtpsSendAck(eAmdtpPktType_t type, bool_t encrypted, bool_t enableACK, uint8_t *buf, uint16_t len)
{
    AmdtpBuildPkt(&amdtpsCb.core, type, encrypted, enableACK, buf, len);
    // send packet
    amdtpsSetupToSend();
    amdtpsConn_t *pConn = amdtps_find_next2send();
    /* send notification */
    if (pConn)
    {
        AttsHandleValueNtf(pConn->connId, AMDTPS_ACK_HDL, amdtpsCb.core.ackPkt.len, amdtpsCb.core.ackPkt.data);
        pConn->amdtpToSend = false;
    }
    else
    {
        return AMDTP_STATUS_TX_NOT_READY;
    }

    return AMDTP_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Timer Expiration handler
//
//*****************************************************************************
static void amdtps_timeout_timer_expired(wsfMsgHdr_t *pMsg)
{
    uint8_t data[] = { amdtpsCb.core.txPktSn };
    //loge(ERR_BT_APOLLO3_AMDTP_TX_TMO, "txPktSn = %d", amdtpsCb.core.txPktSn);
    AmdtpSendControl(&amdtpsCb.core, AMDTP_CONTROL_RESEND_REQ, data, sizeof(data));
    // fire a timer for receiving an AMDTP_STATUS_RESEND_REPLY ACK
    WsfTimerStartMs(&amdtpsCb.core.timeoutTimer, amdtpsCb.core.txTimeoutMs);
}

/*************************************************************************************************/
/*!
 *  \fn     amdtpsHandleValueCnf
 *
 *  \brief  Handle a received ATT handle value confirm.
 *
 *  \param  pMsg     Event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amdtpsHandleValueCnf(attEvt_t *pMsg)
{
    if (pMsg->hdr.status == ATT_SUCCESS)
    {
        if (pMsg->handle == AMDTPS_TX_HDL)
        {
            amdtpsCb.txReady = true;
            // process next data
            AmdtpSendPacketHandler(&amdtpsCb.core);
        }
    }
}

//*****************************************************************************
//
//! @brief initialize amdtp service
//!
//! @param handlerId - connection handle
//! @param pCfg - configuration parameters
//!
//! @return None
//
//*****************************************************************************
void amdtps_init(wsfHandlerId_t handlerId, AmdtpsCfg_t *pCfg, amdtpRecvCback_t recvCback, 
                amdtpTransCback_t transCback, amdtpNotfiyEnabledCback_t notifyCback)
{    
    memset(&amdtpsCb, 0, sizeof(amdtpsCb));
    amdtpsCb.appHandlerId = handlerId;
    amdtpsCb.txReady = false;
    amdtpsCb.core.txState = AMDTP_STATE_INIT;
    amdtpsCb.core.rxState = AMDTP_STATE_RX_IDLE;
    amdtpsCb.core.timeoutTimer.handlerId = handlerId;
    for (int i = 0; i < DM_CONN_MAX; i++)
    {
        amdtpsCb.conn[i].connId = DM_CONN_ID_NONE;
    }

    amdtpsCb.core.lastRxPktSn = 0;
    amdtpsCb.core.txPktSn = 0;

    resetPkt(&amdtpsCb.core.rxPkt);
    amdtpsCb.core.rxPkt.data = rxPktBuf;

    resetPkt(&amdtpsCb.core.txPkt);
    amdtpsCb.core.txPkt.data = txPktBuf;

    resetPkt(&amdtpsCb.core.ackPkt);
    amdtpsCb.core.ackPkt.data = ackPktBuf;

    amdtpsCb.core.recvCback = recvCback;
    amdtpsCb.core.transCback = transCback;
    amdtpsCb.core.notifyCback = notifyCback;

    amdtpsCb.core.txTimeoutMs = TX_TIMEOUT_DEFAULT;

    amdtpsCb.core.data_sender_func = amdtpsSendData;
    amdtpsCb.core.ack_sender_func = amdtpsSendAck;
}

static void amdtps_conn_close(dmEvt_t *pMsg)
{
    // notify ble_data
    //ble_data_reset();
    
    hciDisconnectCmplEvt_t *evt = (hciDisconnectCmplEvt_t*) pMsg;
    dmConnId_t connId = evt->hdr.param;
    /* clear connection */
    amdtpsCb.conn[connId - 1].connId = DM_CONN_ID_NONE;
    amdtpsCb.conn[connId - 1].amdtpToSend = FALSE;

    WsfTimerStop(&amdtpsCb.core.timeoutTimer);
    amdtpsCb.core.txState = AMDTP_STATE_INIT;
    amdtpsCb.core.rxState = AMDTP_STATE_RX_IDLE;
    amdtpsCb.core.lastRxPktSn = 0;
    amdtpsCb.core.txPktSn = 0;
    resetPkt(&amdtpsCb.core.rxPkt);
    resetPkt(&amdtpsCb.core.txPkt);
    resetPkt(&amdtpsCb.core.ackPkt);
}

uint8_t amdtps_write_cback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                   uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
    eAmdtpStatus_t status = AMDTP_STATUS_UNKNOWN_ERROR;
    amdtpPacket_t *pkt = NULL;
    if (handle == AMDTPS_RX_HDL)
    {
        status = AmdtpReceivePkt(&amdtpsCb.core, &amdtpsCb.core.rxPkt, len, pValue);
    }
    else if (handle == AMDTPS_ACK_HDL)
    {
        status = AmdtpReceivePkt(&amdtpsCb.core, &amdtpsCb.core.ackPkt, len, pValue);
    }

    if (status == AMDTP_STATUS_RECEIVE_DONE)
    {
        if (handle == AMDTPS_RX_HDL)
        {
            pkt = &amdtpsCb.core.rxPkt;
        }
        else if (handle == AMDTPS_ACK_HDL)
        {
            pkt = &amdtpsCb.core.ackPkt;
        }

        AmdtpPacketHandler(&amdtpsCb.core, (eAmdtpPktType_t)pkt->header.pktType, pkt->len - AMDTP_CRC_SIZE_IN_PKT, pkt->data);
    }
    else if(AMDTP_STATUS_RECEIVE_CONTINUE != status && AMDTP_STATUS_SUCCESS != status)
    {
        // We errored, log it.
        //loge(ERR_BT_APOLLO3_AMDTP_RX, "%d", status);
    }

    return ATT_SUCCESS;
}

void amdtps_start(dmConnId_t connId, uint8_t timerEvt, uint8_t amdtpCccIdx)
{
    // set conn id
    amdtpsCb.conn[connId - 1].connId = connId;
    amdtpsCb.conn[connId - 1].amdtpToSend = TRUE;

    amdtpsCb.core.timeoutTimer.msg.event = timerEvt;
    amdtpsCb.core.txState = AMDTP_STATE_TX_IDLE;
    amdtpsCb.txReady = true;

    amdtpsCb.core.attMtuSize = AttGetMtu(connId);

    // Notify Application level
    amdtpsCb.core.notifyCback();
    //logp("MTU size = %d bytes\n", amdtpsCb.core.attMtuSize);
}

void amdtps_stop(dmConnId_t connId)
{
    // clear connection
    amdtpsCb.conn[connId - 1].connId = DM_CONN_ID_NONE;
    amdtpsCb.conn[connId - 1].amdtpToSend = FALSE;

    amdtpsCb.core.txState = AMDTP_STATE_INIT;
    amdtpsCb.txReady = false;
}


void amdtps_proc_msg(wsfMsgHdr_t *pMsg)
{
    if (pMsg->event == DM_CONN_OPEN_IND)
    {
        amdtps_conn_open((dmEvt_t *) pMsg);
    }
    else if (pMsg->event == DM_CONN_CLOSE_IND)
    {
        amdtps_conn_close((dmEvt_t *) pMsg);
    }
    else if (pMsg->event == DM_CONN_UPDATE_IND)
    {
        amdtps_conn_update((dmEvt_t *) pMsg);
    }
    else if (pMsg->event == amdtpsCb.core.timeoutTimer.msg.event)
    {
        amdtps_timeout_timer_expired(pMsg);
    }
    else if (pMsg->event == ATTS_HANDLE_VALUE_CNF)
    {
        amdtpsHandleValueCnf((attEvt_t *) pMsg);
    }
}

//*****************************************************************************
//
//! @brief Send data to Client via notification
//!
//! @param type - packet type
//! @param encrypted - is packet encrypted
//! @param enableACK - does client need to response
//! @param buf - data
//! @param len - data length
//!
//! @return status
//
//*****************************************************************************
eAmdtpStatus_t AmdtpsSendPacket(eAmdtpPktType_t type, bool_t encrypted, bool_t enableACK, uint8_t *buf, uint16_t len)
{
    // Check if ready to send notification
    if ( !amdtpsCb.txReady )
    {
        //set in callback amdtpsHandleValueCnf
        return AMDTP_STATUS_TX_NOT_READY;
    }

    // Check if the service is idle to send
    if ( amdtpsCb.core.txState != AMDTP_STATE_TX_IDLE )
    {
        return AMDTP_STATUS_BUSY;
    }

    // Check if data length is valid
    if ( len > AMDTP_MAX_PAYLOAD_SIZE )
    {
        return AMDTP_STATUS_INVALID_PKT_LENGTH;
    }

    AmdtpBuildPkt(&amdtpsCb.core, type, encrypted, enableACK, buf, len);
    AmdtpSendPacketHandler(&amdtpsCb.core);
    return AMDTP_STATUS_SUCCESS;
}
