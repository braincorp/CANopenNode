/*
 * CANopen Multiplexed Process Data Object (MPDO) protocol — CiA 301 §7.2.5.
 *
 * @file        CO_MPDO.c
 * @ingroup     CO_MPDO
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include <string.h>

#include "301/CO_MPDO.h"

#if (CO_CONFIG_MPDO) != 0

#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_RX_DAM | CO_CONFIG_MPDO_RX_SAM))
/*
 * Per-frame RX callback. Runs on whatever context CO_CANprocessCANread runs
 * from — task context on both CP (NuttX signal timer thread) and VC (main
 * loop). Defers OD work to CO_process_MPDO_RX so that handling stays uniform
 * with regular RPDOs.
 */
static void CO_MPDO_receive(void *object, void *msg) {
    CO_MPDO_rx_t *rx = (CO_MPDO_rx_t *) object;
    uint8_t DLC = CO_CANrxMsg_readDLC(msg);
    uint8_t *data = CO_CANrxMsg_readData(msg);

    /* MPDO frames are always 8 bytes (CiA 301 §7.2.5). Drop runts. */
    if (!rx->valid || DLC != 8U) {
        return;
    }

    memcpy(rx->CANrxData, data, sizeof(rx->CANrxData));
    CO_FLAG_SET(rx->CANrxNew);
}
#endif


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_DAM)
/*
 * Apply one DAM frame to the OD. Returns true if the write succeeded so the
 * caller can decide whether to emit EMCY for spec-mandated failures.
 */
static bool_t CO_MPDO_applyDAM(CO_MPDO_t *MPDO, const uint8_t *frame) {
    uint8_t byte0 = frame[0];

    /* DAM iff bit 7 is clear. */
    if ((byte0 & CO_MPDO_BYTE0_SAM_MASK) != 0U) {
        return false;
    }

    uint8_t targetNodeId = byte0 & CO_MPDO_BYTE0_NODEID_MASK;

    /* DAM accepts broadcast (nodeId == 0) and our own node ID. */
    if (targetNodeId != CO_MPDO_DAM_BROADCAST
        && targetNodeId != MPDO->ownNodeId
    ) {
        return true; /* not for us — silently ignore, not an error */
    }

    uint16_t idx = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);
    uint8_t  sub = frame[3];

    /* CiA 301 §7.2.5.2: target must be PDO-writable (ODA_RPDO). */
    OD_entry_t *entry = OD_find(MPDO->OD, idx);
    if (entry == NULL) {
        CO_errorReport(MPDO->em, CO_EM_RPDO_WRONG_LENGTH,
                       CO_EMC_DAM_MPDO, ((uint32_t)idx << 16) | ((uint32_t)sub << 8));
        return false;
    }

    OD_IO_t io;
    ODR_t odRet = OD_getSub(entry, sub, &io, false);
    if (odRet != ODR_OK) {
        CO_errorReport(MPDO->em, CO_EM_RPDO_WRONG_LENGTH,
                       CO_EMC_DAM_MPDO, ((uint32_t)idx << 16) | ((uint32_t)sub << 8));
        return false;
    }

    if ((io.stream.attribute & ODA_RPDO) == 0U) {
        CO_errorReport(MPDO->em, CO_EM_RPDO_WRONG_LENGTH,
                       CO_EMC_DAM_MPDO, ((uint32_t)idx << 16) | ((uint32_t)sub << 8));
        return false;
    }

    /* DAM payload is the entire 4-byte tail. Spec: the destination OD entry
     * must have exactly the length the producer is writing. Without a length
     * field on the wire, we treat the OD entry length as authoritative and
     * write exactly that many bytes, capped at 4. Anything longer than 4 is
     * a violation of the DAM contract. */
    OD_size_t writeLen = io.stream.dataLength;
    if (writeLen == 0U || writeLen > 4U) {
        CO_errorReport(MPDO->em, CO_EM_RPDO_WRONG_LENGTH,
                       CO_EMC_DAM_MPDO, ((uint32_t)idx << 16) | ((uint32_t)sub << 8));
        return false;
    }

    /* Reset stream offset before each write — io->write may be an extension. */
    io.stream.dataOffset = 0;
    OD_size_t countWritten = 0;

    CO_LOCK_OD(MPDO->CANdev);
    ODR_t writeRet = io.write(&io.stream, &frame[4], writeLen, &countWritten);
    CO_UNLOCK_OD(MPDO->CANdev);

    if (writeRet != ODR_OK || countWritten != writeLen) {
        CO_errorReport(MPDO->em, CO_EM_RPDO_WRONG_LENGTH,
                       CO_EMC_DAM_MPDO, ((uint32_t)idx << 16) | ((uint32_t)sub << 8));
        return false;
    }

    return true;
}
#endif /* (CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_DAM */


/******************************************************************************/
CO_ReturnError_t CO_MPDO_init(CO_MPDO_t *MPDO,
                              OD_t *OD,
                              CO_EM_t *em,
                              CO_CANmodule_t *CANdev,
                              uint8_t ownNodeId,
                              uint16_t rxIdxBase,
                              uint16_t txIdxBase)
{
    if (MPDO == NULL || OD == NULL || em == NULL || CANdev == NULL
        || ownNodeId < 1U || ownNodeId > 127U
    ) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    memset(MPDO, 0, sizeof(*MPDO));
    MPDO->OD = OD;
    MPDO->em = em;
    MPDO->CANdev = CANdev;
    MPDO->ownNodeId = ownNodeId;
#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_RX_DAM | CO_CONFIG_MPDO_RX_SAM))
    MPDO->rxIdxBase = rxIdxBase;
#else
    (void) rxIdxBase;
#endif
#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_TX_DAM | CO_CONFIG_MPDO_TX_SAM))
    MPDO->txIdxBase = txIdxBase;
#else
    (void) txIdxBase;
#endif

    return CO_ERROR_NO;
}


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_DAM)
/******************************************************************************/
CO_ReturnError_t CO_MPDO_configRX_DAM(CO_MPDO_t *MPDO,
                                      uint8_t slotIdx,
                                      uint16_t canId)
{
    if (MPDO == NULL || slotIdx >= CO_CONFIG_MPDO_RX_COUNT || canId == 0U
        || (canId & ~0x7FFU) != 0U
    ) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    CO_MPDO_rx_t *rx = &MPDO->rx[slotIdx];
    rx->canId = canId;
    CO_FLAG_CLEAR(rx->CANrxNew);

    CO_ReturnError_t ret = CO_CANrxBufferInit(MPDO->CANdev,
                                              MPDO->rxIdxBase + slotIdx,
                                              canId,
                                              0x7FFU,
                                              0,
                                              (void *) rx,
                                              CO_MPDO_receive);
    if (ret == CO_ERROR_NO) {
        rx->valid = true;
    }
    return ret;
}
#endif


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_DAM)
/******************************************************************************/
CO_ReturnError_t CO_MPDO_configTX_DAM(CO_MPDO_t *MPDO,
                                      uint8_t slotIdx,
                                      uint16_t canId,
                                      uint32_t inhibitTime_us)
{
    if (MPDO == NULL || slotIdx >= CO_CONFIG_MPDO_TX_COUNT || canId == 0U
        || (canId & ~0x7FFU) != 0U
    ) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    CO_MPDO_tx_t *tx = &MPDO->tx[slotIdx];
    tx->CANtxBuff = CO_CANtxBufferInit(MPDO->CANdev,
                                       MPDO->txIdxBase + slotIdx,
                                       canId,
                                       0,
                                       8,
                                       false);
    if (tx->CANtxBuff == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    tx->inhibitTime_us = inhibitTime_us;
    tx->inhibitTimer = 0;
    tx->sendRequest = false;
    tx->valid = true;
    return CO_ERROR_NO;
}


/******************************************************************************/
CO_ReturnError_t CO_MPDO_send_DAM(CO_MPDO_t *MPDO,
                                  uint8_t slotIdx,
                                  uint8_t targetNodeId,
                                  uint16_t idx,
                                  uint8_t sub,
                                  const void *data,
                                  uint8_t len)
{
    if (MPDO == NULL || slotIdx >= CO_CONFIG_MPDO_TX_COUNT
        || targetNodeId > 127U || data == NULL || len == 0U || len > 4U
    ) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    CO_MPDO_tx_t *tx = &MPDO->tx[slotIdx];
    if (!tx->valid) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    if (tx->sendRequest) {
        return CO_ERROR_TX_OVERFLOW;
    }

    /* Build the 8-byte payload. Unused tail bytes are zeroed. */
    uint8_t *frame = tx->pendingFrame;
    frame[0] = targetNodeId & CO_MPDO_BYTE0_NODEID_MASK; /* DAM: bit 7 = 0 */
    frame[1] = (uint8_t)(idx & 0xFFU);
    frame[2] = (uint8_t)(idx >> 8);
    frame[3] = sub;
    memset(&frame[4], 0, 4);
    memcpy(&frame[4], data, len);

    tx->sendRequest = true;
    return CO_ERROR_NO;
}
#endif /* (CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_DAM */


#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_RX_DAM | CO_CONFIG_MPDO_RX_SAM))
/******************************************************************************/
void CO_MPDO_processRX(CO_MPDO_t *MPDO) {
    if (MPDO == NULL) {
        return;
    }

    for (uint8_t i = 0; i < CO_CONFIG_MPDO_RX_COUNT; i++) {
        CO_MPDO_rx_t *rx = &MPDO->rx[i];
        if (!rx->valid || !CO_FLAG_READ(rx->CANrxNew)) {
            continue;
        }

        uint8_t frame[8];
        memcpy(frame, rx->CANrxData, sizeof(frame));
        CO_FLAG_CLEAR(rx->CANrxNew);

#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_DAM)
        (void) CO_MPDO_applyDAM(MPDO, frame);
#else
        (void) frame; /* SAM-only build: stub for now */
#endif
    }
}
#endif


#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_TX_DAM | CO_CONFIG_MPDO_TX_SAM))
/******************************************************************************/
void CO_MPDO_processTX(CO_MPDO_t *MPDO,
                       uint32_t timeDifference_us,
                       uint32_t *timerNext_us)
{
    (void) timerNext_us;
    if (MPDO == NULL) {
        return;
    }

#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_DAM)
    for (uint8_t i = 0; i < CO_CONFIG_MPDO_TX_COUNT; i++) {
        CO_MPDO_tx_t *tx = &MPDO->tx[i];
        if (!tx->valid) {
            continue;
        }

        if (tx->inhibitTimer > timeDifference_us) {
            tx->inhibitTimer -= timeDifference_us;
        } else {
            tx->inhibitTimer = 0;
        }

        if (tx->sendRequest && tx->inhibitTimer == 0U) {
            memcpy(&tx->CANtxBuff->data[0], tx->pendingFrame, 8);
            CO_ReturnError_t ret = CO_CANsend(MPDO->CANdev, tx->CANtxBuff);
            if (ret == CO_ERROR_NO) {
                tx->sendRequest = false;
                tx->inhibitTimer = tx->inhibitTime_us;
            }
            /* On TX overflow, leave sendRequest set so the next tick retries. */
        }

        if (timerNext_us != NULL && tx->sendRequest
            && *timerNext_us > tx->inhibitTimer
        ) {
            *timerNext_us = tx->inhibitTimer;
        }
    }
#endif
}
#endif

#endif /* (CO_CONFIG_MPDO) != 0 */
