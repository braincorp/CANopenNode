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


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_SAM)
/*
 * Apply one SAM frame to the local OD.
 *
 * CiA 301 §7.2.5.3 — byte 0 bit 7 set, low 7 bits = producer node ID; the
 * dispatcher table re-maps (producerNodeId, srcIdx, srcSub) to a local
 * (dstIdx, dstSub). Frames with no matching row are silently dropped; only
 * downstream OD-write failures raise EMCY.
 */
static bool_t CO_MPDO_applySAM(CO_MPDO_t *MPDO, const uint8_t *frame) {
    uint8_t byte0 = frame[0];

    /* SAM iff bit 7 is set. */
    if ((byte0 & CO_MPDO_BYTE0_SAM_MASK) == 0U) {
        return false;
    }

    uint8_t  srcNodeId = byte0 & CO_MPDO_BYTE0_NODEID_MASK;
    uint16_t srcIdx = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);
    uint8_t  srcSub = frame[3];

    /* First-match-wins linear scan; dispatch tables stay small enough that
     * a hash buys nothing — see docs/mpdo-implementation-plan.md §3.3. */
    CO_MPDO_dispatch_t *match = NULL;
    for (uint16_t i = 0; i < CO_CONFIG_MPDO_DISPATCH_COUNT; i++) {
        CO_MPDO_dispatch_t *d = &MPDO->dispatch[i];
        if (!d->valid) {
            continue;
        }
        if (d->srcNodeId == srcNodeId
            && d->srcIdx == srcIdx
            && d->srcSub == srcSub
        ) {
            match = d;
            break;
        }
    }

    /* Dispatcher miss: drop silently. The producer is broadcasting whatever
     * it scans; only rows we have explicitly subscribed to are routed. */
    if (match == NULL) {
        return true;
    }

    OD_entry_t *entry = OD_find(MPDO->OD, match->dstIdx);
    if (entry == NULL) {
        CO_errorReport(MPDO->em, CO_EM_RPDO_WRONG_LENGTH,
                       CO_EMC_DAM_MPDO,
                       ((uint32_t)match->dstIdx << 16) | ((uint32_t)match->dstSub << 8));
        return false;
    }

    OD_IO_t io;
    ODR_t odRet = OD_getSub(entry, match->dstSub, &io, false);
    if (odRet != ODR_OK) {
        CO_errorReport(MPDO->em, CO_EM_RPDO_WRONG_LENGTH,
                       CO_EMC_DAM_MPDO,
                       ((uint32_t)match->dstIdx << 16) | ((uint32_t)match->dstSub << 8));
        return false;
    }

    if ((io.stream.attribute & ODA_RPDO) == 0U) {
        CO_errorReport(MPDO->em, CO_EM_RPDO_WRONG_LENGTH,
                       CO_EMC_DAM_MPDO,
                       ((uint32_t)match->dstIdx << 16) | ((uint32_t)match->dstSub << 8));
        return false;
    }

    OD_size_t writeLen = io.stream.dataLength;
    if (writeLen == 0U || writeLen > 4U) {
        CO_errorReport(MPDO->em, CO_EM_RPDO_WRONG_LENGTH,
                       CO_EMC_DAM_MPDO,
                       ((uint32_t)match->dstIdx << 16) | ((uint32_t)match->dstSub << 8));
        return false;
    }

    io.stream.dataOffset = 0;
    OD_size_t countWritten = 0;

    CO_LOCK_OD(MPDO->CANdev);
    ODR_t writeRet = io.write(&io.stream, &frame[4], writeLen, &countWritten);
    CO_UNLOCK_OD(MPDO->CANdev);

    if (writeRet != ODR_OK || countWritten != writeLen) {
        CO_errorReport(MPDO->em, CO_EM_RPDO_WRONG_LENGTH,
                       CO_EMC_DAM_MPDO,
                       ((uint32_t)match->dstIdx << 16) | ((uint32_t)match->dstSub << 8));
        return false;
    }

    return true;
}
#endif /* (CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_SAM */


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_SAM)
/*
 * OD extension write callback installed on each scanned local OD entry.
 *
 * Lets the underlying storage absorb the write via OD_writeOriginal, then
 * flags the row dirty so CO_MPDO_processTX emits a SAM frame next tick.
 */
static ODR_t CO_MPDO_scanWrite(OD_stream_t *stream,
                               const void *buf,
                               OD_size_t count,
                               OD_size_t *countWritten)
{
    ODR_t ret = OD_writeOriginal(stream, buf, count, countWritten);
    CO_MPDO_scan_t *scan = (CO_MPDO_scan_t *)(stream != NULL ? stream->object : NULL);
    if (ret == ODR_OK && scan != NULL) {
        scan->dirty = true;
    }
    return ret;
}
#endif /* (CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_SAM */


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


#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_RX_DAM | CO_CONFIG_MPDO_RX_SAM))
/* RX-slot subscription is mode-agnostic — the per-frame mode dispatcher in
 * processRX picks DAM vs SAM by byte 0. Public configRX_DAM / configRX_SAM
 * wrappers exist for self-documenting call sites. */
static CO_ReturnError_t CO_MPDO_configRX(CO_MPDO_t *MPDO,
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


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_DAM)
/******************************************************************************/
CO_ReturnError_t CO_MPDO_configRX_DAM(CO_MPDO_t *MPDO,
                                      uint8_t slotIdx,
                                      uint16_t canId)
{
    return CO_MPDO_configRX(MPDO, slotIdx, canId);
}
#endif


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_SAM)
/******************************************************************************/
CO_ReturnError_t CO_MPDO_configRX_SAM(CO_MPDO_t *MPDO,
                                      uint8_t slotIdx,
                                      uint16_t canId)
{
    return CO_MPDO_configRX(MPDO, slotIdx, canId);
}


/******************************************************************************/
CO_ReturnError_t CO_MPDO_dispatchAdd_SAM(CO_MPDO_t *MPDO,
                                         uint8_t producerNodeId,
                                         uint16_t srcIdx,
                                         uint8_t srcSub,
                                         uint16_t dstIdx,
                                         uint8_t dstSub)
{
    if (MPDO == NULL || producerNodeId < 1U || producerNodeId > 127U) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Validate the local destination up front so a mis-provisioned row fails
     * loudly at registration instead of silently dropping frames at runtime.
     * Resolve with odOrig=false to match the view CO_MPDO_applySAM writes
     * through. */
    OD_entry_t *entry = OD_find(MPDO->OD, dstIdx);
    if (entry == NULL) {
        return CO_ERROR_OD_PARAMETERS;
    }
    OD_IO_t io;
    if (OD_getSub(entry, dstSub, &io, false) != ODR_OK) {
        return CO_ERROR_OD_PARAMETERS;
    }
    if ((io.stream.attribute & ODA_RPDO) == 0U
        || io.stream.dataLength == 0U || io.stream.dataLength > 4U
    ) {
        return CO_ERROR_OD_PARAMETERS;
    }

    /* Reject duplicate keys — dispatch must resolve to exactly one row. */
    for (uint16_t i = 0; i < CO_CONFIG_MPDO_DISPATCH_COUNT; i++) {
        CO_MPDO_dispatch_t *d = &MPDO->dispatch[i];
        if (d->valid && d->srcNodeId == producerNodeId
            && d->srcIdx == srcIdx && d->srcSub == srcSub
        ) {
            return CO_ERROR_ILLEGAL_ARGUMENT;
        }
    }

    for (uint16_t i = 0; i < CO_CONFIG_MPDO_DISPATCH_COUNT; i++) {
        CO_MPDO_dispatch_t *d = &MPDO->dispatch[i];
        if (d->valid) {
            continue;
        }
        d->srcNodeId = producerNodeId;
        d->srcIdx = srcIdx;
        d->srcSub = srcSub;
        d->dstIdx = dstIdx;
        d->dstSub = dstSub;
        d->dstLength = (uint8_t)io.stream.dataLength;
        d->valid = true;
        return CO_ERROR_NO;
    }
    return CO_ERROR_OUT_OF_MEMORY;
}
#endif


#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_TX_DAM | CO_CONFIG_MPDO_TX_SAM))
/* TX-slot subscription is mode-agnostic — the public configTX_DAM /
 * configTX_SAM wrappers exist for self-documenting call sites. */
static CO_ReturnError_t CO_MPDO_configTX(CO_MPDO_t *MPDO,
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
#endif


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_DAM)
/******************************************************************************/
CO_ReturnError_t CO_MPDO_configTX_DAM(CO_MPDO_t *MPDO,
                                      uint8_t slotIdx,
                                      uint16_t canId,
                                      uint32_t inhibitTime_us)
{
    return CO_MPDO_configTX(MPDO, slotIdx, canId, inhibitTime_us);
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


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_SAM)
/******************************************************************************/
CO_ReturnError_t CO_MPDO_configTX_SAM(CO_MPDO_t *MPDO,
                                      uint8_t slotIdx,
                                      uint16_t canId,
                                      uint32_t inhibitTime_us)
{
    return CO_MPDO_configTX(MPDO, slotIdx, canId, inhibitTime_us);
}


/******************************************************************************/
CO_ReturnError_t CO_MPDO_scanAdd_SAM(CO_MPDO_t *MPDO,
                                     uint8_t txSlotIdx,
                                     uint16_t srcIdx,
                                     uint8_t srcSub,
                                     uint8_t length)
{
    if (MPDO == NULL || txSlotIdx >= CO_CONFIG_MPDO_TX_COUNT
        || length == 0U || length > 4U
    ) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    if (!MPDO->tx[txSlotIdx].valid) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    OD_entry_t *entry = OD_find(MPDO->OD, srcIdx);
    if (entry == NULL) {
        return CO_ERROR_OD_PARAMETERS;
    }

    /* Probe the storage length via odOrig so we don't trip extensions. */
    OD_IO_t io;
    ODR_t odRet = OD_getSub(entry, srcSub, &io, true);
    if (odRet != ODR_OK) {
        return CO_ERROR_OD_PARAMETERS;
    }
    if (io.stream.dataLength != length) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    CO_MPDO_scan_t *scan = NULL;
    for (uint16_t i = 0; i < CO_CONFIG_MPDO_SCAN_COUNT; i++) {
        if (!MPDO->scan[i].valid) {
            scan = &MPDO->scan[i];
            break;
        }
    }
    if (scan == NULL) {
        return CO_ERROR_OUT_OF_MEMORY;
    }

    scan->srcIdx = srcIdx;
    scan->srcSub = srcSub;
    scan->length = length;
    scan->txSlotIdx = txSlotIdx;
    scan->entry = entry;
    scan->dirty = false;
    scan->scanExt.object = scan;
    scan->scanExt.read = OD_readOriginal;
    scan->scanExt.write = CO_MPDO_scanWrite;

    if (OD_extension_init(entry, &scan->scanExt) != ODR_OK) {
        return CO_ERROR_OD_PARAMETERS;
    }
    scan->valid = true;
    return CO_ERROR_NO;
}
#endif /* (CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_SAM */


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
#endif
#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_SAM)
        (void) CO_MPDO_applySAM(MPDO, frame);
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
    if (MPDO == NULL) {
        return;
    }

    /* Drain inhibit timers once per tick regardless of mode — both DAM and
     * SAM share the same per-slot CANtxBuff and inhibit window. */
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
    }

#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_DAM)
    for (uint8_t i = 0; i < CO_CONFIG_MPDO_TX_COUNT; i++) {
        CO_MPDO_tx_t *tx = &MPDO->tx[i];
        if (!tx->valid) {
            continue;
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

#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_SAM)
    for (uint16_t i = 0; i < CO_CONFIG_MPDO_SCAN_COUNT; i++) {
        CO_MPDO_scan_t *scan = &MPDO->scan[i];
        if (!scan->valid || !scan->dirty) {
            continue;
        }
        CO_MPDO_tx_t *tx = &MPDO->tx[scan->txSlotIdx];
        if (!tx->valid) {
            scan->dirty = false;
            continue;
        }
        if (tx->inhibitTimer != 0U) {
            if (timerNext_us != NULL && *timerNext_us > tx->inhibitTimer) {
                *timerNext_us = tx->inhibitTimer;
            }
            continue;
        }

        /* Clear dirty before reading so concurrent writes during the read
         * re-arm us for next tick rather than being lost. */
        scan->dirty = false;

        OD_IO_t io;
        ODR_t odRet = OD_getSub(scan->entry, scan->srcSub, &io, true);
        if (odRet != ODR_OK) {
            continue;
        }

        uint8_t payload[4];
        memset(payload, 0, sizeof(payload));
        OD_size_t countRead = 0;
        io.stream.dataOffset = 0;

        CO_LOCK_OD(MPDO->CANdev);
        ODR_t readRet = io.read(&io.stream, payload, scan->length, &countRead);
        CO_UNLOCK_OD(MPDO->CANdev);

        if (readRet != ODR_OK || countRead != scan->length) {
            continue;
        }

        uint8_t *frame = tx->CANtxBuff->data;
        frame[0] = (uint8_t)(CO_MPDO_BYTE0_SAM_MASK
                             | (MPDO->ownNodeId & CO_MPDO_BYTE0_NODEID_MASK));
        frame[1] = (uint8_t)(scan->srcIdx & 0xFFU);
        frame[2] = (uint8_t)(scan->srcIdx >> 8);
        frame[3] = scan->srcSub;
        memset(&frame[4], 0, 4);
        memcpy(&frame[4], payload, scan->length);

        if (CO_CANsend(MPDO->CANdev, tx->CANtxBuff) == CO_ERROR_NO) {
            tx->inhibitTimer = tx->inhibitTime_us;
        } else {
            /* TX queue full — re-arm for retry next tick. */
            scan->dirty = true;
        }
    }
#endif
}
#endif

#endif /* (CO_CONFIG_MPDO) != 0 */
