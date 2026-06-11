/**
 * CANopen Multiplexed Process Data Object (MPDO) protocol — CiA 301 §7.2.5.
 *
 * @file        CO_MPDO.h
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

#ifndef CO_MPDO_H
#define CO_MPDO_H

#include "301/CO_ODinterface.h"
#include "301/CO_Emergency.h"
#include "301/CO_driver.h"

/* Default to compiled out unless the target overrides it. */
#ifndef CO_CONFIG_MPDO
#define CO_CONFIG_MPDO (0)
#endif

#if (CO_CONFIG_MPDO) != 0 || defined CO_DOXYGEN

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup CO_MPDO MPDO
 * CANopen Multiplexed Process Data Object — CiA 301 §7.2.5.
 *
 * @ingroup CO_CANopen_301
 * @{
 *
 * MPDO solves "I want to push many distinct OD entries through one PDO slot"
 * without consuming one of the 8 mapping slots per target entry. The producer
 * builds a frame that names the destination (DAM) or names the source (SAM)
 * inline, so a single COB-ID can move payload into arbitrary destinations.
 *
 * Frame layout, 8 data bytes:
 * - byte 0   addressing byte; bit 7 = 0 ⇒ DAM, bit 7 = 1 ⇒ SAM.
 *            For DAM, bits 6..0 are the target node ID (0 ⇒ broadcast).
 *            For SAM, bits 6..0 are the producer node ID.
 * - bytes 1..2  multiplexor index (little-endian).
 * - byte 3       multiplexor sub-index.
 * - bytes 4..7   1..4 bytes of payload, left-aligned (high bytes unused).
 *
 * Transmission type for MPDO is fixed at event-driven (≥ 0xFE).
 */

/** Number of MPDO consumer RX slots. */
#ifndef CO_CONFIG_MPDO_RX_COUNT
#define CO_CONFIG_MPDO_RX_COUNT 1
#endif

/** Number of MPDO producer TX slots. */
#ifndef CO_CONFIG_MPDO_TX_COUNT
#define CO_CONFIG_MPDO_TX_COUNT 1
#endif

/** Maximum number of SAM dispatch entries (consumer side, 0x1FD0 equivalent). */
#ifndef CO_CONFIG_MPDO_DISPATCH_COUNT
#define CO_CONFIG_MPDO_DISPATCH_COUNT 8
#endif

/** Maximum number of SAM scan entries (producer side, 0x1FA0 equivalent). */
#ifndef CO_CONFIG_MPDO_SCAN_COUNT
#define CO_CONFIG_MPDO_SCAN_COUNT 8
#endif

/** MPDO frame addressing mode. */
typedef enum {
    CO_MPDO_MODE_DAM = 0,
    CO_MPDO_MODE_SAM = 1
} CO_MPDO_mode_t;

/** Bit 7 of byte 0 distinguishes SAM from DAM on the wire. */
#define CO_MPDO_BYTE0_SAM_MASK 0x80U
/** Bits 6..0 of byte 0 are the (target/source) node ID. */
#define CO_MPDO_BYTE0_NODEID_MASK 0x7FU
/** Broadcast destination encoding for DAM. */
#define CO_MPDO_DAM_BROADCAST 0U

#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_RX_DAM | CO_CONFIG_MPDO_RX_SAM)) || defined CO_DOXYGEN
/**
 * Per-slot MPDO consumer state. One slot is bound to one CAN-ID — typically
 * the producer's COB-ID for its MPDO-TPDO. The receive callback only buffers
 * the frame; CO_process_MPDO_RX does the OD work in task context.
 */
typedef struct {
    bool_t   valid;            /**< Slot is configured. */
    uint16_t canId;            /**< 11-bit COB-ID, 0 ⇒ slot disabled. */
    volatile void *CANrxNew;   /**< Same flag-style as CO_RPDO_t. */
    uint8_t  CANrxData[8];     /**< Buffered payload. */
} CO_MPDO_rx_t;
#endif

#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_TX_DAM | CO_CONFIG_MPDO_TX_SAM)) || defined CO_DOXYGEN
/**
 * Per-slot MPDO producer state.
 */
typedef struct {
    bool_t   valid;            /**< Slot is configured. */
    CO_CANtx_t *CANtxBuff;     /**< Tx buffer, returned by CO_CANtxBufferInit. */
    uint32_t inhibitTime_us;   /**< Inhibit interval. */
    uint32_t inhibitTimer;     /**< Counts down to 0 between sends. */
    volatile bool_t sendRequest; /**< Caller scheduled a frame (DAM). */
    uint8_t  pendingFrame[8];  /**< Built by CO_MPDO_send_DAM(). */
} CO_MPDO_tx_t;
#endif

#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_SAM) || defined CO_DOXYGEN
/**
 * One row of the SAM dispatcher table. CiA 301 §7.2.5.3 OD 0x1FD0 equivalent.
 * On receive of a SAM frame, the table is searched for a row whose
 * (srcNodeId, srcIdx, srcSub) matches the frame, and the payload is then
 * written to (dstIdx, dstSub) on this node's OD.
 */
typedef struct {
    bool_t   valid;            /**< Slot is configured. */
    uint8_t  srcNodeId;        /**< Producer node ID (1..127). */
    uint16_t srcIdx;           /**< Producer-side OD index. */
    uint8_t  srcSub;           /**< Producer-side OD sub-index. */
    uint16_t dstIdx;           /**< Local OD index to write into. */
    uint8_t  dstSub;           /**< Local OD sub-index to write into. */
    uint8_t  dstLength;        /**< Target storage length captured at registration (1..4). */
} CO_MPDO_dispatch_t;
#endif

#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_SAM) || defined CO_DOXYGEN
/**
 * One row of the SAM scanner table. CiA 301 §7.2.5.3 OD 0x1FA0 equivalent.
 * Bound to a local OD entry via OD_extension_init(); writes to that entry
 * raise the dirty flag, and CO_MPDO_processTX emits one SAM frame per dirty
 * row on the carrier TX slot.
 */
typedef struct {
    bool_t   valid;            /**< Slot is configured. */
    uint16_t srcIdx;           /**< Local OD index being scanned. */
    uint8_t  srcSub;           /**< Local OD sub-index being scanned. */
    uint8_t  length;           /**< 1..4 bytes of payload to emit. */
    uint8_t  txSlotIdx;        /**< Carrier TX slot in CO_MPDO_t::tx[]. */
    OD_entry_t *entry;         /**< Cached OD_find() result, for read on emit. */
    OD_extension_t scanExt;    /**< Installed on the scanned OD entry. */
    volatile bool_t dirty;     /**< Set on local write, cleared on emit. */
} CO_MPDO_scan_t;
#endif

/**
 * Top-level MPDO state, one instance per @ref CO_t.
 */
typedef struct {
    CO_EM_t        *em;
    CO_CANmodule_t *CANdev;
    OD_t           *OD;
    uint8_t         ownNodeId;
#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_RX_DAM | CO_CONFIG_MPDO_RX_SAM)) || defined CO_DOXYGEN
    CO_MPDO_rx_t    rx[CO_CONFIG_MPDO_RX_COUNT];
    uint16_t        rxIdxBase; /**< First rxArray index owned by MPDO. */
#endif
#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_TX_DAM | CO_CONFIG_MPDO_TX_SAM)) || defined CO_DOXYGEN
    CO_MPDO_tx_t    tx[CO_CONFIG_MPDO_TX_COUNT];
    uint16_t        txIdxBase; /**< First txArray index owned by MPDO. */
#endif
#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_SAM) || defined CO_DOXYGEN
    CO_MPDO_dispatch_t dispatch[CO_CONFIG_MPDO_DISPATCH_COUNT];
#endif
#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_SAM) || defined CO_DOXYGEN
    CO_MPDO_scan_t  scan[CO_CONFIG_MPDO_SCAN_COUNT];
#endif
} CO_MPDO_t;


/**
 * Initialize the MPDO container.
 *
 * Resets all slot state; call once after CO_CANopenInitPDO(). Per-slot
 * configuration is applied separately via CO_MPDO_configRX_DAM() and
 * CO_MPDO_configTX_DAM().
 *
 * @param MPDO Container object.
 * @param OD Object Dictionary (used by RX-DAM to resolve writes).
 * @param em Emergency object.
 * @param CANdev CAN module.
 * @param ownNodeId Node ID of this node (1..127); used for DAM target match.
 * @param rxIdxBase First rxArray index reserved for MPDO RX slots.
 * @param txIdxBase First txArray index reserved for MPDO TX slots.
 *
 * @return CO_ERROR_NO on success.
 */
CO_ReturnError_t CO_MPDO_init(CO_MPDO_t *MPDO,
                              OD_t *OD,
                              CO_EM_t *em,
                              CO_CANmodule_t *CANdev,
                              uint8_t ownNodeId,
                              uint16_t rxIdxBase,
                              uint16_t txIdxBase);


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_DAM) || defined CO_DOXYGEN
/**
 * Configure one MPDO consumer slot for DAM reception.
 *
 * @param MPDO Container.
 * @param slotIdx 0..CO_CONFIG_MPDO_RX_COUNT-1.
 * @param canId 11-bit COB-ID to subscribe to.
 *
 * @return CO_ERROR_NO on success.
 */
CO_ReturnError_t CO_MPDO_configRX_DAM(CO_MPDO_t *MPDO,
                                      uint8_t slotIdx,
                                      uint16_t canId);
#endif


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_DAM) || defined CO_DOXYGEN
/**
 * Configure one MPDO producer slot.
 *
 * @param MPDO Container.
 * @param slotIdx 0..CO_CONFIG_MPDO_TX_COUNT-1.
 * @param canId 11-bit COB-ID to publish on.
 * @param inhibitTime_us Minimum spacing between sends on this slot.
 *
 * @return CO_ERROR_NO on success.
 */
CO_ReturnError_t CO_MPDO_configTX_DAM(CO_MPDO_t *MPDO,
                                      uint8_t slotIdx,
                                      uint16_t canId,
                                      uint32_t inhibitTime_us);


/**
 * Fire-and-forget DAM send. Schedules one MPDO frame; the actual CO_CANsend
 * happens inside @ref CO_process_MPDO_TX once the inhibit window has elapsed.
 *
 * @param MPDO Container.
 * @param slotIdx Producer slot to send on.
 * @param targetNodeId 1..127, or 0 for broadcast.
 * @param idx Destination OD index on the consumer.
 * @param sub Destination OD sub-index on the consumer.
 * @param data Payload (1..4 bytes).
 * @param len 1..4.
 *
 * @return CO_ERROR_NO on success, CO_ERROR_ILLEGAL_ARGUMENT on bad args,
 *         CO_ERROR_TX_OVERFLOW if a previous send is still pending.
 */
CO_ReturnError_t CO_MPDO_send_DAM(CO_MPDO_t *MPDO,
                                  uint8_t slotIdx,
                                  uint8_t targetNodeId,
                                  uint16_t idx,
                                  uint8_t sub,
                                  const void *data,
                                  uint8_t len);
#endif


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_RX_SAM) || defined CO_DOXYGEN
/**
 * Configure one MPDO consumer slot for SAM reception. Behaves identically to
 * CO_MPDO_configRX_DAM — same CAN-ID subscription — but is provided as a
 * separate symbol so callers self-document the slot's intent. When both RX
 * modes are compiled in, a single subscribed CAN-ID delivers DAM and SAM
 * frames to the same slot; the mode is picked apart by byte 0 of the payload.
 *
 * @param MPDO Container.
 * @param slotIdx 0..CO_CONFIG_MPDO_RX_COUNT-1.
 * @param canId 11-bit COB-ID to subscribe to.
 *
 * @return CO_ERROR_NO on success.
 */
CO_ReturnError_t CO_MPDO_configRX_SAM(CO_MPDO_t *MPDO,
                                      uint8_t slotIdx,
                                      uint16_t canId);


/**
 * Append one entry to the SAM dispatcher table.
 *
 * On receipt of a SAM frame whose byte 0 names @p producerNodeId and whose
 * (srcIdx, srcSub) match this row, the payload is written into this node's
 * (dstIdx, dstSub). The destination is validated here at registration: it
 * must exist in the OD, have ODA_RPDO set, and have a storage length in 1..4.
 * That length is captured and re-checked on every apply. Duplicate
 * (producerNodeId, srcIdx, srcSub) keys are rejected so dispatch is
 * unambiguous.
 *
 * @param MPDO Container.
 * @param producerNodeId 1..127 — producer node we accept frames from.
 * @param srcIdx Producer OD index named in the SAM frame.
 * @param srcSub Producer OD sub-index named in the SAM frame.
 * @param dstIdx Local OD index to write into.
 * @param dstSub Local OD sub-index to write into.
 *
 * @return CO_ERROR_NO on success,
 *         CO_ERROR_OUT_OF_MEMORY if dispatch table is full,
 *         CO_ERROR_OD_PARAMETERS if the destination is missing, not
 *         PDO-writable, or not 1..4 bytes,
 *         CO_ERROR_ILLEGAL_ARGUMENT on bad args or a duplicate key.
 */
CO_ReturnError_t CO_MPDO_dispatchAdd_SAM(CO_MPDO_t *MPDO,
                                         uint8_t producerNodeId,
                                         uint16_t srcIdx,
                                         uint8_t srcSub,
                                         uint16_t dstIdx,
                                         uint8_t dstSub);
#endif


#if ((CO_CONFIG_MPDO) & CO_CONFIG_MPDO_TX_SAM) || defined CO_DOXYGEN
/**
 * Configure one MPDO producer slot to carry SAM frames.
 *
 * @param MPDO Container.
 * @param slotIdx 0..CO_CONFIG_MPDO_TX_COUNT-1.
 * @param canId 11-bit COB-ID to publish on.
 * @param inhibitTime_us Minimum spacing between sends on this slot.
 *
 * @return CO_ERROR_NO on success.
 */
CO_ReturnError_t CO_MPDO_configTX_SAM(CO_MPDO_t *MPDO,
                                      uint8_t slotIdx,
                                      uint16_t canId,
                                      uint32_t inhibitTime_us);


/**
 * Register a local OD entry for SAM scanning.
 *
 * Installs an OD_extension_t on (srcIdx, srcSub) that flags the row dirty on
 * write while preserving the original storage via OD_writeOriginal. The
 * @ref CO_MPDO_processTX walker then emits one SAM frame per dirty row on
 * the carrier @p txSlotIdx, subject to that slot's inhibit window.
 *
 * Warning: this overwrites any extension previously installed on the entry.
 * Applications that need to coexist with custom OD extensions must wrap the
 * scan plumbing manually for now.
 *
 * @param MPDO Container.
 * @param txSlotIdx Carrier TX slot, must have been configured via
 *                  CO_MPDO_configTX_SAM().
 * @param srcIdx Local OD index to scan.
 * @param srcSub Local OD sub-index to scan.
 * @param length 1..4 — payload size to emit per frame; must match the OD
 *               entry's storage length.
 *
 * @return CO_ERROR_NO on success,
 *         CO_ERROR_OUT_OF_MEMORY if scan table is full,
 *         CO_ERROR_OD_PARAMETERS if (srcIdx, srcSub) is not in the OD,
 *         CO_ERROR_ILLEGAL_ARGUMENT on bad length / TX slot.
 */
CO_ReturnError_t CO_MPDO_scanAdd_SAM(CO_MPDO_t *MPDO,
                                     uint8_t txSlotIdx,
                                     uint16_t srcIdx,
                                     uint8_t srcSub,
                                     uint8_t length);
#endif


#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_RX_DAM | CO_CONFIG_MPDO_RX_SAM)) || defined CO_DOXYGEN
/**
 * Drain pending MPDO frames into the OD. Call once per RT tick after
 * CO_process_RPDO().
 *
 * @param MPDO Container.
 */
void CO_MPDO_processRX(CO_MPDO_t *MPDO);
#endif


#if ((CO_CONFIG_MPDO) & (CO_CONFIG_MPDO_TX_DAM | CO_CONFIG_MPDO_TX_SAM)) || defined CO_DOXYGEN
/**
 * Send any scheduled MPDO frames whose inhibit window has elapsed. Call once
 * per RT tick after CO_process_TPDO().
 *
 * @param MPDO Container.
 * @param timeDifference_us Time since last call.
 * @param [out] timerNext_us info to OS — see CO_process(); may be NULL.
 */
void CO_MPDO_processTX(CO_MPDO_t *MPDO,
                       uint32_t timeDifference_us,
                       uint32_t *timerNext_us);
#endif

/** @} */ /* CO_MPDO */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* (CO_CONFIG_MPDO) != 0 */

#endif /* CO_MPDO_H */
