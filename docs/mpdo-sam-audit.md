# MPDO SAM-mode audit (commit `9ca3840 SAM_MPDO`, baseline `cb7d5b1 DAM_MPDO`)

Read-only audit of `301/CO_MPDO.c` / `301/CO_MPDO.h`. Citations are
`301/CO_MPDO.c:<line>` unless noted. No source files were modified.

---

## 1. CO_MPDO_applySAM — consumer apply path

### 1a. Dispatch key includes producerNodeId — OK
`301/CO_MPDO.c:154-157`: the match requires `d->srcNodeId == srcNodeId &&
d->srcIdx == srcIdx && d->srcSub == srcSub`, where `srcNodeId` is the byte-0
low-7-bit producer node ID (`:142`). Key is the full `(producerNodeId, srcIdx,
srcSub)` tuple as required by ground truth.

### 1b. Byte-0 MSB demux — OK
- SAM apply rejects frames with bit 7 clear: `:138` `if ((byte0 &
  CO_MPDO_BYTE0_SAM_MASK) == 0U) return false;` (MSB=1 ⇒ SAM). Correct.
- DAM apply rejects frames with bit 7 set: `:56`. Correct (MSB=0 ⇒ DAM).
- `CO_MPDO_BYTE0_SAM_MASK = 0x80`, `CO_MPDO_BYTE0_NODEID_MASK = 0x7F`
  (`301/CO_MPDO.h:86-88`). Index little-endian `frame[1] | frame[2]<<8`
  (`:143`), subindex `frame[3]` (`:144`), payload `&frame[4]` (`:205`). All
  match the wire format.

**MEDIUM — both apply functions run unconditionally on every frame.**
`CO_MPDO_processRX` calls `applyDAM` then `applySAM` back-to-back
(`:532-537`) with no mutual exclusion. This is correct *only* because each
function self-rejects the other mode by byte-0 (`:56`, `:138`). It works, but
the design leans entirely on each apply re-deriving the mode; there is no
single demux point. Low functional risk; noted because it makes the
"silently dropped but counted" requirement harder to satisfy (see §3).

### 1c. Payload size vs mapped local entry size — HIGH
`:193-199`: `writeLen = io.stream.dataLength` (the OD entry's own length), and
the code writes exactly that many bytes from `&frame[4]`, rejecting only
`writeLen == 0 || writeLen > 4`. There is **no DLC / wire-length field to
compare against** — MPDO frames are fixed 8 bytes (`CO_MPDO_receive` drops
anything with `DLC != 8`, `:37`), so the wire never carries the real payload
length. Consequences:
- If the OD entry is, say, 2 bytes but the producer only meant to send 1
  meaningful byte, the consumer writes 2 bytes (1 real + 1 zero-pad from the
  fixed frame) with no way to detect the mismatch. The protocol has no length
  field, so this is inherent to MPDO, **but** the implementation silently
  trusts the local OD length as authoritative without any cross-check that the
  dispatch row's intended length equals the destination entry length.
- The dispatch row (`CO_MPDO_dispatch_t`) stores **no length** at all
  (`301/CO_MPDO.h:127-134`), unlike the scan row which stores `length`. So a
  misconfigured `(dstIdx,dstSub)` pointing at a 4-byte entry will happily
  absorb 4 bytes even if the producer's source was 1 byte.

**Fix:** add an expected-length field to `CO_MPDO_dispatch_t`, validate it at
`dispatchAdd` time against the destination OD entry length (mirroring
`scanAdd`'s `io.stream.dataLength != length` check at `:481`), and reject /
count frames where the resolved write length disagrees.

---

## 2. Config-time validation in CO_MPDO_dispatchAdd_SAM — multiple gaps

`CO_MPDO_dispatchAdd_SAM` (`:332-357`) validates **only** `MPDO != NULL` and
`producerNodeId` in 1..127 (`:339`). It then copies the row in and returns. It
performs **zero** validation of the destination. Per the four required checks:

- **(a) nonexistent local target — MISSING (HIGH).** No `OD_find(dstIdx)`.
  An invalid `dstIdx` is accepted at config time and only discovered per-frame
  at runtime in `applySAM` (`:169-175`), which raises EMCY on *every* matching
  frame forever. The header doc at `301/CO_MPDO.h:288-306` even claims "The
  local OD entry must have ODA_RPDO set and a length in 1..4" — but the code
  never enforces any of it here.
- **(b) non-writable target — MISSING (HIGH).** No `ODA_RPDO` check. Deferred
  to runtime (`:186`), again EMCY-per-frame.
- **(c) >4-byte target — MISSING (MEDIUM).** No length probe. Deferred to
  runtime (`:194`).
- **(d) duplicate / overlapping `(nodeId, idx, sub)` keys — MISSING (MEDIUM).**
  The add loop (`:343-355`) just grabs the first free slot; it never scans for
  an existing row with the same `(srcNodeId, srcIdx, srcSub)`. Because
  `applySAM` is first-match-wins (`:159` `break`), a duplicate key silently
  shadows the later row — a latent misconfiguration that is never reported.

Contrast: `CO_MPDO_scanAdd_SAM` (`:455-511`) *does* do the producer-side
equivalents — `OD_find` (`:470`), `OD_getSub` (`:477`), and
`io.stream.dataLength != length` (`:481`). The consumer path is the weaker of
the two and contradicts its own header contract.

**Fix:** in `dispatchAdd_SAM`, after the nodeId check: `OD_find(dstIdx)` →
reject `CO_ERROR_OD_PARAMETERS` if NULL; `OD_getSub(dstSub)` → reject if not
OK; check `(attribute & ODA_RPDO)` → reject if unset; check
`dataLength` in 1..4 → reject otherwise; and scan existing valid rows for a
matching `(srcNodeId,srcIdx,srcSub)` → reject `CO_ERROR_ILLEGAL_ARGUMENT` on
duplicate.

---

## 3. Runtime protections / drop paths

Enumerated drop/reject paths and whether each is **counted** (EMCY) or
**invisible**:

| # | Path | Location | Counted? |
|---|------|----------|----------|
| 1 | RX frame DLC != 8 (runt) | `:37` | **INVISIBLE** — `return` in RX callback, no counter |
| 2 | RX slot not valid | `:37` | INVISIBLE (expected) |
| 3 | SAM frame but bit7 clear (mode mismatch) | `:138` | INVISIBLE (by design, DAM handles it) |
| 4 | DAM frame but bit7 set | `:56` | INVISIBLE (by design) |
| 5 | DAM not addressed to us / broadcast | `:63-67` | INVISIBLE (by design) |
| 6 | **SAM dispatcher miss (no matching row)** | `:165-167` | **INVISIBLE** |
| 7 | dst entry not found | `:171` | counted (EMCY `CO_EMC_DAM_MPDO`) |
| 8 | dst getSub fails | `:180` | counted |
| 9 | dst not ODA_RPDO | `:187` | counted |
| 10 | dst length 0 or >4 | `:195` | counted |
| 11 | OD write rejected / short write | `:209` | counted |

**HIGH — ground truth says "every drop class must be counted"; paths 1 and 6
are invisible.** The spec permits *silently dropping* unmatched SAM tuples on
the wire (no EMCY to the bus), but the audit ground truth additionally
requires an internal counter per drop class for observability. There is **no
counter anywhere** — not for dispatcher misses (#6, the single most important
class for diagnosing a mis-provisioned scanner/dispatcher pairing), nor for
runt frames (#1). A silently-dropped-and-uncounted miss is undebuggable in the
field.

**Fix:** add per-class `uint32_t` counters to `CO_MPDO_t` (at minimum
`dropNoDispatch`, `dropBadDlc`) and increment them at `:37` and `:166`. Do
**not** emit bus EMCY for the miss (spec-correct to stay silent on the wire),
just count internally.

**Deferred-processing context — OK, matches RPDO precedent.** The CAN-RX
callback `CO_MPDO_receive` (`:31-43`) does *only* `memcpy` into `rx->CANrxData`
+ `CO_FLAG_SET(rx->CANrxNew)` (`:41-42`); all OD work (`applyDAM`/`applySAM`,
including `OD.write` under `CO_LOCK_OD`) happens in `CO_MPDO_processRX`
(`:517-539`), which the caller invokes from task context once per tick. This
mirrors the RPDO precedent exactly: `CO_PDO_receive` only `memcpy` +
`CO_FLAG_SET(RPDO->CANrxNew[bufNo])` (`301/CO_PDO.c:505-506`), and
`CO_RPDO_process` drains it later under the `CO_FLAG_READ ... CO_FLAG_CLEAR`
loop (`301/CO_PDO.c:830-894`). OK.

One nuance: RPDO uses a `while (CO_FLAG_READ)` drain loop and clears the flag
*inside* the loop so a frame arriving during processing is re-picked-up
(`301/CO_PDO.c:830,836`). MPDO clears the flag once (`:530`) then processes the
snapshot — a frame arriving between the `memcpy`(`:529`) and the
`CO_FLAG_CLEAR`(`:530`) would set the flag again and be re-read next tick (the
single 8-byte buffer means only the latest is kept, which is acceptable for
event telemetry). LOW; not a defect.

---

## 4. CO_MPDO_scanWrite / processTX SAM walker — producer race

### Ordering trace (clear-before-read)
Producer dirty flag lifecycle:
- Writer side: `CO_MPDO_scanWrite` (`:227-238`) runs on the OD-write path —
  `OD_writeOriginal` stores the value (`:232`), then `scan->dirty = true`
  (`:235`) **after** the store.
- Emitter side in `processTX` (`:592-648`):
  1. `:595` skip if `!dirty`.
  2. `:612` **`scan->dirty = false;`** (clear)
  3. `:615` `OD_getSub`; `:626` `io.read(...)` (read snapshot) under
     `CO_LOCK_OD` (`:625-627`)
  4. `:642` `CO_CANsend`; on failure `:646` `scan->dirty = true` (re-arm).

**Window analysis — CLOSED for the read, with a caveat.** Because the flag is
cleared (`:612`) *before* the read (`:626`), a writer that completes its
`scan->dirty = true` (`:235`) at any point *after* `:612` will leave the flag
set when the current emit finishes, so the next `processTX` tick re-emits with
the newer value. The classic lost-update window (clear *after* read) is
avoided. **OK.**

Caveat ordering inside `scanWrite` (LOW→MEDIUM): the store (`:232`) happens
before the dirty-set (`:235`). Consider interleave: emitter reads the new
value at `:626` (writer already stored at `:232`) but the writer has not yet
reached `:235`; emitter sends the new value; writer then sets `dirty=true` at
`:235`. Result: a *redundant* re-send of an identical value next tick — wasted
bus bandwidth, never a lost update. Harmless for telemetry. There is no memory
barrier between `:232` and `:235`, and `dirty` is `volatile bool_t` only
(`301/CO_MPDO.h:152`); on a weakly-ordered core a reader could observe
`dirty=true` before the stored data is visible. On CP (Cortex-M7) and VC
(Cortex-M; both single-core, writer and emitter on the same task context per
the receive-context comment `:25-29`) this is moot. If `scanWrite` were ever
invoked from a different thread than `processTX`, the lack of a barrier would
be a real (if low-probability) hazard. Recommend documenting the
same-context assumption or adding a barrier; do not rely on `volatile` for
cross-core ordering.

### Inhibit-timer change vs DAM-only behavior — OK, DAM unchanged
Diff `cb7d5b1..9ca3840`: the inhibit **decrement** loop was lifted out of the
`#if TX_DAM` block into a mode-agnostic loop that runs for all valid TX slots
(`:553-565`). In the baseline the decrement lived *inside* the `#if
CO_CONFIG_MPDO_TX_DAM` guard. The DAM send/retry/`timerNext_us` logic itself
(`:567-589`) is byte-for-byte identical to the baseline (only relocated below
the shared decrement). For a DAM-only build (`02_dam` combo) the net behavior
is unchanged: each valid slot's `inhibitTimer` is decremented once per tick,
then the DAM block sends when `inhibitTimer == 0`. The only observable
difference is that in a TX_SAM-only build the timers now tick (the whole point
of the lift — see commit msg). **No DAM regression.** OK.

---

## 5. TX trigger model

**There is NO timer/rotation mechanism. TX is purely dirty-flag-driven.**
The SAM walker (`:593-648`) emits a frame **iff** `scan->dirty` is set
(`:595`). `dirty` is set in exactly one place: `CO_MPDO_scanWrite` (`:235`),
i.e. only when the application *writes* the scanned OD entry. There is no
event timer, no round-robin cursor, no periodic re-arm. The only timer present
is the per-TX-slot **inhibit** timer (`:553-565`, `:603`), which merely *rate-
limits* dirty-driven sends — it never *originates* one. An OD entry that is
written once and never again is emitted once and goes silent.

This contradicts the stated production need (CYCLIC telemetry: rotate through
all scanned entries on a timer). As written, cyclic behavior would have to be
manufactured by the **application** poking the OD entries (or the dirty flags)
on its own timer — there is no library scheduler.

**Where rotation would have to live today:** app-level. The app would need to
either re-write each scanned OD value every cycle (triggering `scanWrite` →
`dirty`), or reach into `MPDO->scan[i].dirty` directly (breaks encapsulation,
and races the inhibit logic).

**RECOMMENDATION (describe only, do NOT implement):** add a library-side
event-timer per scan row, modeled on the existing TPDO `eventTimer`
(`301/CO_PDO.c:1041,1467` — TPDO already implements exactly this cyclic
pattern). Concretely:
- add `uint32_t eventTime_us` + `uint32_t eventTimer` to `CO_MPDO_scan_t`,
  configured via a new arg to `scanAdd_SAM` (0 = event-driven only, current
  behavior).
- in `processTX`, decrement each row's `eventTimer` by `timeDifference_us`;
  when it hits 0, set `dirty = true` and reload `eventTimer = eventTime_us`.
- this reuses the dirty-flag emit path unchanged, keeps the inhibit window
  semantics, exports `timerNext_us` correctly, and matches the CANopenNode
  TPDO precedent so it will be familiar to reviewers.

Rationale for library-side over app-poking: cyclic emission is a transport
concern, not an application concern; keeping it in the library preserves the
inhibit-window invariant (an app poking `dirty` races the
clear-before-read), keeps `scan[]` encapsulated, and gives a single correct
`timerNext_us` to the OS scheduler. Per-row period (vs one global period)
matches real telemetry where different signals want different rates. Do NOT
implement here — recommendation only.

---

## 6. Compile matrix — independently reproduced

**Claimed in commit message:** combinations
`{0, RX_DAM|TX_DAM, RX_SAM|TX_SAM, all four, RX_SAM only, TX_SAM only}` with
`-Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes -Wmissing-prototypes
-std=c11`.

**Method:** compiled `301/CO_MPDO.c` with `gcc` (host) using the in-tree
`example/CO_driver_target.h` blank driver target and the repo's real
`301/CO_config.h` / `301/CO_ODinterface.h`, includes `-I<root> -I<root>/example`,
exact claimed flags, `-c`. Each of the 6 combinations passed via
`-DCO_CONFIG_MPDO=<combo>`. No stubbing of CANopenNode internals was needed —
the example driver target plus the real 301 headers are self-sufficient for a
compile-only check.

**Actual results:**

| Combo | `CO_CONFIG_MPDO` | rc | Result |
|-------|------------------|----|--------|
| 0 | `0` | 0 | **PASS, clean** |
| RX_DAM\|TX_DAM | `0x01\|0x02` | 0 | PASS, **1 warning** |
| RX_SAM\|TX_SAM | `0x04\|0x08` | 0 | PASS, **1 warning** |
| all four | `0x01\|0x02\|0x04\|0x08` | 0 | PASS, **1 warning** |
| RX_SAM only | `0x04` | 0 | PASS, **1 warning** |
| TX_SAM only | `0x08` | 0 | PASS, clean |

**The one warning (5 of 6 combos):**
```
301/CO_MPDO.c:31:49: warning: unused parameter 'msg' [-Wunused-parameter]
   static void CO_MPDO_receive(void *object, void *msg) {
```
`-Wunused-parameter` is part of `-Wextra`, which the commit message lists.

**Verdict on the commit's "compile-tested ... clean" implication: PARTIALLY
SUPPORTED.** All six combinations *compile with rc=0*, so the matrix claim is
true at face value. However the message implies a clean `-Wextra` build, and 5
of 6 combos emit a `-Wunused-parameter` warning. **This warning is a
test-harness artifact, not a library defect:** the example/blank driver defines
`CO_CANrxMsg_readDLC(msg)`/`CO_CANrxMsg_readData(msg)` as macros that discard
their argument (`example/CO_driver_target.h:64-65` → `((uint8_t)0)` /
`((uint8_t *)NULL)`), so `msg` is genuinely unused in the preprocessed TU. On
the **real** CP and VC driver targets these are real functions consuming `msg`
(`src/rt/canopennode/CO_driver_target.h:187-188` declares
`CO_CANrxMsg_readDLC(void* msg)` / `CO_CANrxMsg_readData(void* msg)`; the VC
target uses `static inline` accessors taking `const CO_CANrxMsg_t*`), so the
warning would **not** fire in a production build. The commit author almost
certainly compiled against a real driver target and saw a clean build —
reproducible and plausible. **LOW.** No fix required for the library; if a
warning-clean blank-target build is desired, the harness driver macros (not
`CO_MPDO.c`) are the thing to touch.

`-std=c11` is satisfied (mixed declarations, `for`-loop scope decls, `//`-free
C-style comments all compile clean under `-Wpedantic`).

---

## Fix list ranked by severity

| Sev | Area | File:line | Issue | Fix (describe) |
|-----|------|-----------|-------|----------------|
| HIGH | §2a/b | `301/CO_MPDO.c:332-357` | `dispatchAdd_SAM` never validates dst exists / is ODA_RPDO; contradicts its own header contract | Add `OD_find` + `OD_getSub` + `ODA_RPDO` checks at config time |
| HIGH | §3 #6 | `301/CO_MPDO.c:165-167` | SAM dispatcher miss dropped **and uncounted** — undebuggable | Add `dropNoDispatch` counter to `CO_MPDO_t`, increment on miss (stay silent on bus) |
| HIGH | §1c | `301/CO_MPDO.c:193-199`; `301/CO_MPDO.h:127-134` | No length cross-check; dispatch row carries no expected length, trusts dst OD length blindly | Add `length` to dispatch row, validate vs dst OD entry at config + reject mismatched frames |
| MEDIUM | §2c | `301/CO_MPDO.c:332-357` | dst >4-byte / 0-byte not rejected at config | Add length-1..4 probe in `dispatchAdd_SAM` |
| MEDIUM | §2d | `301/CO_MPDO.c:343-355` | Duplicate/overlapping `(nodeId,idx,sub)` keys silently accepted; later row shadowed | Scan existing valid rows, reject duplicate key |
| MEDIUM | §3 #1 | `301/CO_MPDO.c:37` | Runt (DLC!=8) dropped uncounted | Add `dropBadDlc` counter |
| MEDIUM | §1b | `301/CO_MPDO.c:532-537` | No single mode-demux point; relies on each apply re-deriving mode | Optional: demux byte-0 once in processRX, dispatch to one apply |
| LOW | §4 | `301/CO_MPDO.c:232-235`; `301/CO_MPDO.h:152` | `dirty` write ordered after data store with no barrier; only `volatile` | Document same-context assumption or add barrier; safe on current single-core targets |
| LOW | §6 | `example/CO_driver_target.h:64-65` | `-Wunused-parameter` on `msg` in blank-target build only | Harness artifact; not a library defect. No action on `CO_MPDO.c` |

**Not a bug (verified OK):** dispatch key includes producerNodeId (§1a);
byte-0 SAM/DAM demux correct (§1b); RX work deferred out of CAN-RX context
matching RPDO precedent (§3); clear-before-read closes the lost-update window
(§4); inhibit-timer lift did not regress DAM-only behavior (§4); all 6
compile-matrix combinations build rc=0 (§6).

**Design gap (not a code bug), §5:** TX is purely dirty-flag-driven with no
cyclic timer. Production cyclic-telemetry use requires either app-side poking
(racy, breaks encapsulation) or a new library-side per-row event timer
modeled on TPDO `eventTimer`. Recommend the latter. Not implemented here.
