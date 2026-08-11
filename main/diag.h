/**
 * @file diag.h
 * @brief Long-run diagnostics: counters plus a log that survives a power cycle.
 *
 * This module exists to answer one question: what was the firmware doing
 * during the hours before it stopped responding? Console logging cannot
 * answer it - the failure shows up overnight, and opening a serial monitor
 * usually toggles DTR/RTS, which resets the board and destroys the evidence
 * before it can be read.
 *
 * So events are also appended to a ring buffer in the unused `storage` flash
 * partition. Flash survives unplugging the device, which is exactly the
 * recovery step being investigated: pull the dongle, plug it back in, and the
 * whole history from before the wedge is still there and is dumped to the
 * console on the next boot.
 *
 * Nothing here changes behaviour. Counters are plain increments and the
 * persisted writes happen on a dedicated low-priority task, so no caller ever
 * blocks on flash - an erase stalls the CPU for tens of milliseconds with the
 * cache off, which is not something to do from a BLE callback.
 */

#ifndef DIAG_H
#define DIAG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Free-running counters, incremented from several tasks.
 *
 * Deliberately unsynchronised: these are diagnostics, and a lost increment
 * costs nothing compared to taking a lock on the input path.
 */
typedef struct {
    uint32_t connects;          // links established
    uint32_t disconnects;       // links lost
    int      last_disc_reason;  // BLE_ERR_* / BLE_HS_* of the last disconnect
    uint32_t enc_failures;      // encryption could not be established
    uint32_t bond_deletes;      // stored bonds dropped (each costs an NVS write)
    uint32_t repeat_pairings;   // peer re-paired although a bond existed
    uint32_t conn_updates;      // connection parameter updates applied
    uint32_t notifications;     // input reports received from the keyboard
    uint32_t led_writes;        // LED output reports written to the keyboard
    uint32_t tx_drops;          // CH9329 frames dropped on a full queue
    uint32_t silent_episodes;   // spells of "link up but no input"
} diag_counters_t;

extern diag_counters_t g_diag;

/**
 * @brief Open the persistent log, dump what the previous run left behind, and
 *        record a boot marker including the reset reason.
 *
 * Safe to call when the `storage` partition is missing: the console half of
 * the module keeps working and persistence is silently disabled.
 */
esp_err_t diag_init(void);

/**
 * @brief Log to the console and append to the persistent ring buffer.
 *
 * Never blocks on flash. Call for state changes and anomalies, not for
 * anything that happens per keystroke.
 */
void diag_event(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief Print the whole persistent log to the console, oldest entry first.
 */
void diag_dump(void);

/**
 * @brief Seconds since boot.
 */
uint32_t diag_uptime_s(void);

/**
 * @brief Format uptime as "1d02h03m04s" into @p out (at least 16 bytes).
 */
void diag_format_uptime(uint32_t seconds, char *out, size_t out_len);

#ifdef __cplusplus
}
#endif

#endif /* DIAG_H */
