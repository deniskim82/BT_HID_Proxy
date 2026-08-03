/**
 * @file key_state.h
 * @brief Composite keyboard state: merges one or more source keyboards into
 *        the single boot report the USB side exposes.
 *
 * The PC sees exactly one keyboard (the CH9329), so forwarding each source
 * device's report verbatim is wrong as soon as there is more than one source:
 * device B's report does not contain device A's held modifier, so relaying it
 * tells the host that A released it. Instead each device's state is stored
 * separately here and the whole boot report is recomputed on every change.
 *
 * Slot discipline follows what real 6KRO keyboard firmware does: a pressed key
 * is assigned the first free slot in the 6-key array and keeps that slot until
 * it is released. Nothing is compacted or re-sorted. That keeps held keys
 * stable without needing any separate ordering structure - the array is the
 * ordering structure.
 *
 * This matters even with a single keyboard: an NKRO source can report more
 * than six keys, and truncating the bitmap scan (which runs in keycode order,
 * not press order) lets a held key get bumped out of the report when an
 * unrelated key changes.
 */

#ifndef KEY_STATE_H
#define KEY_STATE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Independent report sources that can be merged.
 *
 * This counts *reports*, not devices: one keyboard commonly exposes both a
 * 6KRO and an NKRO keyboard report and sends on whichever suits the moment.
 * Those two carry independent state - while one holds keys the other reads as
 * empty - so they must not share a slot, or each empty report would wipe the
 * other's keys and the host would see the whole chord released and pressed
 * again, over and over.
 */
#define KEY_STATE_MAX_SOURCES   8

/** Keys tracked per device before overflow (well above any realistic chord) */
#define KEY_STATE_MAX_KEYS      16

/** Key slots in a USB boot report */
#define KEY_STATE_SLOTS         6

/**
 * @brief Forget everything (e.g. on CH9329 re-init).
 */
void key_state_reset(void);

/**
 * @brief Replace one source's currently pressed keys.
 *
 * @param source    Report source index, 0 .. KEY_STATE_MAX_SOURCES-1
 * @param modifiers Modifier byte carried by that report
 * @param keys      Non-modifier keycodes currently held in that report
 * @param count     Number of entries in @p keys
 */
void key_state_source_update(int source, uint8_t modifiers,
                             const uint8_t *keys, int count);

/**
 * @brief Drop a source's contribution entirely (disconnect).
 */
void key_state_source_clear(int source);

/**
 * @brief Build the merged 8-byte boot report.
 *
 * Modifiers are OR-ed across sources - so a modifier held on one keyboard
 * applies to keys typed on another. Keycodes are the union, capped at six.
 *
 * @param out Receives [mod, 0, k1..k6]
 * @return true if the report differs from the previously built one
 */
bool key_state_build_report(uint8_t out[8]);

/**
 * @brief Number of keycodes that did not fit in the six slots.
 *
 * Non-zero only while more than six non-modifier keys are held. Modifiers are
 * a separate field and are never affected.
 */
int key_state_overflow_count(void);

#ifdef __cplusplus
}
#endif

#endif /* KEY_STATE_H */
