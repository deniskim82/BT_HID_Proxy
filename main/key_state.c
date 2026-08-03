/**
 * @file key_state.c
 * @brief Composite keyboard state implementation
 */

#include "key_state.h"
#include <string.h>

typedef struct {
    bool active;
    uint8_t modifiers;
    uint8_t keys[KEY_STATE_MAX_KEYS];
    int num_keys;
} device_state_t;

static device_state_t s_devices[KEY_STATE_MAX_DEVICES];

/* The six boot-report slots. 0 = free. A key keeps its slot until released. */
static uint8_t s_slots[KEY_STATE_SLOTS];

static uint8_t s_last_report[8];
static bool s_last_valid;
static int s_overflow;

void key_state_reset(void)
{
    memset(s_devices, 0, sizeof(s_devices));
    memset(s_slots, 0, sizeof(s_slots));
    memset(s_last_report, 0, sizeof(s_last_report));
    s_last_valid = false;
    s_overflow = 0;
}

void key_state_device_update(int device, uint8_t modifiers,
                             const uint8_t *keys, int count)
{
    if (device < 0 || device >= KEY_STATE_MAX_DEVICES) {
        return;
    }

    device_state_t *d = &s_devices[device];
    d->active = true;
    d->modifiers = modifiers;
    d->num_keys = 0;

    for (int i = 0; i < count && d->num_keys < KEY_STATE_MAX_KEYS; i++) {
        if (keys[i] != 0) {
            d->keys[d->num_keys++] = keys[i];
        }
    }
}

void key_state_device_clear(int device)
{
    if (device < 0 || device >= KEY_STATE_MAX_DEVICES) {
        return;
    }
    memset(&s_devices[device], 0, sizeof(s_devices[device]));
}

static bool key_is_held(uint8_t key)
{
    for (int i = 0; i < KEY_STATE_MAX_DEVICES; i++) {
        const device_state_t *d = &s_devices[i];
        if (!d->active) {
            continue;
        }
        for (int k = 0; k < d->num_keys; k++) {
            if (d->keys[k] == key) {
                return true;
            }
        }
    }
    return false;
}

static bool key_has_slot(uint8_t key)
{
    for (int i = 0; i < KEY_STATE_SLOTS; i++) {
        if (s_slots[i] == key) {
            return true;
        }
    }
    return false;
}

bool key_state_build_report(uint8_t out[8])
{
    // Release slots whose key is no longer held anywhere. Slots are freed in
    // place rather than compacted, so the keys that are still down keep the
    // positions they were assigned when pressed.
    for (int i = 0; i < KEY_STATE_SLOTS; i++) {
        if (s_slots[i] != 0 && !key_is_held(s_slots[i])) {
            s_slots[i] = 0;
        }
    }

    // Assign a slot to every newly pressed key, in the order the devices
    // reported them. Anything that does not fit is dropped: the alternative
    // the HID spec suggests - filling all six slots with ErrorRollOver - would
    // disable every key that is currently working, which is worse to use.
    s_overflow = 0;

    for (int i = 0; i < KEY_STATE_MAX_DEVICES; i++) {
        const device_state_t *d = &s_devices[i];
        if (!d->active) {
            continue;
        }
        for (int k = 0; k < d->num_keys; k++) {
            uint8_t key = d->keys[k];
            if (key_has_slot(key)) {
                continue;
            }

            int free_slot = -1;
            for (int s = 0; s < KEY_STATE_SLOTS; s++) {
                if (s_slots[s] == 0) {
                    free_slot = s;
                    break;
                }
            }

            if (free_slot >= 0) {
                s_slots[free_slot] = key;
            } else {
                s_overflow++;
            }
        }
    }

    uint8_t report[8] = {0};

    // Modifiers are OR-ed, so one keyboard's held Shift applies to another
    // keyboard's keys. They live in their own byte and never compete for the
    // six key slots.
    for (int i = 0; i < KEY_STATE_MAX_DEVICES; i++) {
        if (s_devices[i].active) {
            report[0] |= s_devices[i].modifiers;
        }
    }

    for (int i = 0; i < KEY_STATE_SLOTS; i++) {
        report[2 + i] = s_slots[i];
    }

    memcpy(out, report, 8);

    bool changed = !s_last_valid || memcmp(report, s_last_report, 8) != 0;
    memcpy(s_last_report, report, 8);
    s_last_valid = true;

    return changed;
}

int key_state_overflow_count(void)
{
    return s_overflow;
}
