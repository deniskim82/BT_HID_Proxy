/**
 * @file hid_parser.c
 * @brief Minimal HID report descriptor parser for keyboard devices
 */

#include "hid_parser.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "HID_PARSER";

/* HID item tags (prefix byte with size bits masked off) */
#define ITEM_USAGE_PAGE     0x04
#define ITEM_LOGICAL_MIN    0x14
#define ITEM_LOGICAL_MAX    0x24
#define ITEM_REPORT_SIZE    0x74
#define ITEM_REPORT_ID      0x84
#define ITEM_REPORT_COUNT   0x94
#define ITEM_INPUT          0x80
#define ITEM_OUTPUT         0x90
#define ITEM_FEATURE        0xB0
#define ITEM_COLLECTION     0xA0
#define ITEM_END_COLLECTION 0xC0
#define ITEM_USAGE          0x08
#define ITEM_USAGE_MIN      0x18
#define ITEM_USAGE_MAX      0x28
#define ITEM_LONG           0xFC

#define USAGE_PAGE_KEYBOARD 0x07
#define USAGE_PAGE_LED      0x08

#define MAIN_ITEM_CONSTANT  0x01

// Track bit cursors for up to this many distinct report IDs (per direction)
#define MAX_TRACKED_REPORTS 8

typedef struct {
    uint8_t report_id;
    uint16_t bits;
} report_cursor_t;

static uint16_t *cursor_get(report_cursor_t *cursors, int *count, uint8_t report_id)
{
    for (int i = 0; i < *count; i++) {
        if (cursors[i].report_id == report_id) {
            return &cursors[i].bits;
        }
    }
    if (*count < MAX_TRACKED_REPORTS) {
        cursors[*count].report_id = report_id;
        cursors[*count].bits = 0;
        return &cursors[(*count)++].bits;
    }
    return NULL;
}

bool hid_parser_parse_report_map(const uint8_t *desc, size_t desc_len,
                                 hid_report_map_info_t *out)
{
    if (desc == NULL || out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    // Global state
    uint16_t usage_page = 0;
    uint8_t report_id = 0;
    uint32_t report_size = 0;
    uint32_t report_count = 0;

    // Local state (reset after each main item)
    uint32_t usage_min = 0, usage_max = 0;
    bool usage_min_set = false;

    report_cursor_t in_cursors[MAX_TRACKED_REPORTS];
    int in_cursor_count = 0;
    report_cursor_t out_cursors[MAX_TRACKED_REPORTS];
    int out_cursor_count = 0;

    size_t pos = 0;
    while (pos < desc_len) {
        uint8_t prefix = desc[pos];

        if (prefix == ITEM_LONG) {
            // Long item: skip (size byte follows tag byte)
            if (pos + 2 >= desc_len) {
                break;
            }
            pos += 3 + desc[pos + 1];
            continue;
        }

        uint8_t size_code = prefix & 0x03;
        uint8_t item_size = (size_code == 3) ? 4 : size_code;
        uint8_t tag = prefix & 0xFC;

        if (pos + 1 + item_size > desc_len) {
            break;
        }

        uint32_t value = 0;
        for (int i = 0; i < item_size; i++) {
            value |= ((uint32_t)desc[pos + 1 + i]) << (8 * i);
        }

        switch (tag) {
        case ITEM_USAGE_PAGE:
            usage_page = (uint16_t)value;
            break;
        case ITEM_REPORT_ID:
            report_id = (uint8_t)value;
            break;
        case ITEM_REPORT_SIZE:
            report_size = value;
            break;
        case ITEM_REPORT_COUNT:
            report_count = value;
            break;
        case ITEM_USAGE_MIN:
            usage_min = value;
            usage_min_set = true;
            break;
        case ITEM_USAGE_MAX:
            usage_max = value;
            break;

        case ITEM_INPUT: {
            uint16_t *cursor = cursor_get(in_cursors, &in_cursor_count, report_id);
            uint32_t bits = report_size * report_count;
            bool constant = (value & MAIN_ITEM_CONSTANT) != 0;

            if (cursor != NULL) {
                if (!constant && usage_page == USAGE_PAGE_KEYBOARD) {
                    hid_kb_layout_t *kb = &out->kb;

                    // A keyboard report may span several main items with the
                    // same report ID; bind the layout to the first ID that
                    // yields keyboard usages.
                    if (!kb->valid || kb->report_id == report_id) {
                        kb->report_id = report_id;

                        if (report_size == 1 && usage_min_set && usage_min >= 0xE0) {
                            // Modifier bits (usages 0xE0-0xE7)
                            kb->mod_present = true;
                            kb->mod_bit_offset = *cursor;
                            kb->valid = true;
                        } else if (report_size == 1 && report_count > 1) {
                            // NKRO bitmap
                            if (kb->keys_kind == HID_KEYS_NONE) {
                                kb->keys_kind = HID_KEYS_BITMAP;
                                kb->keys_bit_offset = *cursor;
                                kb->keys_count = (uint16_t)report_count;
                                kb->keys_usage_min = usage_min_set ? (uint8_t)usage_min : 0;
                                kb->valid = true;
                            }
                        } else if (report_size == 8) {
                            // Keycode array (boot-style)
                            // Prefer an array over a previously seen bitmap:
                            // some keyboards expose both, array first is rare,
                            // so only take the first keys field we see.
                            if (kb->keys_kind == HID_KEYS_NONE) {
                                kb->keys_kind = HID_KEYS_ARRAY;
                                kb->keys_bit_offset = *cursor;
                                kb->keys_count = (uint16_t)report_count;
                                kb->valid = true;
                            }
                        }
                    }
                }

                *cursor += bits;

                if (out->kb.valid && out->kb.report_id == report_id) {
                    out->kb.report_bits = *cursor;
                }
            }

            usage_min_set = false;
            usage_max = 0;
            (void)usage_max;
            break;
        }

        case ITEM_OUTPUT: {
            uint16_t *cursor = cursor_get(out_cursors, &out_cursor_count, report_id);
            bool constant = (value & MAIN_ITEM_CONSTANT) != 0;

            if (!constant && usage_page == USAGE_PAGE_LED && !out->has_led_output) {
                out->has_led_output = true;
                out->led_report_id = report_id;
            }

            if (cursor != NULL) {
                *cursor += report_size * report_count;
            }

            usage_min_set = false;
            break;
        }

        case ITEM_FEATURE:
        case ITEM_COLLECTION:
        case ITEM_END_COLLECTION:
            usage_min_set = false;
            break;

        default:
            break;
        }

        pos += 1 + item_size;
    }

    if (out->kb.valid && out->kb.keys_kind == HID_KEYS_NONE) {
        // Modifiers only, no key field found - not usable
        out->kb.valid = false;
    }

    if (out->kb.valid) {
        ESP_LOGI(TAG, "Keyboard input report: id=%d kind=%s mod@%d keys@%d count=%d bits=%d",
                 out->kb.report_id,
                 out->kb.keys_kind == HID_KEYS_ARRAY ? "array" : "bitmap",
                 out->kb.mod_present ? out->kb.mod_bit_offset : -1,
                 out->kb.keys_bit_offset, out->kb.keys_count, out->kb.report_bits);
    } else {
        ESP_LOGW(TAG, "No keyboard input report found in report map (%d bytes)", (int)desc_len);
    }

    if (out->has_led_output) {
        ESP_LOGI(TAG, "LED output report: id=%d", out->led_report_id);
    }

    return out->kb.valid;
}

static inline uint8_t get_byte_at_bit(const uint8_t *data, size_t len, uint16_t bit_offset)
{
    // All real-world keyboard fields are byte-aligned; enforce that here and
    // fall back to 0 for anything out of range.
    if ((bit_offset % 8) != 0) {
        return 0;
    }
    size_t byte = bit_offset / 8;
    return (byte < len) ? data[byte] : 0;
}

bool hid_parser_to_boot_report(const hid_kb_layout_t *layout,
                               const uint8_t *data, size_t len,
                               uint8_t boot[8])
{
    if (layout == NULL || data == NULL || boot == NULL || !layout->valid) {
        return false;
    }

    memset(boot, 0, 8);

    if (layout->mod_present) {
        boot[0] = get_byte_at_bit(data, len, layout->mod_bit_offset);
    }

    if (layout->keys_kind == HID_KEYS_ARRAY) {
        size_t start = layout->keys_bit_offset / 8;
        if ((layout->keys_bit_offset % 8) != 0) {
            return false;
        }
        int out_idx = 0;
        for (uint16_t i = 0; i < layout->keys_count && out_idx < 6; i++) {
            if (start + i >= len) {
                break;
            }
            uint8_t key = data[start + i];
            if (key != 0) {
                boot[2 + out_idx++] = key;
            }
        }
        return true;
    }

    if (layout->keys_kind == HID_KEYS_BITMAP) {
        size_t start_bit = layout->keys_bit_offset;
        int out_idx = 0;
        for (uint16_t i = 0; i < layout->keys_count && out_idx < 6; i++) {
            size_t bit = start_bit + i;
            size_t byte = bit / 8;
            if (byte >= len) {
                break;
            }
            if (data[byte] & (1u << (bit % 8))) {
                boot[2 + out_idx++] = layout->keys_usage_min + (uint8_t)i;
            }
        }
        return true;
    }

    return false;
}
