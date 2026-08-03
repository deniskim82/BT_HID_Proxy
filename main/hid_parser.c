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

#define USAGE_PAGE_GEN_DESKTOP 0x01
#define USAGE_PAGE_KEYBOARD 0x07
#define USAGE_PAGE_LED      0x08
#define USAGE_PAGE_CONSUMER 0x0C

#define MAIN_ITEM_VARIABLE  0x02

/* System control usages live in this Generic Desktop range (Power/Sleep/Wake) */
#define SYSTEM_USAGE_MIN    0x81
#define SYSTEM_USAGE_MAX    0xB7

#define MAIN_ITEM_CONSTANT  0x01

// Track bit cursors for up to this many distinct report IDs (per direction)
#define MAX_TRACKED_REPORTS 12

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

/**
 * @brief Find the layout for a report ID, creating it if there is room.
 */
static hid_kb_layout_t *layout_get(hid_report_map_info_t *out, uint8_t report_id)
{
    for (int i = 0; i < out->num_kbs; i++) {
        if (out->kbs[i].report_id == report_id) {
            return &out->kbs[i];
        }
    }
    if (out->num_kbs < HID_MAX_KB_REPORTS) {
        hid_kb_layout_t *kb = &out->kbs[out->num_kbs++];
        memset(kb, 0, sizeof(*kb));
        kb->report_id = report_id;
        return kb;
    }
    return NULL;
}

static hid_ext_layout_t *ext_get(hid_report_map_info_t *out, uint8_t report_id,
                                 hid_ext_kind_t kind)
{
    for (int i = 0; i < out->num_exts; i++) {
        if (out->exts[i].report_id == report_id && out->exts[i].kind == kind) {
            return &out->exts[i];
        }
    }
    if (out->num_exts < HID_MAX_EXT_REPORTS) {
        hid_ext_layout_t *e = &out->exts[out->num_exts++];
        memset(e, 0, sizeof(*e));
        e->report_id = report_id;
        e->kind = kind;
        return e;
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
    uint32_t usage_min = 0;
    bool usage_min_set = false;

    // Explicit Usage() items. Descriptors commonly enumerate the usages one by
    // one and then give a Usage Minimum of 1 rather than the real first usage,
    // so the explicit list has to win when it is present.
    uint32_t usages[8];
    int num_usages = 0;

    report_cursor_t in_cursors[MAX_TRACKED_REPORTS];
    int in_cursor_count = 0;
    report_cursor_t out_cursors[MAX_TRACKED_REPORTS];
    int out_cursor_count = 0;

    size_t pos = 0;
    while (pos < desc_len) {
        uint8_t prefix = desc[pos];

        if (prefix == ITEM_LONG) {
            // Long item: data size byte follows the tag byte
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
        case ITEM_USAGE:
            if (num_usages < (int)(sizeof(usages) / sizeof(usages[0]))) {
                usages[num_usages++] = value;
            }
            break;
        case ITEM_USAGE_MIN:
            usage_min = value;
            usage_min_set = true;
            break;

        case ITEM_INPUT: {
            uint16_t *cursor = cursor_get(in_cursors, &in_cursor_count, report_id);
            bool constant = (value & MAIN_ITEM_CONSTANT) != 0;

            if (cursor != NULL) {
                if (!constant && usage_page == USAGE_PAGE_KEYBOARD) {
                    hid_kb_layout_t *kb = layout_get(out, report_id);

                    if (kb != NULL) {
                        if (report_size == 1 && usage_min_set && usage_min >= 0xE0) {
                            // Modifier bits (usages 0xE0-0xE7)
                            kb->mod_present = true;
                            kb->mod_bit_offset = *cursor;
                        } else if (report_size == 1 && report_count > 1) {
                            // NKRO bitmap
                            if (kb->keys_kind == HID_KEYS_NONE) {
                                kb->keys_kind = HID_KEYS_BITMAP;
                                kb->keys_bit_offset = *cursor;
                                kb->keys_count = (uint16_t)report_count;
                                kb->keys_usage_min = usage_min_set ? (uint8_t)usage_min : 0;
                            }
                        } else if (report_size == 8) {
                            // Keycode array (boot-style)
                            if (kb->keys_kind == HID_KEYS_NONE) {
                                kb->keys_kind = HID_KEYS_ARRAY;
                                kb->keys_bit_offset = *cursor;
                                kb->keys_count = (uint16_t)report_count;
                            }
                        }
                    }
                }

                if (!constant) {
                    hid_ext_kind_t kind = HID_EXT_NONE;

                    if (usage_page == USAGE_PAGE_CONSUMER) {
                        kind = HID_EXT_CONSUMER;
                    } else if (usage_page == USAGE_PAGE_GEN_DESKTOP) {
                        // Generic Desktop also covers mouse axes, so only the
                        // system-control usage range qualifies. Check the
                        // explicit usages first - descriptors often list
                        // 0x81/0x82/0x83 and then declare Usage Minimum 1.
                        for (int u = 0; u < num_usages; u++) {
                            if (usages[u] >= SYSTEM_USAGE_MIN &&
                                usages[u] <= SYSTEM_USAGE_MAX) {
                                kind = HID_EXT_SYSTEM;
                                break;
                            }
                        }
                        if (kind == HID_EXT_NONE && usage_min_set &&
                            usage_min >= SYSTEM_USAGE_MIN &&
                            usage_min <= SYSTEM_USAGE_MAX) {
                            kind = HID_EXT_SYSTEM;
                        }
                    }

                    if (kind != HID_EXT_NONE) {
                        hid_ext_layout_t *e = ext_get(out, report_id, kind);
                        if (e != NULL && e->count == 0) {
                            bool variable = (value & MAIN_ITEM_VARIABLE) != 0;
                            if (variable && report_size == 1) {
                                e->is_bitmap = true;
                                e->count = (uint16_t)report_count;
                                // Explicit usages win: Usage Minimum is often
                                // a placeholder like 1 in these descriptors.
                                e->usage_min = (num_usages > 0) ? usages[0]
                                             : (usage_min_set ? usage_min : 0);
                            } else if (!variable &&
                                       (report_size == 8 || report_size == 16)) {
                                e->is_bitmap = false;
                                e->count = (uint16_t)report_count;
                                e->item_bits = (uint16_t)report_size;
                            }
                            if (e->count != 0) {
                                e->bit_offset = *cursor;
                                e->valid = true;
                            }
                        }
                    }
                }

                *cursor += report_size * report_count;

                for (int i = 0; i < out->num_exts; i++) {
                    if (out->exts[i].report_id == report_id) {
                        out->exts[i].report_bits = *cursor;
                    }
                }

                hid_kb_layout_t *kb = NULL;
                for (int i = 0; i < out->num_kbs; i++) {
                    if (out->kbs[i].report_id == report_id) {
                        kb = &out->kbs[i];
                        break;
                    }
                }
                if (kb != NULL) {
                    kb->report_bits = *cursor;
                }
            }

            usage_min_set = false;
            num_usages = 0;
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
            num_usages = 0;
            break;
        }

        case ITEM_FEATURE:
        case ITEM_COLLECTION:
        case ITEM_END_COLLECTION:
            usage_min_set = false;
            num_usages = 0;
            break;

        default:
            break;
        }

        pos += 1 + item_size;
    }

    // Drop entries that turned out to have no key field (modifiers only)
    int kept = 0;
    for (int i = 0; i < out->num_kbs; i++) {
        if (out->kbs[i].keys_kind != HID_KEYS_NONE) {
            out->kbs[i].valid = true;
            if (kept != i) {
                out->kbs[kept] = out->kbs[i];
            }
            kept++;
        }
    }
    out->num_kbs = kept;

    ESP_LOGI(TAG, "Report map: %d bytes, %d keyboard input report(s)",
             (int)desc_len, out->num_kbs);

    for (int i = 0; i < out->num_kbs; i++) {
        const hid_kb_layout_t *kb = &out->kbs[i];
        ESP_LOGI(TAG, "  [%d] id=%u %s mod@%d keys@%u count=%u usage_min=0x%02X bits=%u",
                 i, kb->report_id,
                 kb->keys_kind == HID_KEYS_ARRAY ? "array" : "bitmap",
                 kb->mod_present ? (int)kb->mod_bit_offset : -1,
                 kb->keys_bit_offset, kb->keys_count,
                 kb->keys_usage_min, kb->report_bits);
    }

    int ext_kept = 0;
    for (int i = 0; i < out->num_exts; i++) {
        if (out->exts[i].valid && out->exts[i].count > 0) {
            if (ext_kept != i) {
                out->exts[ext_kept] = out->exts[i];
            }
            ext_kept++;
        }
    }
    out->num_exts = ext_kept;

    for (int i = 0; i < out->num_exts; i++) {
        const hid_ext_layout_t *e = &out->exts[i];
        ESP_LOGI(TAG, "  ext[%d] id=%u %s %s off=%u count=%u item_bits=%u min=0x%02X",
                 i, e->report_id,
                 e->kind == HID_EXT_CONSUMER ? "consumer" : "system",
                 e->is_bitmap ? "bitmap" : "array",
                 e->bit_offset, e->count, e->item_bits,
                 (unsigned)e->usage_min);
    }

    if (out->has_led_output) {
        ESP_LOGI(TAG, "  LED output report id=%u", out->led_report_id);
    }

    if (out->num_kbs == 0) {
        ESP_LOGW(TAG, "No keyboard input report found in report map");
    }

    return out->num_kbs > 0;
}

const hid_kb_layout_t *hid_parser_find_layout(const hid_report_map_info_t *info,
                                              uint8_t report_id)
{
    if (info == NULL) {
        return NULL;
    }
    for (int i = 0; i < info->num_kbs; i++) {
        if (info->kbs[i].report_id == report_id) {
            return &info->kbs[i];
        }
    }
    return NULL;
}

const hid_ext_layout_t *hid_parser_find_ext(const hid_report_map_info_t *info,
                                            uint8_t report_id)
{
    if (info == NULL) {
        return NULL;
    }
    for (int i = 0; i < info->num_exts; i++) {
        if (info->exts[i].valid && info->exts[i].report_id == report_id) {
            return &info->exts[i];
        }
    }
    return NULL;
}

int hid_parser_extract_usages(const hid_ext_layout_t *layout,
                              const uint8_t *data, size_t len,
                              uint16_t *out, int max_out)
{
    if (layout == NULL || !layout->valid || data == NULL || out == NULL) {
        return 0;
    }

    int n = 0;

    if (layout->is_bitmap) {
        for (uint16_t i = 0; i < layout->count && n < max_out; i++) {
            size_t bit = layout->bit_offset + i;
            size_t byte = bit / 8;
            if (byte >= len) {
                break;
            }
            if (data[byte] & (1u << (bit % 8))) {
                out[n++] = (uint16_t)(layout->usage_min + i);
            }
        }
        return n;
    }

    // Array of usage codes; zero means "no key"
    if ((layout->bit_offset % 8) != 0) {
        return 0;
    }
    size_t pos = layout->bit_offset / 8;
    size_t step = layout->item_bits / 8;

    for (uint16_t i = 0; i < layout->count && n < max_out; i++) {
        size_t off = pos + (size_t)i * step;
        if (off + step > len) {
            break;
        }
        uint16_t usage = data[off];
        if (step == 2) {
            usage |= (uint16_t)data[off + 1] << 8;   // little-endian
        }
        if (usage != 0) {
            out[n++] = usage;
        }
    }
    return n;
}

static inline uint8_t get_byte_at_bit(const uint8_t *data, size_t len, uint16_t bit_offset)
{
    // All real-world keyboard fields are byte-aligned; anything else reads 0.
    if ((bit_offset % 8) != 0) {
        return 0;
    }
    size_t byte = bit_offset / 8;
    return (byte < len) ? data[byte] : 0;
}

int hid_parser_extract_keys(const hid_kb_layout_t *layout,
                            const uint8_t *data, size_t len,
                            uint8_t *mod_out, uint8_t *keys_out, int max_keys)
{
    if (layout == NULL || data == NULL || keys_out == NULL || !layout->valid) {
        return -1;
    }

    if (mod_out != NULL) {
        *mod_out = layout->mod_present
                 ? get_byte_at_bit(data, len, layout->mod_bit_offset) : 0;
    }

    int n = 0;

    if (layout->keys_kind == HID_KEYS_ARRAY) {
        if ((layout->keys_bit_offset % 8) != 0) {
            return -1;
        }
        size_t start = layout->keys_bit_offset / 8;
        for (uint16_t i = 0; i < layout->keys_count && n < max_keys; i++) {
            if (start + i >= len) {
                break;
            }
            uint8_t key = data[start + i];
            if (key != 0) {
                keys_out[n++] = key;
            }
        }
        return n;
    }

    if (layout->keys_kind == HID_KEYS_BITMAP) {
        for (uint16_t i = 0; i < layout->keys_count && n < max_keys; i++) {
            size_t bit = layout->keys_bit_offset + i;
            size_t byte = bit / 8;
            if (byte >= len) {
                break;
            }
            if (data[byte] & (1u << (bit % 8))) {
                keys_out[n++] = layout->keys_usage_min + (uint8_t)i;
            }
        }
        return n;
    }

    return -1;
}
