/**
 * @file hid_parser.h
 * @brief Minimal HID report descriptor parser for keyboard devices
 *
 * Parses a HID report map just far enough to answer three questions:
 *  1. Which input report IDs carry keyboard keys, and what is each layout?
 *  2. Is a layout a standard key array (boot-like) or an NKRO bitmap?
 *  3. Which output report ID carries the LED (Caps/Num/Scroll) state?
 *
 * A keyboard commonly declares SEVERAL keyboard input reports (e.g. a 6KRO
 * boot-style report plus an NKRO bitmap report) and sends on whichever one
 * matches its current mode. All of them are reported here, and the host
 * subscribes to all of them - picking only one is how the previous version
 * ended up connected but silent on NKRO keyboards.
 *
 * Reports that are not positively identified as keyboard input are never
 * returned, which is what keeps media/vendor/handshake reports from being
 * mistaken for keystrokes.
 */

#ifndef HID_PARSER_H
#define HID_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of distinct keyboard input reports tracked per device */
#define HID_MAX_KB_REPORTS  4

typedef enum {
    HID_KEYS_NONE = 0,
    HID_KEYS_ARRAY,     // Array of 8-bit keycodes (boot-style, 6KRO)
    HID_KEYS_BITMAP,    // Bitmap, one bit per keycode (NKRO)
} hid_keys_kind_t;

typedef struct {
    bool valid;

    uint8_t report_id;          // 0 = device does not use report IDs

    // Modifier field (usually 8 bits, usages 0xE0-0xE7)
    bool mod_present;
    uint16_t mod_bit_offset;    // Bit offset within the report payload

    // Key field
    hid_keys_kind_t keys_kind;
    uint16_t keys_bit_offset;
    uint16_t keys_count;        // Array: number of keycode slots. Bitmap: number of bits.
    uint8_t keys_usage_min;     // Bitmap only: usage of bit 0

    uint16_t report_bits;       // Total payload size in bits
} hid_kb_layout_t;

/** Maximum number of non-keyboard usage reports tracked (consumer + system) */
#define HID_MAX_EXT_REPORTS 4

typedef enum {
    HID_EXT_NONE = 0,
    HID_EXT_CONSUMER,   // Usage page 0x0C - media/browser/application keys
    HID_EXT_SYSTEM,     // Generic Desktop system control - power/sleep/wake
} hid_ext_kind_t;

/**
 * @brief Layout of an input report carrying non-keyboard usages.
 *
 * Two encodings occur in practice: an array of usage codes (Report Size 8 or
 * 16, "Input (Array)"), and a bitmap with one bit per usage.
 */
typedef struct {
    bool valid;
    uint8_t report_id;
    hid_ext_kind_t kind;

    bool is_bitmap;
    uint16_t bit_offset;
    uint16_t count;       // bitmap: number of bits; array: number of fields
    uint16_t item_bits;   // array: bits per field (8 or 16)
    uint32_t usage_min;   // bitmap: usage of bit 0
    uint16_t report_bits;
} hid_ext_layout_t;

typedef struct {
    hid_kb_layout_t kbs[HID_MAX_KB_REPORTS];    // Keyboard input report layouts
    int num_kbs;

    hid_ext_layout_t exts[HID_MAX_EXT_REPORTS]; // Consumer / system control
    int num_exts;

    bool has_led_output;
    uint8_t led_report_id;      // Output report carrying LED usages (page 0x08)
} hid_report_map_info_t;

/**
 * @brief Parse a HID report descriptor.
 *
 * @param desc      Report descriptor bytes
 * @param desc_len  Descriptor length
 * @param out       Parsed result (zeroed on entry)
 * @return true if at least one keyboard input report was identified
 */
bool hid_parser_parse_report_map(const uint8_t *desc, size_t desc_len,
                                 hid_report_map_info_t *out);

/**
 * @brief Look up a parsed layout by report ID.
 * @return NULL if that report ID is not a keyboard input report
 */
const hid_kb_layout_t *hid_parser_find_layout(const hid_report_map_info_t *info,
                                              uint8_t report_id);

/**
 * @brief Look up a parsed non-keyboard usage report by report ID.
 * @return NULL if that report ID is not a consumer or system control report
 */
const hid_ext_layout_t *hid_parser_find_ext(const hid_report_map_info_t *info,
                                            uint8_t report_id);

/**
 * @brief Extract the currently pressed usages from a consumer/system report.
 *
 * @param layout   Layout from hid_parser_parse_report_map()
 * @param data     Raw report payload (without report ID prefix)
 * @param len      Payload length in bytes
 * @param out      Receives the active usage codes
 * @param max_out  Capacity of @p out
 * @return number of usages written (0 = all keys released)
 */
int hid_parser_extract_usages(const hid_ext_layout_t *layout,
                              const uint8_t *data, size_t len,
                              uint16_t *out, int max_out);

/**
 * @brief Translate a raw keyboard input report to an 8-byte boot report.
 *
 * @param layout  Layout from hid_parser_parse_report_map()
 * @param data    Raw report payload (without report ID prefix)
 * @param len     Payload length in bytes
 * @param boot    Output: 8-byte boot report [mod, 0, k1..k6]
 * @return true on success
 */
bool hid_parser_to_boot_report(const hid_kb_layout_t *layout,
                               const uint8_t *data, size_t len,
                               uint8_t boot[8]);

#ifdef __cplusplus
}
#endif

#endif /* HID_PARSER_H */
