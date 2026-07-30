/**
 * @file hid_parser.h
 * @brief Minimal HID report descriptor parser for keyboard devices
 *
 * Parses a HID report map just far enough to answer three questions:
 *  1. Which input report ID carries keyboard keys, and what is its layout?
 *  2. Is that layout a standard key array (boot-like) or an NKRO bitmap?
 *  3. Which output report ID carries the LED (Caps/Num/Scroll) state?
 *
 * This is the structural fix for the "random key repeats forever" bug of the
 * previous implementation: only reports positively identified as keyboard
 * input are ever translated and forwarded to the USB side.
 */

#ifndef HID_PARSER_H
#define HID_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

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

    uint16_t report_bits;       // Total payload size in bits (for length sanity check)
} hid_kb_layout_t;

typedef struct {
    hid_kb_layout_t kb;         // Keyboard input report layout

    bool has_led_output;
    uint8_t led_report_id;      // Output report carrying LED usages (page 0x08)
} hid_report_map_info_t;

/**
 * @brief Parse a HID report descriptor.
 *
 * @param desc      Report descriptor bytes
 * @param desc_len  Descriptor length
 * @param out       Parsed result (zeroed on entry)
 * @return true if a keyboard input report was identified
 */
bool hid_parser_parse_report_map(const uint8_t *desc, size_t desc_len,
                                 hid_report_map_info_t *out);

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
