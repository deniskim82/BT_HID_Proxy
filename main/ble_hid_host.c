/**
 * @file ble_hid_host.c
 * @brief BLE HID (HOGP) Host based on NimBLE
 *
 * Structure:
 *  - NimBLE host task: all GAP/GATT events and the discovery state machine
 *  - Control task: orchestrates reconnect cycles and pairing mode
 *
 * Connection sequence:
 *  connect -> exchange MTU -> encrypt (bond) -> discover HID service ->
 *  read report map -> discover report descriptors -> read report references ->
 *  subscribe to the keyboard input report only -> CONNECTED
 *
 * Only notifications from the characteristic positively identified as the
 * keyboard input report are translated and forwarded; everything else
 * (media keys, vendor reports, battery, ...) is ignored by design.
 */

#include "ble_hid_host.h"
#include "hid_parser.h"
#include "key_state.h"
#include "storage.h"
#include "config.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "BLE_HID";

/* NimBLE bond persistence (implemented in the nimble component) */
void ble_store_config_init(void);

/* ============================================================================
 * Constants
 * ============================================================================ */

#define UUID_HID_SERVICE        0x1812
#define UUID_CHR_REPORT_MAP     0x2A4B
#define UUID_CHR_REPORT         0x2A4D
#define UUID_CHR_PROTOCOL_MODE  0x2A4E
#define UUID_CHR_BOOT_KB_IN     0x2A22
#define UUID_CHR_BOOT_KB_OUT    0x2A32
#define UUID_DSC_CCCD           0x2902
#define UUID_DSC_REPORT_REF     0x2908

#define REPORT_TYPE_INPUT       1
#define REPORT_TYPE_OUTPUT      2

#define MAX_HID_CHRS            16
#define REPORT_MAP_MAX_LEN      512

#define EVT_READY               BIT0
#define EVT_FAILED              BIT1
#define EVT_SCAN_DONE           BIT2

#define CTRL_TASK_STACK         4096
#define CTRL_TASK_PRIO          3

typedef enum {
    CTRL_CMD_PAIR = 1,
} ctrl_cmd_t;

/* ============================================================================
 * State
 * ============================================================================ */

typedef struct {
    uint16_t def_handle;
    uint16_t val_handle;
    uint16_t end_handle;
    uint16_t uuid16;
    uint8_t properties;
    // Descriptors (0 = not present)
    uint16_t ccc_handle;
    uint16_t ref_handle;
    // From Report Reference descriptor
    uint8_t report_id;
    uint8_t report_type;
    bool ref_read;
} hid_chr_t;

static bool s_initialized = false;
static uint8_t s_own_addr_type;

static ble_hid_keyboard_cb_t s_keyboard_cb = NULL;
static ble_hid_state_cb_t s_state_cb = NULL;
static ble_hid_ext_cb_t s_ext_cb = NULL;

static volatile ble_hid_state_t s_state = BLE_HID_STATE_IDLE;

// Bonded target keyboard
static bool s_has_target = false;
static ble_addr_t s_target_addr;
static char s_target_name[32] = {0};

// Pairing-mode scan bookkeeping (written by host task, read by control task)
static volatile bool s_pairing_mode = false;
static volatile bool s_best_set = false;
static ble_addr_t s_best_addr;
static int8_t s_best_rssi;
static char s_best_name[32];

// Active connection
static volatile uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool s_enc_retried = false;

// GATT discovery context
static uint16_t s_svc_start_handle, s_svc_end_handle;
static hid_chr_t s_chrs[MAX_HID_CHRS];
static int s_num_chrs = 0;
static int s_disc_idx = 0;
static uint8_t s_report_map[REPORT_MAP_MAX_LEN];
static size_t s_report_map_len = 0;
static hid_report_map_info_t s_map_info;

/**
 * @brief One subscribed keyboard input report.
 *
 * A keyboard typically declares more than one (e.g. 6KRO + NKRO) and sends on
 * whichever matches its current mode, so all of them are subscribed and each
 * notification is decoded with the layout belonging to its own handle.
 */
typedef struct {
    uint16_t val_handle;
    uint16_t ccc_handle;
    const hid_kb_layout_t *layout;  // keyboard report (NULL => see ext/boot)
    const hid_ext_layout_t *ext;    // consumer / system control report
} kb_sub_t;

static kb_sub_t s_subs[HID_MAX_KB_REPORTS + HID_MAX_EXT_REPORTS + 1];
static volatile int s_num_subs = 0;
static int s_sub_idx = 0;

static volatile uint16_t s_led_output_val_handle = 0;

// Diagnostics: log the first few reports after each connection
#define INPUT_LOG_BUDGET    10
static int s_input_log_budget = 0;

// Tasks / sync
static TaskHandle_t s_ctrl_task_handle = NULL;
static QueueHandle_t s_ctrl_queue = NULL;
static EventGroupHandle_t s_events = NULL;
static SemaphoreHandle_t s_sync_sem = NULL;

/* Forward declarations */
static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_discovery(void);
static void fail_connection(void);
static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg);
static int report_map_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg);

/* ============================================================================
 * State helpers
 * ============================================================================ */

static void set_state(ble_hid_state_t new_state)
{
    if (s_state == new_state) {
        return;
    }
    s_state = new_state;
    if (s_state_cb) {
        s_state_cb(new_state);
    }
}

static void reset_disc_ctx(void)
{
    s_svc_start_handle = 0;
    s_svc_end_handle = 0;
    s_num_chrs = 0;
    s_disc_idx = 0;
    s_report_map_len = 0;
    memset(&s_map_info, 0, sizeof(s_map_info));
    s_num_subs = 0;
    s_sub_idx = 0;
    memset(s_subs, 0, sizeof(s_subs));
    s_led_output_val_handle = 0;
    memset(s_chrs, 0, sizeof(s_chrs));
}

static void fail_connection(void)
{
    uint16_t conn = s_conn_handle;
    if (conn != BLE_HS_CONN_HANDLE_NONE) {
        int rc = ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0 && rc != BLE_HS_EALREADY) {
            // Connection already gone and no DISCONNECT event will fire
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            xEventGroupSetBits(s_events, EVT_FAILED);
        }
    } else {
        xEventGroupSetBits(s_events, EVT_FAILED);
    }
}

/* ============================================================================
 * Advertisement filtering
 * ============================================================================ */

static bool adv_is_hid_keyboard(const struct ble_gap_disc_desc *disc, char *name_out, size_t name_len)
{
    struct ble_hs_adv_fields fields;

    if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) != 0) {
        return false;
    }

    bool is_hid = false;

    // Appearance category 0x03C0-0x03FF = HID; 0x03C1 = keyboard
    if (fields.appearance_is_present &&
        (fields.appearance & 0xFFC0) == 0x03C0) {
        is_hid = true;
    }

    for (int i = 0; i < fields.num_uuids16 && !is_hid; i++) {
        if (ble_uuid_u16(&fields.uuids16[i].u) == UUID_HID_SERVICE) {
            is_hid = true;
        }
    }

    if (is_hid && name_out != NULL && fields.name != NULL && fields.name_len > 0) {
        size_t n = fields.name_len < name_len - 1 ? fields.name_len : name_len - 1;
        memcpy(name_out, fields.name, n);
        name_out[n] = '\0';
    }

    return is_hid;
}

/* ============================================================================
 * Scanning / connecting
 * ============================================================================ */

static int start_scan(bool active, int32_t duration_ms)
{
    struct ble_gap_disc_params params = {
        .itvl = 0x0050,             // 50ms
        .window = 0x0030,           // 30ms
        .filter_policy = 0,
        .limited = 0,
        .passive = active ? 0 : 1,
        .filter_duplicates = 0,
    };

    int rc = ble_gap_disc(s_own_addr_type, duration_ms, &params, gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "ble_gap_disc failed: %d", rc);
    }
    return rc;
}

static int connect_to(const ble_addr_t *addr, int32_t timeout_ms)
{
    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
    }

    struct ble_gap_conn_params params = {
        .scan_itvl = 0x0020,        // 20ms
        .scan_window = 0x0020,
        .itvl_min = 6,              // 7.5ms
        .itvl_max = 12,             // 15ms
        .latency = 0,
        .supervision_timeout = 400, // 4s
        .min_ce_len = 0,
        .max_ce_len = 0,
    };

    set_state(BLE_HID_STATE_CONNECTING);

    int rc = ble_gap_connect(s_own_addr_type, addr, timeout_ms, &params,
                             gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        char addr_str[18];
        storage_addr_to_str(addr->val, addr_str);
        ESP_LOGW(TAG, "ble_gap_connect(%s) failed: %d", addr_str, rc);
        xEventGroupSetBits(s_events, EVT_FAILED);
    }
    return rc;
}

/* ============================================================================
 * GATT discovery state machine (runs on NimBLE host task)
 * ============================================================================ */

static void configure_reports(void);
static void next_dsc_discovery(void);
static void next_ref_read(void);

static int mtu_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                  uint16_t mtu, void *arg)
{
    return 0;
}

static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg)
{
    if (error->status == 0 && service != NULL) {
        if (s_svc_start_handle == 0) {
            s_svc_start_handle = service->start_handle;
            s_svc_end_handle = service->end_handle;
        }
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        if (s_svc_start_handle == 0) {
            ESP_LOGW(TAG, "HID service not found");
            fail_connection();
            return 0;
        }
        int rc = ble_gattc_disc_all_chrs(conn_handle, s_svc_start_handle,
                                         s_svc_end_handle, chr_disc_cb, NULL);
        if (rc != 0) {
            ESP_LOGW(TAG, "disc_all_chrs failed: %d", rc);
            fail_connection();
        }
        return 0;
    }

    ESP_LOGW(TAG, "Service discovery failed: %d", error->status);
    fail_connection();
    return 0;
}

static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == 0 && chr != NULL) {
        if (s_num_chrs < MAX_HID_CHRS) {
            hid_chr_t *c = &s_chrs[s_num_chrs++];
            c->def_handle = chr->def_handle;
            c->val_handle = chr->val_handle;
            c->properties = chr->properties;
            c->uuid16 = (chr->uuid.u.type == BLE_UUID_TYPE_16)
                        ? ble_uuid_u16(&chr->uuid.u) : 0;
        }
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        // Compute per-characteristic end handles for descriptor discovery
        for (int i = 0; i < s_num_chrs; i++) {
            s_chrs[i].end_handle = (i + 1 < s_num_chrs)
                                   ? s_chrs[i + 1].def_handle - 1
                                   : s_svc_end_handle;
        }

        // Read the report map next
        uint16_t map_handle = 0;
        for (int i = 0; i < s_num_chrs; i++) {
            if (s_chrs[i].uuid16 == UUID_CHR_REPORT_MAP) {
                map_handle = s_chrs[i].val_handle;
                break;
            }
        }

        if (map_handle == 0) {
            ESP_LOGW(TAG, "No report map characteristic; trying boot fallback");
            s_disc_idx = 0;
            next_dsc_discovery();
            return 0;
        }

        int rc = ble_gattc_read_long(conn_handle, map_handle, 0,
                                     report_map_read_cb, NULL);
        if (rc != 0) {
            ESP_LOGW(TAG, "read_long(report map) failed: %d", rc);
            fail_connection();
        }
        return 0;
    }

    ESP_LOGW(TAG, "Characteristic discovery failed: %d", error->status);
    fail_connection();
    return 0;
}

static int report_map_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg)
{
    if (error->status == 0 && attr != NULL) {
        uint16_t chunk = OS_MBUF_PKTLEN(attr->om);
        if (s_report_map_len + chunk <= REPORT_MAP_MAX_LEN) {
            os_mbuf_copydata(attr->om, 0, chunk, &s_report_map[s_report_map_len]);
            s_report_map_len += chunk;
        }
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        // Dump the raw descriptor: with it, any keyboard that still misbehaves
        // can be diagnosed offline from a single serial capture.
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, s_report_map, s_report_map_len, ESP_LOG_INFO);
        hid_parser_parse_report_map(s_report_map, s_report_map_len, &s_map_info);
        s_disc_idx = 0;
        next_dsc_discovery();
        return 0;
    }

    ESP_LOGW(TAG, "Report map read failed: %d", error->status);
    fail_connection();
    return 0;
}

static bool chr_needs_descriptors(const hid_chr_t *c)
{
    return c->uuid16 == UUID_CHR_REPORT || c->uuid16 == UUID_CHR_BOOT_KB_IN;
}

static int dsc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                       void *arg)
{
    if (error->status == 0 && dsc != NULL) {
        hid_chr_t *c = &s_chrs[s_disc_idx];
        if (dsc->uuid.u.type == BLE_UUID_TYPE_16) {
            uint16_t u = ble_uuid_u16(&dsc->uuid.u);
            if (u == UUID_DSC_CCCD) {
                c->ccc_handle = dsc->handle;
            } else if (u == UUID_DSC_REPORT_REF) {
                c->ref_handle = dsc->handle;
            }
        }
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        s_disc_idx++;
        next_dsc_discovery();
        return 0;
    }

    ESP_LOGW(TAG, "Descriptor discovery failed: %d", error->status);
    fail_connection();
    return 0;
}

static void next_dsc_discovery(void)
{
    while (s_disc_idx < s_num_chrs && !chr_needs_descriptors(&s_chrs[s_disc_idx])) {
        s_disc_idx++;
    }

    if (s_disc_idx >= s_num_chrs) {
        s_disc_idx = 0;
        next_ref_read();
        return;
    }

    hid_chr_t *c = &s_chrs[s_disc_idx];
    if (c->end_handle < c->val_handle + 1) {
        // No room for descriptors
        s_disc_idx++;
        next_dsc_discovery();
        return;
    }

    int rc = ble_gattc_disc_all_dscs(s_conn_handle, c->val_handle, c->end_handle,
                                     dsc_disc_cb, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "disc_all_dscs failed: %d", rc);
        fail_connection();
    }
}

static int ref_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       struct ble_gatt_attr *attr, void *arg)
{
    hid_chr_t *c = (hid_chr_t *)arg;

    if (error->status == 0 && attr != NULL) {
        uint8_t buf[2] = {0, 0};
        uint16_t len = OS_MBUF_PKTLEN(attr->om);
        os_mbuf_copydata(attr->om, 0, len > 2 ? 2 : len, buf);
        c->report_id = buf[0];
        c->report_type = buf[1];
        ESP_LOGD(TAG, "Report ref: handle=%d id=%d type=%d",
                 c->val_handle, c->report_id, c->report_type);
    } else {
        ESP_LOGW(TAG, "Report reference read failed: %d", error->status);
    }

    c->ref_read = true;
    next_ref_read();
    return 0;
}

static void next_ref_read(void)
{
    for (int i = 0; i < s_num_chrs; i++) {
        hid_chr_t *c = &s_chrs[i];
        if (c->uuid16 == UUID_CHR_REPORT && c->ref_handle != 0 && !c->ref_read) {
            int rc = ble_gattc_read(s_conn_handle, c->ref_handle, ref_read_cb, c);
            if (rc != 0) {
                ESP_LOGW(TAG, "read(report ref) failed: %d", rc);
                c->ref_read = true;
                continue;
            }
            return;
        }
    }

    configure_reports();
}

static void subscribe_next(void);

static int subscribe_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg)
{
    if (error->status != 0) {
        // One report failing to subscribe is not fatal as long as another
        // one works - drop it from the list and carry on.
        ESP_LOGW(TAG, "CCCD write failed on handle %d: %d",
                 s_subs[s_sub_idx].val_handle, error->status);
        s_subs[s_sub_idx].val_handle = 0;
    } else {
        const kb_sub_t *sub = &s_subs[s_sub_idx];
        const char *what = "boot keyboard";
        uint8_t rid = 0;
        if (sub->layout != NULL) {
            rid = sub->layout->report_id;
            what = (sub->layout->keys_kind == HID_KEYS_ARRAY) ? "keyboard array"
                                                             : "keyboard bitmap";
        } else if (sub->ext != NULL) {
            rid = sub->ext->report_id;
            what = (sub->ext->kind == HID_EXT_CONSUMER) ? "consumer" : "system";
        }
        ESP_LOGI(TAG, "Subscribed: handle=%d report_id=%d (%s)",
                 sub->val_handle, rid, what);
    }

    s_sub_idx++;
    subscribe_next();
    return 0;
}

static void subscribe_next(void)
{
    while (s_sub_idx < s_num_subs) {
        kb_sub_t *sub = &s_subs[s_sub_idx];

        uint8_t ccc_val[2] = {0x01, 0x00};
        int rc = ble_gattc_write_flat(s_conn_handle, sub->ccc_handle,
                                      ccc_val, sizeof(ccc_val), subscribe_cb, NULL);
        if (rc == 0) {
            return;  // Wait for the callback
        }

        ESP_LOGW(TAG, "CCCD write start failed on handle %d: %d", sub->val_handle, rc);
        sub->val_handle = 0;
        s_sub_idx++;
    }

    // Done: at least one live subscription is required
    int live = 0;
    for (int i = 0; i < s_num_subs; i++) {
        if (s_subs[i].val_handle != 0) {
            live++;
        }
    }

    if (live == 0) {
        ESP_LOGW(TAG, "No keyboard report could be subscribed");
        fail_connection();
        return;
    }

    ESP_LOGI(TAG, "Relaying %d keyboard report(s)", live);
    s_input_log_budget = INPUT_LOG_BUDGET;
    set_state(BLE_HID_STATE_CONNECTED);
    xEventGroupSetBits(s_events, EVT_READY);
}

static hid_chr_t *find_chr_uuid(uint16_t uuid16)
{
    for (int i = 0; i < s_num_chrs; i++) {
        if (s_chrs[i].uuid16 == uuid16) {
            return &s_chrs[i];
        }
    }
    return NULL;
}

static void configure_reports(void)
{
    hid_chr_t *led_chr = NULL;

    // Log what we found, so a keyboard that misbehaves can be diagnosed
    // from a single serial capture.
    for (int i = 0; i < s_num_chrs; i++) {
        hid_chr_t *c = &s_chrs[i];
        if (c->uuid16 == UUID_CHR_REPORT) {
            ESP_LOGI(TAG, "Report chr: handle=%d id=%d type=%d ccc=%d props=0x%02X",
                     c->val_handle, c->report_id, c->report_type,
                     c->ccc_handle, c->properties);
        }
    }

    // Subscribe to EVERY input report the report map identified as keyboard.
    // Keyboards commonly declare both a 6KRO and an NKRO report and send on
    // whichever matches their current mode; subscribing to only one leaves
    // the link up but silent.
    for (int i = 0; i < s_num_chrs && s_num_subs < HID_MAX_KB_REPORTS; i++) {
        hid_chr_t *c = &s_chrs[i];
        if (c->uuid16 != UUID_CHR_REPORT) {
            continue;
        }

        if (s_map_info.has_led_output && led_chr == NULL &&
            c->report_type == REPORT_TYPE_OUTPUT &&
            c->report_id == s_map_info.led_report_id) {
            led_chr = c;
        }

        if (c->report_type != REPORT_TYPE_INPUT || c->ccc_handle == 0) {
            continue;
        }

        const hid_kb_layout_t *layout = hid_parser_find_layout(&s_map_info, c->report_id);
        if (layout == NULL) {
            continue;   // Not a keyboard report - never forwarded
        }

        s_subs[s_num_subs].val_handle = c->val_handle;
        s_subs[s_num_subs].ccc_handle = c->ccc_handle;
        s_subs[s_num_subs].layout = layout;
        s_num_subs++;
    }

    // Media / system control reports: forwarded so volume, playback and
    // power keys work like they would on a directly attached keyboard.
    for (int i = 0; i < s_num_chrs &&
                    s_num_subs < (int)(sizeof(s_subs) / sizeof(s_subs[0])); i++) {
        hid_chr_t *c = &s_chrs[i];
        if (c->uuid16 != UUID_CHR_REPORT ||
            c->report_type != REPORT_TYPE_INPUT || c->ccc_handle == 0) {
            continue;
        }
        const hid_ext_layout_t *ext = hid_parser_find_ext(&s_map_info, c->report_id);
        if (ext == NULL) {
            continue;
        }
        s_subs[s_num_subs].val_handle = c->val_handle;
        s_subs[s_num_subs].ccc_handle = c->ccc_handle;
        s_subs[s_num_subs].layout = NULL;
        s_subs[s_num_subs].ext = ext;
        s_num_subs++;
    }

    // Devices that use a single report without report IDs may omit the Report
    // Reference descriptor entirely.
    if (s_num_subs == 0 && s_map_info.num_kbs == 1 && s_map_info.kbs[0].report_id == 0) {
        hid_chr_t *only = NULL;
        int count = 0;
        for (int i = 0; i < s_num_chrs; i++) {
            hid_chr_t *c = &s_chrs[i];
            if (c->uuid16 == UUID_CHR_REPORT && c->ccc_handle != 0 &&
                c->report_type != REPORT_TYPE_OUTPUT) {
                only = c;
                count++;
            }
        }
        if (count == 1) {
            s_subs[0].val_handle = only->val_handle;
            s_subs[0].ccc_handle = only->ccc_handle;
            s_subs[0].layout = &s_map_info.kbs[0];
            s_num_subs = 1;
        }
    }

    hid_chr_t *proto = find_chr_uuid(UUID_CHR_PROTOCOL_MODE);

    if (s_num_subs > 0) {
        // Report protocol (0x01) is the default, but be explicit: a keyboard
        // left in boot mode by a previous host would otherwise stay silent
        // on the report characteristics we just subscribed to.
        if (proto != NULL) {
            uint8_t mode = 0x01;
            ble_gattc_write_no_rsp_flat(s_conn_handle, proto->val_handle, &mode, 1);
        }

        hid_chr_t *boot_out = find_chr_uuid(UUID_CHR_BOOT_KB_OUT);
        s_led_output_val_handle = (led_chr != NULL) ? led_chr->val_handle
                                : (boot_out != NULL) ? boot_out->val_handle : 0;
    } else {
        // Boot protocol fallback
        hid_chr_t *boot_in = find_chr_uuid(UUID_CHR_BOOT_KB_IN);
        hid_chr_t *boot_out = find_chr_uuid(UUID_CHR_BOOT_KB_OUT);

        if (boot_in == NULL || boot_in->ccc_handle == 0) {
            ESP_LOGW(TAG, "No usable keyboard input report (kb reports=%d, chrs=%d)",
                     s_map_info.num_kbs, s_num_chrs);
            fail_connection();
            return;
        }

        ESP_LOGI(TAG, "Falling back to boot protocol");
        if (proto != NULL) {
            uint8_t mode = 0x00;
            ble_gattc_write_no_rsp_flat(s_conn_handle, proto->val_handle, &mode, 1);
        }

        s_subs[0].val_handle = boot_in->val_handle;
        s_subs[0].ccc_handle = boot_in->ccc_handle;
        s_subs[0].layout = NULL;    // Already in boot format
        s_num_subs = 1;
        s_led_output_val_handle = (boot_out != NULL) ? boot_out->val_handle : 0;
    }

    s_sub_idx = 0;
    subscribe_next();
}

static void start_discovery(void)
{
    reset_disc_ctx();

    int rc = ble_gattc_disc_svc_by_uuid(s_conn_handle,
                                        BLE_UUID16_DECLARE(UUID_HID_SERVICE),
                                        svc_disc_cb, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "disc_svc_by_uuid failed: %d", rc);
        fail_connection();
    }
}

/* ============================================================================
 * Input handling
 * ============================================================================ */

static void handle_input_notification(const kb_sub_t *sub, struct os_mbuf *om)
{
    uint8_t raw[32];
    uint16_t len = OS_MBUF_PKTLEN(om);
    if (len > sizeof(raw)) {
        len = sizeof(raw);
    }
    os_mbuf_copydata(om, 0, len, raw);

    if (sub->ext != NULL) {
        uint16_t usages[8];
        int n = hid_parser_extract_usages(sub->ext, raw, len, usages,
                                          sizeof(usages) / sizeof(usages[0]));
        if (s_input_log_budget > 0) {
            s_input_log_budget--;
            ESP_LOGI(TAG, "IN %s handle=%d len=%d usages=%d first=0x%04X",
                     sub->ext->kind == HID_EXT_CONSUMER ? "consumer" : "system",
                     sub->val_handle, len, n, n > 0 ? usages[0] : 0);
        }
        if (s_ext_cb) {
            s_ext_cb(sub->ext->kind == HID_EXT_SYSTEM, usages, n);
        }
        return;
    }

    uint8_t modifiers = 0;
    uint8_t keys[KEY_STATE_MAX_KEYS];
    int n;

    if (sub->layout == NULL) {
        // Boot keyboard input report is already in boot format
        modifiers = len > 0 ? raw[0] : 0;
        n = 0;
        for (uint16_t i = 2; i < len && i < 8 && n < (int)sizeof(keys); i++) {
            if (raw[i] != 0) {
                keys[n++] = raw[i];
            }
        }
    } else {
        n = hid_parser_extract_keys(sub->layout, raw, len, &modifiers,
                                    keys, sizeof(keys));
        if (n < 0) {
            return;
        }
    }

    if (s_input_log_budget > 0) {
        s_input_log_budget--;
        ESP_LOGI(TAG, "IN handle=%d len=%d mod=%02X keys=%d [%02X %02X %02X %02X %02X %02X]",
                 sub->val_handle, len, modifiers, n,
                 n > 0 ? keys[0] : 0, n > 1 ? keys[1] : 0,
                 n > 2 ? keys[2] : 0, n > 3 ? keys[3] : 0,
                 n > 4 ? keys[4] : 0, n > 5 ? keys[5] : 0);
    }

    if (s_keyboard_cb) {
        // Single connection for now, so every source is device 0
        s_keyboard_cb(0, modifiers, keys, n);
    }
}

/* ============================================================================
 * GAP event handler (runs on NimBLE host task)
 * ============================================================================ */

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        const struct ble_gap_disc_desc *disc = &event->disc;

        if (s_pairing_mode) {
            char name[32] = {0};
            if (adv_is_hid_keyboard(disc, name, sizeof(name)) &&
                disc->rssi >= BLE_PAIRING_RSSI_MIN) {
                bool same = s_best_set && ble_addr_cmp(&s_best_addr, &disc->addr) == 0;

                if (same) {
                    if (disc->rssi > s_best_rssi) {
                        s_best_rssi = disc->rssi;
                    }
                    if (name[0] != '\0' && s_best_name[0] == '\0') {
                        strncpy(s_best_name, name, sizeof(s_best_name) - 1);
                    }
                } else if (!s_best_set || disc->rssi > s_best_rssi) {
                    s_best_addr = disc->addr;
                    s_best_rssi = disc->rssi;
                    memset(s_best_name, 0, sizeof(s_best_name));
                    strncpy(s_best_name, name, sizeof(s_best_name) - 1);
                    s_best_set = true;

                    char addr_str[18];
                    storage_addr_to_str(disc->addr.val, addr_str);
                    ESP_LOGI(TAG, "Candidate: %s rssi=%d '%s'",
                             addr_str, disc->rssi, name);
                }
            }
        } else if (s_has_target) {
            // Reconnect scan: match the bonded keyboard's identity address
            if (ble_addr_cmp(&disc->addr, &s_target_addr) == 0) {
                ESP_LOGI(TAG, "Bonded keyboard is advertising, connecting");
                connect_to(&s_target_addr, 10000);
            }
        }
        return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGD(TAG, "Scan complete (reason=%d)", event->disc_complete.reason);
        xEventGroupSetBits(s_events, EVT_SCAN_DONE);
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_enc_retried = false;
            ESP_LOGI(TAG, "Connected (handle=%d), initiating security",
                     s_conn_handle);

            ble_gattc_exchange_mtu(s_conn_handle, mtu_cb, NULL);

            int rc = ble_gap_security_initiate(s_conn_handle);
            if (rc != 0 && rc != BLE_HS_EALREADY) {
                ESP_LOGW(TAG, "security_initiate failed: %d", rc);
                fail_connection();
            }
        } else {
            ESP_LOGD(TAG, "Connect failed: %d", event->connect.status);
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            xEventGroupSetBits(s_events, EVT_FAILED);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected (reason=%d)", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        reset_disc_ctx();
        if (!s_pairing_mode) {
            set_state(s_has_target ? BLE_HID_STATE_SCANNING : BLE_HID_STATE_IDLE);
        }
        xEventGroupSetBits(s_events, EVT_FAILED);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, "Link encrypted, starting GATT discovery");
            start_discovery();
        } else {
            ESP_LOGW(TAG, "Encryption failed: %d", event->enc_change.status);
            // Typical cause: the keyboard dropped our bond (e.g. its pairing
            // slot was reused on another host). Delete ours and retry once,
            // which triggers fresh pairing.
            if (!s_enc_retried) {
                s_enc_retried = true;
                struct ble_gap_conn_desc desc;
                if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
                    ble_store_util_delete_peer(&desc.peer_id_addr);
                }
                int rc = ble_gap_security_initiate(event->enc_change.conn_handle);
                if (rc != 0) {
                    fail_connection();
                }
            } else {
                fail_connection();
            }
        }
        return 0;

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        struct ble_sm_io io;
        memset(&io, 0, sizeof(io));
        io.action = event->passkey.params.action;

        if (io.action == BLE_SM_IOACT_DISP) {
            io.passkey = BLE_STATIC_PASSKEY;
            ESP_LOGI(TAG, "==> Type passkey %06d on the keyboard and press Enter",
                     (int)io.passkey);
            ble_sm_inject_io(event->passkey.conn_handle, &io);
        } else if (io.action == BLE_SM_IOACT_NUMCMP) {
            io.numcmp_accept = 1;
            ble_sm_inject_io(event->passkey.conn_handle, &io);
        } else {
            ESP_LOGW(TAG, "Unsupported passkey action: %d", io.action);
        }
        return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        // Peer wants to re-pair although we already have a bond: drop the old
        // bond and accept. This is the multi-device keyboard "come back from
        // another host" path.
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint16_t h = event->notify_rx.attr_handle;
        for (int i = 0; i < s_num_subs; i++) {
            if (s_subs[i].val_handle == h) {
                handle_input_notification(&s_subs[i], event->notify_rx.om);
                return 0;
            }
        }
        // Not a keyboard report: deliberately ignored (this is what prevents
        // media/vendor reports from being injected as keystrokes)
        ESP_LOGD(TAG, "Ignored notification on handle %d", h);
        return 0;
    }

    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
        // Accept whatever the keyboard asks for (it knows its power budget)
        if (event->conn_update_req.self_params != NULL &&
            event->conn_update_req.peer_params != NULL) {
            *event->conn_update_req.self_params = *event->conn_update_req.peer_params;
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGD(TAG, "MTU: %d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

/* ============================================================================
 * NimBLE host task plumbing
 * ============================================================================ */

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset, reason=%d", reason);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) {
        rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to configure own address: %d", rc);
    }
    xSemaphoreGive(s_sync_sem);
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ============================================================================
 * Control task: reconnect cycles & pairing
 * ============================================================================ */

static void save_current_peer_as_target(void)
{
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(s_conn_handle, &desc) != 0) {
        return;
    }

    s_target_addr = desc.peer_id_addr;
    s_has_target = true;

    paired_device_t dev = {0};
    memcpy(dev.addr, desc.peer_id_addr.val, 6);
    dev.addr_type = desc.peer_id_addr.type;
    dev.valid = true;
    strncpy(dev.name, s_best_name[0] ? s_best_name : s_target_name,
            sizeof(dev.name) - 1);
    strncpy(s_target_name, dev.name, sizeof(s_target_name) - 1);
    storage_save_paired_device(&dev);
}

static void run_pairing(void)
{
    ESP_LOGI(TAG, "Entering pairing mode");

    // Drop any current connection first
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        xEventGroupClearBits(s_events, EVT_FAILED);
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        xEventGroupWaitBits(s_events, EVT_FAILED, pdTRUE, pdFALSE,
                            pdMS_TO_TICKS(2000));
    }
    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
    }

    s_best_set = false;
    s_best_rssi = -128;
    memset(s_best_name, 0, sizeof(s_best_name));
    s_pairing_mode = true;
    set_state(BLE_HID_STATE_PAIRING);

    xEventGroupClearBits(s_events, EVT_SCAN_DONE | EVT_READY | EVT_FAILED);

    if (start_scan(true, BLE_PAIRING_SCAN_SEC * 1000) != 0) {
        s_pairing_mode = false;
        set_state(BLE_HID_STATE_ERROR);
        return;
    }

    // Collect candidates for the full scan window, then pick the best
    xEventGroupWaitBits(s_events, EVT_SCAN_DONE, pdTRUE, pdFALSE,
                        pdMS_TO_TICKS((BLE_PAIRING_SCAN_SEC + 2) * 1000));

    if (!s_best_set) {
        ESP_LOGW(TAG, "Pairing: no keyboard found");
        s_pairing_mode = false;
        set_state(s_has_target ? BLE_HID_STATE_SCANNING : BLE_HID_STATE_IDLE);
        return;
    }

    char addr_str[18];
    storage_addr_to_str(s_best_addr.val, addr_str);
    ESP_LOGI(TAG, "Pairing with %s '%s' (rssi=%d)", addr_str, s_best_name, s_best_rssi);

    // Single-keyboard device: forget all previous bonds
    ble_store_clear();
    storage_clear_paired_device();
    s_has_target = false;

    xEventGroupClearBits(s_events, EVT_READY | EVT_FAILED);
    connect_to(&s_best_addr, 15000);

    EventBits_t bits = xEventGroupWaitBits(s_events, EVT_READY | EVT_FAILED,
                                           pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(40000));
    if (bits & EVT_READY) {
        save_current_peer_as_target();
        ESP_LOGI(TAG, "Pairing complete");
    } else {
        ESP_LOGW(TAG, "Pairing failed");
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        set_state(BLE_HID_STATE_ERROR);
        vTaskDelay(pdMS_TO_TICKS(1000));
        set_state(BLE_HID_STATE_IDLE);
    }

    s_pairing_mode = false;
}

static void run_reconnect_cycle(int attempt)
{
    xEventGroupClearBits(s_events, EVT_READY | EVT_FAILED | EVT_SCAN_DONE);

    // Alternate: mostly scan for the keyboard's advertisements (cheap, catches
    // wake-up), but periodically try a direct connection, which also covers
    // bonded keyboards using resolvable private addresses.
    bool direct = ((attempt % 4) == 3);

    if (direct) {
        set_state(BLE_HID_STATE_CONNECTING);
        connect_to(&s_target_addr, BLE_DIRECT_CONNECT_TIMEOUT_MS);
        xEventGroupWaitBits(s_events, EVT_READY | EVT_FAILED, pdTRUE, pdFALSE,
                            pdMS_TO_TICKS(BLE_DIRECT_CONNECT_TIMEOUT_MS + 30000));
        return;
    }

    set_state(BLE_HID_STATE_SCANNING);
    if (start_scan(false, BLE_RECONNECT_SCAN_SEC * 1000) != 0) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    // Wait for: match found -> connection chain -> READY/FAILED,
    // or scan window elapsed with no match.
    EventBits_t bits;
    for (;;) {
        bits = xEventGroupWaitBits(s_events,
                                   EVT_READY | EVT_FAILED | EVT_SCAN_DONE,
                                   pdTRUE, pdFALSE,
                                   pdMS_TO_TICKS((BLE_RECONNECT_SCAN_SEC + 45) * 1000));
        if (bits & (EVT_READY | EVT_FAILED)) {
            return;
        }
        if (bits & EVT_SCAN_DONE) {
            // If the scan ended because we found the target and started
            // connecting, keep waiting for the connection result.
            if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE ||
                s_state == BLE_HID_STATE_CONNECTING) {
                continue;
            }
            return;
        }
        // Timeout safety net
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return;
    }
}

static void ctrl_task(void *param)
{
    // Wait for the NimBLE host to sync
    xSemaphoreTake(s_sync_sem, portMAX_DELAY);
    ESP_LOGI(TAG, "BLE host synced");

    // Load bonded keyboard from NVS
    paired_device_t dev;
    if (storage_load_paired_device(&dev) == ESP_OK) {
        memcpy(s_target_addr.val, dev.addr, 6);
        s_target_addr.type = dev.addr_type;
        strncpy(s_target_name, dev.name, sizeof(s_target_name) - 1);
        s_has_target = true;
    }

    set_state(s_has_target ? BLE_HID_STATE_SCANNING : BLE_HID_STATE_IDLE);

    int attempt = 0;

    for (;;) {
        uint8_t cmd = 0;
        if (xQueueReceive(s_ctrl_queue, &cmd, pdMS_TO_TICKS(BLE_RECONNECT_DELAY_MS)) == pdTRUE) {
            if (cmd == CTRL_CMD_PAIR) {
                run_pairing();
                attempt = 0;
                continue;
            }
        }

        if (s_state == BLE_HID_STATE_CONNECTED || s_pairing_mode) {
            attempt = 0;
            continue;
        }

        if (s_has_target) {
            run_reconnect_cycle(attempt++);
        }
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

void ble_hid_host_set_ext_cb(ble_hid_ext_cb_t cb)
{
    s_ext_cb = cb;
}

esp_err_t ble_hid_host_init(ble_hid_keyboard_cb_t keyboard_cb, ble_hid_state_cb_t state_cb)
{
    if (s_initialized) {
        return ESP_OK;
    }
    if (keyboard_cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_keyboard_cb = keyboard_cb;
    s_state_cb = state_cb;

    s_sync_sem = xSemaphoreCreateBinary();
    s_events = xEventGroupCreate();
    s_ctrl_queue = xQueueCreate(4, sizeof(uint8_t));
    if (s_sync_sem == NULL || s_events == NULL || s_ctrl_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Host configuration: bondable central with a displayable static passkey
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_DISP_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_device_name_set("BT_HID_Proxy");
    ble_store_config_init();

    nimble_port_freertos_init(nimble_host_task);

    BaseType_t rc = xTaskCreate(ctrl_task, "ble_ctrl", CTRL_TASK_STACK, NULL,
                                CTRL_TASK_PRIO, &s_ctrl_task_handle);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task");
        return ESP_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "BLE HID host initialized");
    return ESP_OK;
}

esp_err_t ble_hid_host_start_pairing(void)
{
    if (!s_initialized || s_ctrl_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t cmd = CTRL_CMD_PAIR;
    if (xQueueSend(s_ctrl_queue, &cmd, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t ble_hid_host_send_led_status(uint8_t led_status)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t conn = s_conn_handle;
    uint16_t handle = s_led_output_val_handle;

    if (conn == BLE_HS_CONN_HANDLE_NONE || handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    int rc = ble_gattc_write_no_rsp_flat(conn, handle, &led_status, 1);
    if (rc != 0) {
        ESP_LOGW(TAG, "LED write failed: %d", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

ble_hid_state_t ble_hid_host_get_state(void)
{
    return s_state;
}

bool ble_hid_host_is_connected(void)
{
    return s_state == BLE_HID_STATE_CONNECTED;
}
