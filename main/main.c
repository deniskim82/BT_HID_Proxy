/**
 * @file main.c
 * @brief BT HID Proxy - BLE Keyboard to USB HID Bridge
 *
 * [BLE Keyboard] --(BLE/HOGP)--> [ESP32/NimBLE] --(UART)--> [CH9329] --(USB)--> [PC]
 *
 * Hardware: M5Stamp Pico (ESP32-PICO-D4)
 *  - CH9329 on UART1: TX=GPIO32, RX=GPIO33, 115200 baud
 *  - Button: GPIO39 (long-press 5s = pairing mode)
 *  - RGB LED: GPIO27 (SK6812)
 *
 * Hot path: BLE notification -> boot report translation -> CH9329 TX queue.
 * No blocking calls anywhere on that path.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "config.h"
#include "diag.h"
#include "storage.h"
#include "ch9329.h"
#include "ble_hid_host.h"
#include "key_state.h"
#include "button.h"
#include "led_status.h"

static const char *TAG = "MAIN";

/* USB HID key codes for LED-related keys */
#define HID_KEY_CAPS_LOCK    0x39
#define HID_KEY_SCROLL_LOCK  0x47
#define HID_KEY_NUM_LOCK     0x53

// One-shot timer used to poll the PC's LED state shortly after a lock key
// press (or after connecting), without ever blocking the input path.
static esp_timer_handle_t s_led_poll_timer = NULL;

static uint8_t s_prev_keys[KEY_STATE_MAX_KEYS] = {0};
static int s_prev_key_count = 0;

// Set from the button callback, serviced by the main task (see on_button_event)
static volatile bool s_dump_requested = false;

/* ============================================================================
 * LED state polling
 * ============================================================================ */

static void led_poll_timer_cb(void *arg)
{
    ch9329_request_led_status();
}

static void schedule_led_poll(uint32_t delay_ms)
{
    if (s_led_poll_timer == NULL) {
        return;
    }
    esp_timer_stop(s_led_poll_timer);
    esp_timer_start_once(s_led_poll_timer, (uint64_t)delay_ms * 1000);
}

static bool has_key(const uint8_t *keys, int count, uint8_t key_code)
{
    for (int i = 0; i < count; i++) {
        if (keys[i] == key_code) {
            return true;
        }
    }
    return false;
}

static bool lock_key_just_pressed(const uint8_t *curr, int curr_count,
                                  const uint8_t *prev, int prev_count)
{
    static const uint8_t lock_keys[] = {
        HID_KEY_CAPS_LOCK, HID_KEY_NUM_LOCK, HID_KEY_SCROLL_LOCK
    };

    for (size_t i = 0; i < sizeof(lock_keys); i++) {
        if (has_key(curr, curr_count, lock_keys[i]) &&
            !has_key(prev, prev_count, lock_keys[i])) {
            return true;
        }
    }
    return false;
}

/* ============================================================================
 * Callbacks
 * ============================================================================ */

/**
 * @brief Keyboard state from BLE. Runs on NimBLE host task - non-blocking.
 *
 * The device's keys are handed to the merge layer, which recomputes the whole
 * boot report. Sending the device's own report straight through would drop a
 * modifier held on a different keyboard once more than one is supported.
 */
static void on_keyboard_report(int source, uint8_t modifiers,
                               const uint8_t *keys, int count)
{
    key_state_source_update(source, modifiers, keys, count);

    uint8_t boot[8];
    bool changed = key_state_build_report(boot);
    if (changed) {
        ch9329_send_keyboard_report(boot);
    }

    // Lock keys are detected on the merged report, not on this one source:
    // with several reports feeding the merge, a single source's view is not
    // the state the host actually sees.
    if (changed &&
        lock_key_just_pressed(&boot[2], KEY_STATE_SLOTS,
                              s_prev_keys, s_prev_key_count)) {
        schedule_led_poll(LED_POLL_AFTER_KEY_MS);
    }

    if (changed) {
        s_prev_key_count = KEY_STATE_SLOTS;
        memcpy(s_prev_keys, &boot[2], KEY_STATE_SLOTS);
    }
}

/**
 * @brief PC LED state from CH9329 (Bit0=Num, Bit1=Caps, Bit2=Scroll).
 *        Runs on the CH9329 RX task; forward to the keyboard.
 */
static void on_pc_led_status(uint8_t led_status)
{
    ble_hid_host_send_led_status(led_status);
}

/**
 * @brief Media / system control keys from the keyboard.
 *        Runs on the NimBLE host task - non-blocking.
 */
static void on_ext_keys(bool is_system, const uint16_t *usages, int count)
{
    if (is_system) {
        ch9329_send_system_usages(usages, count);
    } else {
        ch9329_send_consumer_usages(usages, count);
    }
}

static void on_ble_state_change(ble_hid_state_t state)
{
    switch (state) {
    case BLE_HID_STATE_IDLE:
        led_status_set(LED_STATUS_IDLE);
        break;
    case BLE_HID_STATE_SCANNING:
        led_status_set(LED_STATUS_SCANNING);
        break;
    case BLE_HID_STATE_CONNECTING:
        led_status_set(LED_STATUS_CONNECTING);
        break;
    case BLE_HID_STATE_CONNECTED:
        led_status_set(LED_STATUS_CONNECTED);
        // Clean slate on the USB side, then sync LED state
        memset(s_prev_keys, 0, sizeof(s_prev_keys));
        s_prev_key_count = 0;
        key_state_reset();
        ch9329_release_all_keys();
        ch9329_send_consumer_usages(NULL, 0);
        ch9329_send_system_usages(NULL, 0);
        schedule_led_poll(LED_POLL_AFTER_CONNECT_MS);
        break;
    case BLE_HID_STATE_PAIRING:
        led_status_set(LED_STATUS_PAIRING);
        break;
    case BLE_HID_STATE_ERROR:
        led_status_set(LED_STATUS_ERROR);
        break;
    }

    if (state != BLE_HID_STATE_CONNECTED) {
        // Never leave keys held on the PC when the link is not up
        key_state_reset();
        ch9329_release_all_keys();
        ch9329_send_consumer_usages(NULL, 0);
        ch9329_send_system_usages(NULL, 0);
    }
}

/**
 * @brief Button events (runs in Timer Service task - only queue posts here)
 */
static void on_button_event(button_event_t event)
{
    if (event == BUTTON_EVENT_LONG_PRESS) {
        ESP_LOGI(TAG, "Long press: entering pairing mode");
        ch9329_release_all_keys();
        ble_hid_host_start_pairing();
    } else if (event == BUTTON_EVENT_SHORT_PRESS) {
        // Re-query the CH9329 so its USB enumeration state can be checked
        // on demand, e.g. after re-plugging the USB cable.
        ESP_LOGI(TAG, "Short press: querying CH9329 status");
        ch9329_request_info();
        ch9329_request_para_cfg();
        // Ask the main task to dump the persistent log, so the history can be
        // read at a console without waiting for (or forcing) a reboot.
        // Dumping here would hold the Timer Service task for seconds.
        s_dump_requested = true;
    }
}

/* ============================================================================
 * Health monitoring
 * ============================================================================ */

static const char *state_name(ble_hid_state_t s)
{
    switch (s) {
    case BLE_HID_STATE_IDLE:       return "IDLE";
    case BLE_HID_STATE_SCANNING:   return "SCAN";
    case BLE_HID_STATE_CONNECTING: return "CONN?";
    case BLE_HID_STATE_CONNECTED:  return "CONN";
    case BLE_HID_STATE_PAIRING:    return "PAIR";
    case BLE_HID_STATE_ERROR:      return "ERR";
    default:                       return "?";
    }
}

/**
 * @brief One line describing everything that drifts over a long run.
 *
 * The three resource figures matter most. Free heap falling steadily points
 * at a leak; min-free heap records the worst moment even if it recovered; and
 * free mbufs is where a NimBLE leak shows first, because that pool is fixed
 * at init and a link can look alive right up until it runs dry.
 */
static void health_tick(bool persist)
{
    char up[16];
    diag_format_uptime(diag_uptime_s(), up, sizeof(up));

    char line[160];
    snprintf(line, sizeof(line),
             "%s st=%s conn=%u/%u(r=0x%02x) enc!=%u bond-=%u rep=%u upd=%u "
             "in=%u led=%u drop=%u heap=%u/%u mbuf=%d",
             up, state_name(ble_hid_host_get_state()),
             (unsigned)g_diag.connects, (unsigned)g_diag.disconnects,
             (unsigned)g_diag.last_disc_reason,
             (unsigned)g_diag.enc_failures, (unsigned)g_diag.bond_deletes,
             (unsigned)g_diag.repeat_pairings, (unsigned)g_diag.conn_updates,
             (unsigned)g_diag.notifications, (unsigned)g_diag.led_writes,
             (unsigned)g_diag.tx_drops,
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size(),
             ble_hid_host_mbuf_free());

    ESP_LOGI(TAG, "HEALTH %s", line);

    if (persist) {
        // The ring record holds less than this line, so persist the figures
        // that only make sense as a trend and drop the ones that are also
        // reported as events when they change.
        diag_event("H %s h=%u/%u m=%d c=%u/%u",
                   state_name(ble_hid_host_get_state()),
                   (unsigned)esp_get_free_heap_size(),
                   (unsigned)esp_get_minimum_free_heap_size(),
                   ble_hid_host_mbuf_free(),
                   (unsigned)g_diag.connects, (unsigned)g_diag.disconnects);
    }
}

/**
 * @brief Record both ends of a spell where the link is up but nothing arrives.
 *
 * Silence is not a fault by itself - nobody types at night. It becomes
 * evidence only in combination, which is why the end of the spell is logged
 * too: silence that ends with input was just an idle keyboard, while silence
 * that ends only at a disconnect or a reboot is the failure being chased.
 */
static void check_silent_link(bool *silent)
{
    if (ble_hid_host_get_state() != BLE_HID_STATE_CONNECTED) {
        *silent = false;
        return;
    }

    uint32_t idle_ms = ble_hid_host_ms_since_input();

    if (!*silent && idle_ms >= DIAG_SILENT_LINK_MS) {
        *silent = true;
        g_diag.silent_episodes++;
        diag_event("SILENT link up, no input for %umin", (unsigned)(idle_ms / 60000));
    } else if (*silent && idle_ms < DIAG_SILENT_LINK_MS) {
        *silent = false;
        diag_event("SILENT ended, input resumed");
    }
}

/* ============================================================================
 * Initialization
 * ============================================================================ */

static esp_err_t init_all(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "BT HID Proxy (BLE/NimBLE) - Initializing");
    ESP_LOGI(TAG, "========================================");

    // First, so the previous run's log is dumped before anything overwrites
    // it and so the boot marker records why we restarted.
    diag_init();

    ret = storage_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Storage init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ch9329_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CH9329 init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ch9329_set_led_callback(on_pc_led_status);

    const esp_timer_create_args_t timer_args = {
        .callback = led_poll_timer_cb,
        .name = "led_poll",
    };
    ret = esp_timer_create(&timer_args, &s_led_poll_timer);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LED poll timer create failed (non-critical): %s",
                 esp_err_to_name(ret));
    }

    ret = led_status_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LED init failed (non-critical): %s", esp_err_to_name(ret));
    }

    ret = button_init(on_button_event);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Button init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // BLE last: reconnection starts immediately if a bonded keyboard exists
    ble_hid_host_set_ext_cb(on_ext_keys);
    ret = ble_hid_host_init(on_keyboard_report, on_ble_state_change);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE HID host init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Initialization complete");
    return ESP_OK;
}

void app_main(void)
{
    if (init_all() != ESP_OK) {
        ESP_LOGE(TAG, "Initialization failed, restarting in 5 seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }

    // Main task: health tick. Console every minute, persisted every fifth one
    // so the flash ring covers days rather than hours. The loop runs once a
    // second only so a dump request does not have to wait for the tick.
    int tick = 0;
    uint32_t elapsed_ms = 0;
    bool silent = false;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (s_dump_requested) {
            s_dump_requested = false;
            diag_dump();
        }

        elapsed_ms += 1000;
        if (elapsed_ms < DIAG_HEALTH_TICK_MS) {
            continue;
        }
        elapsed_ms = 0;
        tick++;

        health_tick(tick % DIAG_HEALTH_PERSIST_EVERY == 0);
        check_silent_link(&silent);

        // Refresh the CH9329's USB enumeration state. The reply is only
        // logged when it changes, so this is quiet unless something happens.
        ch9329_request_info();
    }
}
