/**
 * @file diag.c
 * @brief Persistent diagnostics ring buffer in the `storage` flash partition.
 *
 * Record layout is fixed size so the ring needs no index and no filesystem:
 * erased flash reads as 0xFF, so a record whose sequence number is 0xFFFFFFFF
 * is free. On boot the whole area is scanned for the highest sequence number,
 * which identifies both where writing resumes and where the oldest entry is.
 *
 * The `storage` partition was declared in partitions.csv from the start but
 * never used by anything, so this costs no space that was in use. It is also
 * deliberately NOT the `nvs` partition: bond churn in NVS is one of the
 * suspects being investigated, and a diagnostic that competes for the same 24
 * kB would change the very behaviour it is meant to observe.
 */

#include "diag.h"
#include "config.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "DIAG";

diag_counters_t g_diag = {0};

#define DIAG_TEXT_LEN       56
#define DIAG_REC_SIZE       64      // must divide the 4 kB flash sector evenly
#define DIAG_SEQ_EMPTY      0xFFFFFFFFu
#define DIAG_SECTOR_SIZE    4096
#define DIAG_RECS_PER_SECTOR (DIAG_SECTOR_SIZE / DIAG_REC_SIZE)

typedef struct {
    uint32_t seq;                   // 0xFFFFFFFF = erased/free
    uint32_t uptime_s;              // seconds since boot when the entry was made
    char     text[DIAG_TEXT_LEN];   // NUL-terminated, truncated if longer
} diag_record_t;

_Static_assert(sizeof(diag_record_t) == DIAG_REC_SIZE, "diag record must be 64 bytes");

static const esp_partition_t *s_part = NULL;
static uint32_t s_total_recs = 0;
static uint32_t s_next_idx = 0;      // where the next record goes
static uint32_t s_next_seq = 1;
static QueueHandle_t s_queue = NULL;

/* ============================================================================
 * Ring buffer primitives
 * ============================================================================ */

static bool read_record(uint32_t idx, diag_record_t *out)
{
    return esp_partition_read(s_part, (size_t)idx * DIAG_REC_SIZE,
                              out, sizeof(*out)) == ESP_OK;
}

/**
 * @brief Append one record, erasing the next sector when the write crosses
 *        into it. Runs only on the writer task.
 */
static void write_record(const diag_record_t *rec)
{
    size_t offset = (size_t)s_next_idx * DIAG_REC_SIZE;

    // Flash must be erased before it can be written, and the smallest unit is
    // a sector. Landing on a sector boundary means the oldest 64 entries are
    // about to be recycled.
    if (s_next_idx % DIAG_RECS_PER_SECTOR == 0) {
        esp_err_t err = esp_partition_erase_range(s_part, offset, DIAG_SECTOR_SIZE);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "erase at 0x%x failed: %s", (unsigned)offset,
                     esp_err_to_name(err));
            return;
        }
    }

    if (esp_partition_write(s_part, offset, rec, sizeof(*rec)) != ESP_OK) {
        ESP_LOGW(TAG, "write at 0x%x failed", (unsigned)offset);
        return;
    }

    s_next_idx = (s_next_idx + 1) % s_total_recs;
    s_next_seq++;
}

/**
 * @brief Locate the write position by finding the highest sequence number.
 */
static void scan_ring(void)
{
    uint32_t best_seq = 0;
    uint32_t best_idx = 0;
    bool found = false;

    for (uint32_t i = 0; i < s_total_recs; i++) {
        diag_record_t rec;
        if (!read_record(i, &rec) || rec.seq == DIAG_SEQ_EMPTY) {
            continue;
        }
        if (!found || rec.seq > best_seq) {
            best_seq = rec.seq;
            best_idx = i;
            found = true;
        }
    }

    if (found) {
        s_next_idx = (best_idx + 1) % s_total_recs;
        s_next_seq = best_seq + 1;
    } else {
        s_next_idx = 0;
        s_next_seq = 1;
    }
}

/* ============================================================================
 * Writer task
 * ============================================================================ */

static void diag_writer_task(void *arg)
{
    diag_record_t rec;
    for (;;) {
        if (xQueueReceive(s_queue, &rec, portMAX_DELAY) == pdTRUE) {
            rec.seq = s_next_seq;
            write_record(&rec);
        }
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

uint32_t diag_uptime_s(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);
}

void diag_format_uptime(uint32_t seconds, char *out, size_t out_len)
{
    uint32_t d = seconds / 86400;
    uint32_t h = (seconds % 86400) / 3600;
    uint32_t m = (seconds % 3600) / 60;
    uint32_t s = seconds % 60;

    if (d > 0) {
        snprintf(out, out_len, "%ud%02uh%02um%02us",
                 (unsigned)d, (unsigned)h, (unsigned)m, (unsigned)s);
    } else {
        snprintf(out, out_len, "%02uh%02um%02us",
                 (unsigned)h, (unsigned)m, (unsigned)s);
    }
}

void diag_event(const char *fmt, ...)
{
    diag_record_t rec;
    rec.seq = 0;    // filled in by the writer task
    rec.uptime_s = diag_uptime_s();

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(rec.text, sizeof(rec.text), fmt, ap);
    va_end(ap);

    char up[16];
    diag_format_uptime(rec.uptime_s, up, sizeof(up));
    ESP_LOGI(TAG, "[%s] %s", up, rec.text);

    if (s_queue == NULL) {
        return;
    }
    // Never wait: dropping a diagnostic line is always better than stalling a
    // BLE callback behind a flash erase.
    if (xQueueSend(s_queue, &rec, 0) != pdTRUE) {
        ESP_LOGW(TAG, "diag queue full, entry not persisted");
    }
}

void diag_dump(void)
{
    if (s_part == NULL) {
        ESP_LOGW(TAG, "No persistent log (storage partition unavailable)");
        return;
    }

    // Oldest entry is the one right after the newest, i.e. the write position.
    uint32_t printed = 0;
    ESP_LOGI(TAG, "---------- persistent log (oldest first) ----------");

    for (uint32_t n = 0; n < s_total_recs; n++) {
        uint32_t idx = (s_next_idx + n) % s_total_recs;
        diag_record_t rec;
        if (!read_record(idx, &rec) || rec.seq == DIAG_SEQ_EMPTY) {
            continue;
        }
        rec.text[DIAG_TEXT_LEN - 1] = '\0';

        char up[16];
        diag_format_uptime(rec.uptime_s, up, sizeof(up));
        ESP_LOGI(TAG, "#%-6u [%s] %s", (unsigned)rec.seq, up, rec.text);
        printed++;
    }

    ESP_LOGI(TAG, "---------- end of log (%u entries, capacity %u) ----------",
             (unsigned)printed, (unsigned)s_total_recs);
}

/**
 * @brief Human-readable reset cause. This alone distinguishes a clean
 *        unplug/replug from a brownout, a panic, or a watchdog reset - three
 *        very different explanations for the same observed "it came back".
 */
static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external pin";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "PANIC/exception";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "other watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "unknown";
    }
}

esp_err_t diag_init(void)
{
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      ESP_PARTITION_SUBTYPE_ANY, "storage");
    if (s_part == NULL) {
        ESP_LOGW(TAG, "'storage' partition not found - console logging only");
    } else {
        s_total_recs = s_part->size / DIAG_REC_SIZE;
        scan_ring();
        ESP_LOGI(TAG, "Persistent log: %u entries capacity, resuming at #%u",
                 (unsigned)s_total_recs, (unsigned)s_next_seq);

        // Everything the previous run recorded, before this run overwrites any
        // of it. This is the whole point of the module.
        diag_dump();

        s_queue = xQueueCreate(16, sizeof(diag_record_t));
        if (s_queue == NULL) {
            ESP_LOGE(TAG, "queue alloc failed - persistence disabled");
        } else if (xTaskCreate(diag_writer_task, "diag_wr", 3072, NULL, 1,
                               NULL) != pdPASS) {
            ESP_LOGE(TAG, "writer task failed - persistence disabled");
            vQueueDelete(s_queue);
            s_queue = NULL;
        }
    }

    esp_reset_reason_t reason = esp_reset_reason();
    diag_event("=== BOOT === reset=%s heap=%u", reset_reason_str(reason),
               (unsigned)esp_get_free_heap_size());

    return ESP_OK;
}
