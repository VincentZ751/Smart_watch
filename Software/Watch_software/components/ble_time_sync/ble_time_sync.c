#include "ble_time_sync.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_WATCH";
static const char *DEVICE_NAME = "SmartWatch";

// Client UUIDs (Current Time Service on iPhone)
#define CTS_SVC_UUID          0x1805
#define CTS_CHR_UUID          0x2A2B

// Server UUIDs (HID + Device Info)
#define DIS_SVC_UUID          0x180A
#define DIS_PNP_ID_UUID       0x2A50

#define HID_SVC_UUID          0x1812
#define HID_INFO_UUID         0x2A4A
#define HID_REPORT_MAP_UUID   0x2A4B
#define HID_CTRL_PT_UUID      0x2A4C
#define HID_REPORT_UUID       0x2A4D
#define HID_REPORT_REF_UUID   0x2908

void ble_store_config_init(void);

static uint8_t own_addr_type = 0;
static bool nimble_synced = false;
static bool sync_active = false;
static bool keep_alive_hid = false;
static uint16_t active_conn_handle = BLE_HS_CONN_HANDLE_NONE;

static uint16_t cts_svc_start = 0;
static uint16_t cts_svc_end   = 0;
static uint16_t cts_chr_val_handle = 0;

static uint16_t hid_media_val_handle = 0; // Report ID 1 (Consumer)
static uint16_t hid_call_val_handle  = 0; // Report ID 2 (Telephony)

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);

// ============================================================================
// BLE HID REPORT MAP: Report 1 = Media, Report 2 = Phone Calls
// ============================================================================
static const uint8_t hid_report_map[] = {
    // --- Report ID 1: Consumer Control (Media) ---
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x05,        //   Report Count (5)
    0x09, 0xCD,        //   Bit 0: Play/Pause
    0x09, 0xB5,        //   Bit 1: Scan Next Track
    0x09, 0xB6,        //   Bit 2: Scan Previous Track
    0x09, 0xE9,        //   Bit 3: Volume Increment
    0x09, 0xEA,        //   Bit 4: Volume Decrement
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x75, 0x03,        //   Report Size (3)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x03,        //   Input (Const,Var,Abs) - padding
    0xC0,              // End Collection

    // --- Report ID 2: Telephony (Calls) ---
    0x05, 0x0B,        // Usage Page (Telephony)
    0x09, 0x05,        // Usage (Headset)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x02,        //   Report Count (2)
    0x09, 0x20,        //   Bit 0: Hook Switch (Answer / End Call)
    0x09, 0x2F,        //   Bit 1: Phone Mute
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x75, 0x06,        //   Report Size (6)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x03,        //   Input (Const,Var,Abs) - padding
    0xC0               // End Collection
};

static const uint8_t hid_info_val[]        = { 0x11, 0x01, 0x00, 0x02 };
static const uint8_t hid_media_ref[]       = { 0x01, 0x01 }; // Report ID 1, Input
static const uint8_t hid_call_ref[]        = { 0x02, 0x01 }; // Report ID 2, Input
static const uint8_t dis_pnp_id[]          = { 0x02, 0xE5, 0x02, 0x01, 0x00, 0x01, 0x00 };

static int gatt_hid_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid = ble_uuid_u16(ctxt->chr ? ctxt->chr->uuid : ctxt->dsc->uuid);

    switch (uuid) {
        case HID_INFO_UUID:
            return os_mbuf_append(ctxt->om, hid_info_val, sizeof(hid_info_val)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case HID_REPORT_MAP_UUID:
            return os_mbuf_append(ctxt->om, hid_report_map, sizeof(hid_report_map)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case HID_REPORT_UUID: {
            uint8_t zero = 0;
            return os_mbuf_append(ctxt->om, &zero, sizeof(zero)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        case HID_REPORT_REF_UUID: {
            const uint8_t *ref = ((uintptr_t)arg == 2) ? hid_call_ref : hid_media_ref;
            return os_mbuf_append(ctxt->om, ref, 2) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        case DIS_PNP_ID_UUID:
            return os_mbuf_append(ctxt->om, dis_pnp_id, sizeof(dis_pnp_id)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case HID_CTRL_PT_UUID:
            return 0;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(DIS_SVC_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(DIS_PNP_ID_UUID),
                .access_cb = gatt_hid_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 }
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(HID_SVC_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(HID_INFO_UUID),
                .access_cb = gatt_hid_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = BLE_UUID16_DECLARE(HID_REPORT_MAP_UUID),
                .access_cb = gatt_hid_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = BLE_UUID16_DECLARE(HID_CTRL_PT_UUID),
                .access_cb = gatt_hid_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                // Report ID 1: Media Control
                .uuid = BLE_UUID16_DECLARE(HID_REPORT_UUID),
                .access_cb = gatt_hid_access_cb,
                .val_handle = &hid_media_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(HID_REPORT_REF_UUID),
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = gatt_hid_access_cb,
                        .arg = (void *)1,
                    },
                    { 0 }
                },
            },
            {
                // Report ID 2: Call Control
                .uuid = BLE_UUID16_DECLARE(HID_REPORT_UUID),
                .access_cb = gatt_hid_access_cb,
                .val_handle = &hid_call_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(HID_REPORT_REF_UUID),
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = gatt_hid_access_cb,
                        .arg = (void *)2,
                    },
                    { 0 }
                },
            },
            { 0 }
        },
    },
    { 0 }
};

// ============================================================================
// ADVERTISING & CTS CLIENT
// ============================================================================
static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;

    fields.appearance = 0x03C0;
    fields.appearance_is_present = 1;
    fields.uuids16 = (ble_uuid16_t[]){ BLE_UUID16_INIT(HID_SVC_UUID) };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting adv fields: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Error starting adv: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as '%s' (HID)...", DEVICE_NAME);
    }
}

static int cts_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       struct ble_gatt_attr *attr, void *arg)
{
    if (error->status != 0 || attr == NULL) {
        ESP_LOGE(TAG, "CTS read failed (status=%d)", error->status);
        return 0;
    }

    uint8_t data[10] = {0};
    if (OS_MBUF_PKTLEN(attr->om) < 7) {
        return 0;
    }
    os_mbuf_copydata(attr->om, 0, sizeof(data), data);

    struct tm t = {
        .tm_year  = (data[0] | (data[1] << 8)) - 1900,
        .tm_mon   = data[2] - 1,
        .tm_mday  = data[3],
        .tm_hour  = data[4],
        .tm_min   = data[5],
        .tm_sec   = data[6],
        .tm_isdst = -1
    };

    struct timeval tv = { .tv_sec = mktime(&t), .tv_usec = 0 };
    settimeofday(&tv, NULL);

    ESP_LOGI(TAG, ">>> TIME SYNCED: %04u-%02u-%02u %02u:%02u:%02u <<<",
             (data[0] | (data[1] << 8)), data[2], data[3], data[4], data[5], data[6]);

    // Only disconnect if we're in one-shot Time Sync mode
    if (!keep_alive_hid) {
        sync_active = false;
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
}

static int cts_chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == 0 && chr != NULL) {
        cts_chr_val_handle = chr->val_handle;
    } else if (error->status == BLE_HS_EDONE && cts_chr_val_handle != 0) {
        ble_gattc_read(conn_handle, cts_chr_val_handle, cts_read_cb, NULL);
    }
    return 0;
}

static int cts_svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_svc *svc, void *arg)
{
    if (error->status == 0 && svc != NULL) {
        cts_svc_start = svc->start_handle;
        cts_svc_end   = svc->end_handle;
    } else if (error->status == BLE_HS_EDONE && cts_svc_start != 0) {
        cts_chr_val_handle = 0;
        ble_gattc_disc_chrs_by_uuid(conn_handle, cts_svc_start, cts_svc_end,
                                    BLE_UUID16_DECLARE(CTS_CHR_UUID),
                                    cts_chr_disc_cb, NULL);
    }
    return 0;
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                active_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "iPhone connected!");
                ble_gap_security_initiate(active_conn_handle);
            } else if (sync_active || keep_alive_hid) {
                start_advertising();
            }
            break;

        case BLE_GAP_EVENT_ENC_CHANGE:
            if (event->enc_change.status == 0) {
                ESP_LOGI(TAG, "Link encrypted! Ready for HID & CTS.");
                cts_svc_start = 0;
                cts_svc_end   = 0;
                ble_gattc_disc_svc_by_uuid(event->enc_change.conn_handle,
                                           BLE_UUID16_DECLARE(CTS_SVC_UUID),
                                           cts_svc_disc_cb, NULL);
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            active_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGI(TAG, "iPhone disconnected.");
            if (sync_active || keep_alive_hid) {
                start_advertising();
            }
            break;

        case BLE_GAP_EVENT_REPEAT_PAIRING: {
            struct ble_gap_conn_desc desc;
            ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
            ble_store_util_delete_peer(&desc.peer_id_addr);
            return BLE_GAP_REPEAT_PAIRING_RETRY;
        }

        default:
            break;
    }
    return 0;
}

static void ble_on_sync(void)
{
    ble_hs_id_infer_auto(0, &own_addr_type);
    nimble_synced = true;
    if (sync_active || keep_alive_hid) {
        start_advertising();
    }
}

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ============================================================================
// PUBLIC API
// ============================================================================
esp_err_t ble_time_sync_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svr_svcs) == 0 ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svr_svcs) == 0 ? ESP_OK : ESP_FAIL);

    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_store_config_init();

    nimble_port_freertos_init(ble_host_task);
    return ESP_OK;
}

esp_err_t ble_time_sync_start(void)
{
    keep_alive_hid = false;
    sync_active = true;
    if (nimble_synced && active_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        start_advertising();
    }
    return ESP_OK;
}

void ble_time_sync_stop(void)
{
    sync_active = false;
    if (!keep_alive_hid) {
        ble_gap_adv_stop();
        if (active_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(active_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
}

bool ble_time_sync_is_active(void)
{
    return sync_active;
}

esp_err_t ble_hid_mode_start(void)
{
    keep_alive_hid = true;
    if (nimble_synced && active_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        start_advertising();
    }
    return ESP_OK;
}

void ble_hid_mode_stop(void)
{
    keep_alive_hid = false;
    sync_active = false;
    ble_gap_adv_stop();
    if (active_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(active_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

bool ble_hid_is_connected(void)
{
    return (active_conn_handle != BLE_HS_CONN_HANDLE_NONE);
}

esp_err_t ble_hid_send_key(ble_hid_key_t key)
{
    if (active_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "HID key ignored — iPhone not connected yet.");
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t handle = hid_media_val_handle;
    uint8_t mask = 0;

    switch (key) {
        case BLE_HID_PLAY_PAUSE:      mask = (1 << 0); break;
        case BLE_HID_NEXT_TRACK:      mask = (1 << 1); break;
        case BLE_HID_PREV_TRACK:      mask = (1 << 2); break;
        case BLE_HID_VOL_UP:          mask = (1 << 3); break;
        case BLE_HID_VOL_DOWN:        mask = (1 << 4); break;
        case BLE_HID_CALL_ANSWER_END: handle = hid_call_val_handle; mask = (1 << 0); break;
        case BLE_HID_CALL_MUTE:       handle = hid_call_val_handle; mask = (1 << 1); break;
        default: return ESP_ERR_INVALID_ARG;
    }

    // Press
    struct os_mbuf *om_press = ble_hs_mbuf_from_flat(&mask, 1);
    if (!om_press) return ESP_ERR_NO_MEM;
    int rc = ble_gatts_notify_custom(active_conn_handle, handle, om_press);
    if (rc != 0) return ESP_FAIL;

    vTaskDelay(pdMS_TO_TICKS(30));

    // Release
    uint8_t release = 0;
    struct os_mbuf *om_rel = ble_hs_mbuf_from_flat(&release, 1);
    if (!om_rel) return ESP_ERR_NO_MEM;
    rc = ble_gatts_notify_custom(active_conn_handle, handle, om_rel);

    return (rc == 0) ? ESP_OK : ESP_FAIL;
}