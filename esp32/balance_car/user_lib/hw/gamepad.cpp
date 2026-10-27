#include "gamepad.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>

extern "C" void ble_store_config_init(void);

namespace gamepad
{
    namespace
    {
        constexpr uint8_t ADDRESS[] = {0x76, 0x99, 0x01, 0x58, 0x0b, 0xfc};
        constexpr const char *TAG = "gamepad";
        portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
        state latest{};
        SemaphoreHandle_t synced = nullptr;
        uint8_t own_address_type = 0;
        uint16_t connection = BLE_HS_CONN_HANDLE_NONE;
        bool scanning = false, connecting = false, discovering = false;
        uint16_t service_start = 0, service_end = 0;
        struct report { uint16_t value = 0, end = 0, ccc = 0; };
        report reports[8]{};
        uint8_t report_count = 0, report_index = 0;

        float axis(const uint8_t *data)
        {
            const uint16_t raw = data[0] | uint16_t(data[1]) << 8;
            const float value = (int32_t(raw) - 32768) / 32768.0f;
            constexpr float deadzone = 0.03f;
            return fabsf(value) <= deadzone ? 0.0f :
                copysignf(fminf(1.0f, (fabsf(value) - deadzone) / (1.0f - deadzone)), value);
        }

        void fail(int rc)
        {
            ESP_LOGE(TAG, "GATT setup failed rc=%d", rc);
            ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
        }

        void subscribe_next();
        int written(uint16_t, const ble_gatt_error *error, ble_gatt_attr *, void *)
        {
            if(error->status) { fail(error->status); return 0; }
            ++report_index;
            subscribe_next();
            return 0;
        }

        int descriptors(uint16_t, const ble_gatt_error *error, uint16_t, const ble_gatt_dsc *dsc, void *)
        {
            if(!error->status && ble_uuid_u16(&dsc->uuid.u) == 0x2902)
                reports[report_index].ccc = dsc->handle;
            else if(error->status == BLE_HS_EDONE)
            {
                if(!reports[report_index].ccc) { ++report_index; subscribe_next(); return 0; }
                const uint8_t enable[] = {1, 0};
                const int rc = ble_gattc_write_flat(connection, reports[report_index].ccc, enable, sizeof(enable), written, nullptr);
                if(rc) fail(rc);
            }
            else if(error->status) fail(error->status);
            return 0;
        }

        void subscribe_next()
        {
            if(report_index == report_count)
            {
                portENTER_CRITICAL(&lock);
                latest.connected = true;
                portEXIT_CRITICAL(&lock);
                ESP_LOGI(TAG, "Xbox input subscribed, reports=%u", report_count);
                return;
            }
            const auto &r = reports[report_index];
            const int rc = ble_gattc_disc_all_dscs(connection, r.value, r.end, descriptors, nullptr);
            if(rc) fail(rc);
        }

        int characteristics(uint16_t, const ble_gatt_error *error, const ble_gatt_chr *chr, void *)
        {
            if(!error->status)
            {
                if(report_count && !reports[report_count - 1].end)
                    reports[report_count - 1].end = chr->def_handle - 1;
                if(ble_uuid_u16(&chr->uuid.u) == 0x2a4d && (chr->properties & BLE_GATT_CHR_F_NOTIFY) && report_count < 8)
                    reports[report_count++] = {chr->val_handle, 0, 0};
            }
            else if(error->status == BLE_HS_EDONE)
            {
                if(!report_count) { fail(BLE_HS_ENOENT); return 0; }
                if(!reports[report_count - 1].end) reports[report_count - 1].end = service_end;
                report_index = 0;
                subscribe_next();
            }
            else fail(error->status);
            return 0;
        }

        int services(uint16_t, const ble_gatt_error *error, const ble_gatt_svc *svc, void *)
        {
            if(!error->status) { service_start = svc->start_handle; service_end = svc->end_handle; }
            else if(error->status == BLE_HS_EDONE)
            {
                if(!service_start) { fail(BLE_HS_ENOENT); return 0; }
                const int rc = ble_gattc_disc_all_chrs(connection, service_start, service_end, characteristics, nullptr);
                if(rc) fail(rc);
            }
            else fail(error->status);
            return 0;
        }

        void discover()
        {
            if(discovering) return;
            discovering = true;
            const ble_uuid16_t hid = BLE_UUID16_INIT(0x1812);
            const int rc = ble_gattc_disc_svc_by_uuid(connection, &hid.u, services, nullptr);
            if(rc) fail(rc);
        }

        int connection_event(ble_gap_event *event, void *)
        {
            switch(event->type)
            {
            case BLE_GAP_EVENT_CONNECT:
                connecting = false;
                if(event->connect.status) { ESP_LOGW(TAG, "connect rc=%d", event->connect.status); break; }
                connection = event->connect.conn_handle;
                discovering = false;
                service_start = service_end = report_count = report_index = 0;
                memset(reports, 0, sizeof(reports));
                { const int rc = ble_gap_security_initiate(connection); if(rc) fail(rc); }
                break;
            case BLE_GAP_EVENT_ENC_CHANGE:
                if(event->enc_change.status) fail(event->enc_change.status);
                else discover();
                break;
            case BLE_GAP_EVENT_REPEAT_PAIRING:
                {
                    ble_gap_conn_desc desc{};
                    if(ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0)
                        ble_store_util_delete_peer(&desc.peer_id_addr);
                }
                return BLE_GAP_REPEAT_PAIRING_RETRY;
            case BLE_GAP_EVENT_DISCONNECT:
                connection = BLE_HS_CONN_HANDLE_NONE;
                connecting = discovering = false;
                portENTER_CRITICAL(&lock);
                latest.connected = false;
                latest.forward = latest.turn = 0.0f;
                latest.neutral_seen = false;
                portEXIT_CRITICAL(&lock);
                ESP_LOGI(TAG, "disconnected reason=%d", event->disconnect.reason);
                break;
            case BLE_GAP_EVENT_NOTIFY_RX:
                if(OS_MBUF_PKTLEN(event->notify_rx.om) == 16)
                {
                    uint8_t data[16];
                    if(os_mbuf_copydata(event->notify_rx.om, 0, sizeof(data), data)) break;
                    const float forward = -axis(data + 2), turn = -axis(data + 4);
                    portENTER_CRITICAL(&lock);
                    latest.received_us = esp_timer_get_time();
                    latest.forward = forward;
                    latest.turn = turn;
                    if(forward == 0.0f && turn == 0.0f) latest.neutral_seen = true;
                    portEXIT_CRITICAL(&lock);
                }
                break;
            }
            return 0;
        }

        int scan_event(ble_gap_event *event, void *)
        {
            if(event->type == BLE_GAP_EVENT_DISC_COMPLETE) scanning = false;
            else if(event->type == BLE_GAP_EVENT_DISC && !connecting &&
                    !memcmp(event->disc.addr.val, ADDRESS, sizeof(ADDRESS)) &&
                    event->disc.event_type != BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND)
            {
                ble_gap_disc_cancel();
                scanning = false;
                connecting = true;
                const int rc = ble_gap_connect(own_address_type, &event->disc.addr, 10000, nullptr, connection_event, nullptr);
                if(rc) { connecting = false; ESP_LOGW(TAG, "connect start rc=%d", rc); }
            }
            return 0;
        }

        void on_sync()
        {
            if(ble_hs_id_infer_auto(0, &own_address_type) == 0) xSemaphoreGive(synced);
        }

        void host_task(void *)
        {
            nimble_port_run();
            nimble_port_freertos_deinit();
        }

        void task(void *)
        {
            xSemaphoreTake(synced, portMAX_DELAY);
            while(true)
            {
                if(connection == BLE_HS_CONN_HANDLE_NONE && !scanning && !connecting)
                {
                    ble_gap_disc_params params{};
                    params.itvl = 80;
                    params.window = 40;
                    scanning = true;
                    const int rc = ble_gap_disc(own_address_type, 3000, &params, scan_event, nullptr);
                    if(rc) scanning = false;
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }

    bool init()
    {
        esp_log_level_set("NimBLE", ESP_LOG_WARN);
        esp_log_level_set(TAG, ESP_LOG_WARN);
        ESP_ERROR_CHECK(nvs_flash_init());
        synced = xSemaphoreCreateBinary();
        if(!synced || nimble_port_init() != ESP_OK) return false;
        ble_hs_cfg.sync_cb = on_sync;
        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
        ble_store_config_init();
        ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
        ble_hs_cfg.sm_bonding = 1;
        ble_hs_cfg.sm_mitm = 0;
        nimble_port_freertos_init(host_task);
        vTaskPrioritySet(xTaskGetHandle("nimble_host"), 3);
        return xTaskCreatePinnedToCore(task, "gamepad", 4096, nullptr, 1, nullptr, 0) == pdPASS;
    }

    void get_state(state &snapshot)
    {
        portENTER_CRITICAL(&lock);
        snapshot = latest;
        portEXIT_CRITICAL(&lock);
    }
}
