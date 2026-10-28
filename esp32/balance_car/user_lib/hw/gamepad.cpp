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
        constexpr uint8_t ADDRESS[] = {0x76, 0x99, 0x01, 0x58, 0x0B, 0xFC};
        constexpr float DEADZONE = 0.03f;
        constexpr const char *TAG = "gamepad";

        struct report
        {
            uint16_t value_handle = 0;
            uint16_t end_handle = 0;
            uint16_t ccc_handle = 0;
        };

        portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
        state latest{};
        SemaphoreHandle_t synced = nullptr;
        uint8_t own_address_type = 0;
        uint16_t connection = BLE_HS_CONN_HANDLE_NONE;
        uint16_t service_start = 0;
        uint16_t service_end = 0;
        uint8_t report_count = 0;
        uint8_t report_index = 0;
        report reports[8]{};

        bool scanning = false;
        bool connecting = false;
        bool discovering = false;

        /**
         * @brief 将无符号摇杆读数转换为带连续死区的正负单位值
         */
        float axis(const uint8_t *data)
        {
            const uint16_t raw = data[0] | static_cast<uint16_t>(data[1]) << 8;
            const float value = (static_cast<int32_t>(raw) - 32768) / 32768.0f;

            if(fabsf(value) <= DEADZONE)
            {
                return 0.0f;
            }

            const float magnitude = fminf(1.0f, (fabsf(value) - DEADZONE) / (1.0f - DEADZONE));
            return copysignf(magnitude, value);
        }

        /**
         * @brief 结束失败的输入订阅流程，交由连接任务重新连接
         */
        void fail(int result)
        {
            ESP_LOGE(TAG, "GATT setup failed rc=%d", result);
            ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
        }

        void subscribe_next();

        /**
         * @brief 当前报告通知开启后继续订阅下一报告
         */
        int written(uint16_t, const ble_gatt_error *error, ble_gatt_attr *, void *)
        {
            if(error->status)
            {
                fail(error->status);
                return 0;
            }

            ++report_index;
            subscribe_next();
            return 0;
        }

        /**
         * @brief 查找当前输入报告的通知描述符并异步写入
         */
        int descriptors(
            uint16_t, const ble_gatt_error *error, uint16_t,
            const ble_gatt_dsc *descriptor, void *)
        {
            if(!error->status && ble_uuid_u16(&descriptor->uuid.u) == 0x2902)
            {
                reports[report_index].ccc_handle = descriptor->handle;
            }
            else if(error->status == BLE_HS_EDONE)
            {
                const report &current = reports[report_index];

                if(!current.ccc_handle)
                {
                    ++report_index;
                    subscribe_next();
                    return 0;
                }

                const uint8_t enable[] = {1, 0};
                const int result = ble_gattc_write_flat(
                    connection, current.ccc_handle, enable, sizeof(enable), written, nullptr);

                if(result)
                {
                    fail(result);
                }
            }
            else if(error->status)
            {
                fail(error->status);
            }

            return 0;
        }

        /**
         * @brief 按顺序完成输入报告订阅，不阻塞 NimBLE 回调任务
         */
        void subscribe_next()
        {
            if(report_index == report_count)
            {
                portENTER_CRITICAL(&lock);
                latest.connected = true;
                portEXIT_CRITICAL(&lock);
                return;
            }

            const report &current = reports[report_index];
            const int result = ble_gattc_disc_all_dscs(
                connection, current.value_handle, current.end_handle, descriptors, nullptr);

            if(result)
            {
                fail(result);
            }
        }

        /**
         * @brief 收集 HID 服务中支持通知的输入报告
         */
        int characteristics(
            uint16_t, const ble_gatt_error *error, const ble_gatt_chr *characteristic, void *)
        {
            if(!error->status)
            {
                if(report_count && !reports[report_count - 1].end_handle)
                {
                    reports[report_count - 1].end_handle = characteristic->def_handle - 1;
                }

                if(ble_uuid_u16(&characteristic->uuid.u) == 0x2A4D &&
                   (characteristic->properties & BLE_GATT_CHR_F_NOTIFY) &&
                   report_count < sizeof(reports) / sizeof(reports[0]))
                {
                    reports[report_count++] = {characteristic->val_handle, 0, 0};
                }
            }
            else if(error->status == BLE_HS_EDONE)
            {
                if(!report_count)
                {
                    fail(BLE_HS_ENOENT);
                    return 0;
                }

                if(!reports[report_count - 1].end_handle)
                {
                    reports[report_count - 1].end_handle = service_end;
                }

                report_index = 0;
                subscribe_next();
            }
            else
            {
                fail(error->status);
            }

            return 0;
        }

        /**
         * @brief HID 服务发现完成后继续发现输入特征
         */
        int services(uint16_t, const ble_gatt_error *error, const ble_gatt_svc *service, void *)
        {
            if(!error->status)
            {
                service_start = service->start_handle;
                service_end = service->end_handle;
            }
            else if(error->status == BLE_HS_EDONE)
            {
                if(!service_start)
                {
                    fail(BLE_HS_ENOENT);
                    return 0;
                }

                const int result = ble_gattc_disc_all_chrs(
                    connection, service_start, service_end, characteristics, nullptr);

                if(result)
                {
                    fail(result);
                }
            }
            else
            {
                fail(error->status);
            }

            return 0;
        }

        /**
         * @brief 链路加密完成后启动一次 HID 服务发现
         */
        void discover()
        {
            if(discovering)
            {
                return;
            }

            discovering = true;
            const ble_uuid16_t hid = BLE_UUID16_INIT(0x1812);
            const int result = ble_gattc_disc_svc_by_uuid(connection, &hid.u, services, nullptr);

            if(result)
            {
                fail(result);
            }
        }

        /**
         * @brief 处理连接、配对和 Xbox 输入通知
         */
        int connection_event(ble_gap_event *event, void *)
        {
            switch(event->type)
            {
            case BLE_GAP_EVENT_CONNECT:
                {
                    connecting = false;

                    if(event->connect.status)
                    {
                        ESP_LOGW(TAG, "connect rc=%d", event->connect.status);
                        break;
                    }

                    connection = event->connect.conn_handle;
                    discovering = false;
                    service_start = 0;
                    service_end = 0;
                    report_count = 0;
                    report_index = 0;
                    memset(reports, 0, sizeof(reports));
                    const int result = ble_gap_security_initiate(connection);

                    if(result)
                    {
                        fail(result);
                    }
                }
                break;

            case BLE_GAP_EVENT_ENC_CHANGE:
                if(event->enc_change.status)
                {
                    fail(event->enc_change.status);
                }
                else
                {
                    discover();
                }
                break;

            case BLE_GAP_EVENT_REPEAT_PAIRING:
                {
                    ble_gap_conn_desc description{};

                    if(ble_gap_conn_find(event->repeat_pairing.conn_handle, &description) == 0)
                    {
                        ble_store_util_delete_peer(&description.peer_id_addr);
                    }
                }
                return BLE_GAP_REPEAT_PAIRING_RETRY;

            case BLE_GAP_EVENT_DISCONNECT:
                connection = BLE_HS_CONN_HANDLE_NONE;
                connecting = false;
                discovering = false;

                portENTER_CRITICAL(&lock);
                latest.connected = false;
                latest.forward = 0.0f;
                latest.turn = 0.0f;
                latest.neutral_seen = false;
                portEXIT_CRITICAL(&lock);
                break;

            case BLE_GAP_EVENT_NOTIFY_RX:
                if(OS_MBUF_PKTLEN(event->notify_rx.om) == 16)
                {
                    uint8_t data[16];

                    if(os_mbuf_copydata(event->notify_rx.om, 0, sizeof(data), data))
                    {
                        break;
                    }

                    const float forward = -axis(data + 2);
                    const float turn = -axis(data + 4);

                    portENTER_CRITICAL(&lock);
                    latest.received_us = esp_timer_get_time();
                    latest.forward = forward;
                    latest.turn = turn;

                    if(forward == 0.0f && turn == 0.0f)
                    {
                        latest.neutral_seen = true;
                    }

                    portEXIT_CRITICAL(&lock);
                }
                break;
            }

            return 0;
        }

        /**
         * @brief 扫描指定地址的手柄并异步建立连接
         */
        int scan_event(ble_gap_event *event, void *)
        {
            if(event->type == BLE_GAP_EVENT_DISC_COMPLETE)
            {
                scanning = false;
            }
            else if(event->type == BLE_GAP_EVENT_DISC && !connecting &&
                    !memcmp(event->disc.addr.val, ADDRESS, sizeof(ADDRESS)) &&
                    event->disc.event_type != BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND)
            {
                ble_gap_disc_cancel();
                scanning = false;
                connecting = true;
                const int result = ble_gap_connect(
                    own_address_type, &event->disc.addr, 10000, nullptr, connection_event, nullptr);

                if(result)
                {
                    connecting = false;
                    ESP_LOGW(TAG, "connect start rc=%d", result);
                }
            }

            return 0;
        }

        /**
         * @brief 通知连接任务蓝牙地址已就绪
         */
        void on_sync()
        {
            if(ble_hs_id_infer_auto(0, &own_address_type) == 0)
            {
                xSemaphoreGive(synced);
            }
        }

        void host_task(void *)
        {
            nimble_port_run();
            nimble_port_freertos_deinit();
        }

        /**
         * @brief 在 Core 0 管理扫描与重连
         */
        void task(void *)
        {
            xSemaphoreTake(synced, portMAX_DELAY);

            while(true)
            {
                if(connection == BLE_HS_CONN_HANDLE_NONE && !scanning && !connecting)
                {
                    ble_gap_disc_params parameters{};
                    parameters.itvl = 80;
                    parameters.window = 40;
                    scanning = true;
                    const int result = ble_gap_disc(own_address_type, 3000, &parameters, scan_event, nullptr);

                    if(result)
                    {
                        scanning = false;
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }

    /**
     * @brief 初始化 NimBLE 并创建 Core 0 手柄连接任务
     *
     * @return true 初始化完成且任务已创建
     * @return false 初始化或任务创建失败
     */
    bool init()
    {
        esp_log_level_set("NimBLE", ESP_LOG_WARN);
        esp_log_level_set(TAG, ESP_LOG_WARN);
        ESP_ERROR_CHECK(nvs_flash_init());
        synced = xSemaphoreCreateBinary();

        if(!synced || nimble_port_init() != ESP_OK)
        {
            return false;
        }

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

    /**
     * @brief 获取归一化摇杆输入与连接状态快照
     */
    void get_state(state &snapshot)
    {
        portENTER_CRITICAL(&lock);
        snapshot = latest;
        portEXIT_CRITICAL(&lock);
    }
}
