#include "wifi_debug.h"
#include "wifi_credentials.h"
#include "test.h"
#include "hw/sensor.h"
#include "hw/motor.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

namespace wifi_debug {
namespace {
constexpr uint16_t PORT = 3333;
bool connected = false;
// All fields little endian; 112 bytes. Sensor timestamps distinguish fresh data
// from repeated network snapshots. The control loop remains 200 Hz.
struct __attribute__((packed)) packet {
    uint32_t magic = 0x31574342, sequence;
    uint64_t sent_us, imu_us;
    float pitch, roll, yaw, gx, gy, gz, speed_a, speed_b;
    float iq_a, iq_b, id_a, id_b, target_a, target_b, bus;
    uint32_t updates, max_dt_us, max_calculate_us, active, fault;
    uint32_t current_age_a_us, current_age_b_us;
};
static_assert(sizeof(packet) == 112);

void event(void *, esp_event_base_t base, int32_t id, void *data) {
    if(base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if(base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        __atomic_store_n(&connected, false, __ATOMIC_RELEASE);
        auto *e = static_cast<wifi_event_sta_disconnected_t *>(data);
        printf("WIFI disconnected reason=%u retry in 5s\n", e->reason);
        // Reconnection is performed by the network task, never in control.
    } else if(base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto *e = static_cast<ip_event_got_ip_t *>(data);
        __atomic_store_n(&connected, true, __ATOMIC_RELEASE);
        printf("WIFI connected ip=" IPSTR " tcp=%u\n", IP2STR(&e->ip_info.ip), PORT);
    }
}

void snapshot(packet &p) {
    sensor::package s{}; sensor::get_package(s);
    motor::foc_info m{}; motor::get_info(m);
    p.sent_us = esp_timer_get_time(); p.imu_us = s.imu.timestamp_us;
    p.pitch = -s.imu.angle[0]; p.roll = s.imu.angle[1]; p.yaw = s.imu.angle[2];
    p.gx = s.imu.gyro[0]; p.gy = s.imu.gyro[1]; p.gz = s.imu.gyro[2];
    p.speed_a = s.left_encoder.speed_rad_s; p.speed_b = s.right_encoder.speed_rad_s;
    p.iq_a = m.current[0].iq_A; p.iq_b = m.current[1].iq_A;
    p.id_a = m.current[0].id_A; p.id_b = m.current[1].id_A;
    p.target_a = m.target[0].iq_A; p.target_b = m.target[1].iq_A;
    p.bus = m.bus_voltage_V; p.fault = static_cast<uint32_t>(m.fault.code);
    uint32_t u, dt, calc, active; test::network_stats(u, dt, calc, active);
    p.updates = u; p.max_dt_us = dt; p.max_calculate_us = calc; p.active = active;
    p.current_age_a_us = p.sent_us - m.current[0].timestamp_us;
    p.current_age_b_us = p.sent_us - m.current[1].timestamp_us;
}

bool send_all(int fd, const void *data, size_t size) {
    auto *p = static_cast<const char *>(data);
    while(size) {
        const int n = send(fd, p, size, 0);
        if(n <= 0) return false;
        p += n; size -= n;
    }
    return true;
}

void network_task(void *) {
    int server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(server < 0) { vTaskDelete(nullptr); return; }
    int yes = 1; setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in local{}; local.sin_family = AF_INET; local.sin_port = htons(PORT);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if(bind(server, reinterpret_cast<sockaddr *>(&local), sizeof(local)) || listen(server, 1)) {
        close(server); vTaskDelete(nullptr); return;
    }
    fcntl(server, F_SETFL, O_NONBLOCK);
    int client = -1, udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(udp >= 0) {
        sockaddr_in udp_local = local; udp_local.sin_port = htons(PORT + 1);
        if(bind(udp, reinterpret_cast<sockaddr *>(&udp_local), sizeof(udp_local))) {
            close(udp); udp = -1;
        } else fcntl(udp, F_SETFL, O_NONBLOCK);
    }
    sockaddr_in peer{}, destination{};
    char line[96]{}; size_t used = 0; bool overflow = false;
    unsigned hz = 0; bool use_udp = false;
    uint32_t seq = 0, dropped = 0, missed = 0;
    uint64_t due = 0, retry = esp_timer_get_time() + 5000000;
    while(true) {
        uint64_t now = esp_timer_get_time();
        if(!__atomic_load_n(&connected, __ATOMIC_ACQUIRE)) {
            if(client >= 0) { close(client); client = -1; hz = 0; }
            if(now >= retry) { esp_wifi_connect(); retry = now + 5000000; }
            vTaskDelay(pdMS_TO_TICKS(100)); continue;
        }
        if(client < 0) {
            socklen_t len = sizeof(peer);
            client = accept(server, reinterpret_cast<sockaddr *>(&peer), &len);
            if(client >= 0) {
                setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
                timeval timeout{0, 200000};
                setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
                used = 0; overflow = false; hz = 0;
            }
        }
        bool failed = false;
        if(udp >= 0) {
            // Client primes its firewall/NAT path to UDP 3334 before subscribing.
            char probe[96]; sockaddr_in from{}; socklen_t size = sizeof(from);
            recvfrom(udp, probe, sizeof(probe), 0, reinterpret_cast<sockaddr *>(&from), &size);
        }
        if(client >= 0) {
            char input[128]; const int n = recv(client, input, sizeof(input), MSG_DONTWAIT);
            if(n == 0 || (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN)) failed = true;
            for(int i = 0; i < n && !failed; ++i) {
                const char c = input[i];
                if(c != '\n' && c != '\r') {
                    if(used + 1 < sizeof(line)) line[used++] = c;
                    else overflow = true;
                    continue;
                }
                if(!used && !overflow) continue;
                line[used] = 0; used = 0;
                char reply[768]{};
                if(overflow) { snprintf(reply, sizeof(reply), "REJECT line too long\n"); overflow = false; }
                else if(strncmp(line, "PING ", 5) == 0) snprintf(reply, sizeof(reply), "PONG %s\n", line + 5);
                else if(strcmp(line, "NET") == 0) {
                    wifi_ap_record_t ap{}; esp_wifi_sta_get_ap_info(&ap);
                    snprintf(reply, sizeof(reply), "NET rssi=%d hz=%u bytes=%u attempts=%lu drops=%lu missed=%lu\n",
                        ap.rssi, hz, unsigned(sizeof(packet)), (unsigned long)seq,
                        (unsigned long)dropped, (unsigned long)missed);
                } else if(strncmp(line, "STREAM ", 7) == 0) {
                    char transport[8]{}; unsigned rate = 0, port = 3334;
                    int fields = sscanf(line, "STREAM %7s %u %u", transport, &rate, &port);
                    if(fields < 2 || rate > 200 || port == 0 || port > 65535 ||
                       (strcmp(transport, "TCP") && strcmp(transport, "UDP")) ||
                       (!strcmp(transport, "UDP") && udp < 0)) {
                        snprintf(reply, sizeof(reply), "REJECT STREAM TCP|UDP 0..200 [udp_port]\n");
                    } else {
                        hz = rate; use_udp = !strcmp(transport, "UDP");
                        destination = peer; destination.sin_port = htons(port);
                        seq = dropped = missed = 0; due = esp_timer_get_time() + 10000;
                        snprintf(reply, sizeof(reply), "STREAM %s %u bytes=%u\n", transport, rate, unsigned(sizeof(packet)));
                    }
                } else if(!test::network_command(line, reply, sizeof(reply))) {
                    snprintf(reply, sizeof(reply), "REJECT console busy\n");
                }
                failed = !send_all(client, reply, strlen(reply));
            }
            now = esp_timer_get_time();
            if(!failed && hz && now >= due) {
                const uint32_t interval = 1000000U / hz;
                if(now - due >= interval) { missed += (now - due) / interval; due = now; }
                due += interval;
                packet p{}; p.sequence = seq++; snapshot(p);
                if(use_udp) {
                    if(sendto(udp, &p, sizeof(p), 0, reinterpret_cast<sockaddr *>(&destination), sizeof(destination)) != sizeof(p)) ++dropped;
                } else failed = !send_all(client, &p, sizeof(p));
            }
        }
        if(failed) { close(client); client = -1; hz = 0; }
        // No disconnect-triggered STOP: balancing remains autonomous on the car.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
}

bool init() {
    esp_err_t err = nvs_flash_init();
    // Preserve NVS calibration/configuration rather than silently erasing it.
    if(err != ESP_OK) { printf("WIFI nvs init failed %s\n", esp_err_to_name(err)); return false; }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event, nullptr));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t config{};
    snprintf(reinterpret_cast<char *>(config.sta.ssid), sizeof(config.sta.ssid), "%s", DEBUG_WIFI_SSID);
    snprintf(reinterpret_cast<char *>(config.sta.password), sizeof(config.sta.password), "%s", DEBUG_WIFI_PASSWORD);
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    uint8_t mac[6]{}; esp_wifi_get_mac(WIFI_IF_STA, mac);
    printf("WIFI STA_MAC=%02X:%02X:%02X:%02X:%02X:%02X ssid=%s\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], DEBUG_WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    return xTaskCreatePinnedToCore(network_task, "wifi_debug", 6144, nullptr, 1, nullptr, 0) == pdPASS;
}
}
