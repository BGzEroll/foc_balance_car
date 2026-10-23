#include "ota.h"
#include "ota_credentials.h"
#include "test.h"
#include "wifi_debug.h"
#include "hw/motor.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "psa/crypto.h"
#include <stdio.h>
#include <string.h>

namespace ota {
namespace {
constexpr int64_t CONFIRM_US = 60000000;
constexpr char SCHEMA[] = "current_pid_v1";
uint32_t pending = 0, app_ready = 0, staged = 0, restarting = 0;
int64_t boot_us = 0;
const esp_partition_t *target = nullptr;
char staged_sha[65]{};
bool boot_pending() { return __atomic_load_n(&pending,__ATOMIC_ACQUIRE)!=0; }

esp_err_t json(httpd_req_t *r, const char *body, const char *status = "200 OK") {
    httpd_resp_set_status(r,status);
    httpd_resp_set_type(r,"application/json");
    httpd_resp_set_hdr(r,"Cache-Control","no-store");
    httpd_resp_set_hdr(r,"Connection","close");
    return httpd_resp_sendstr(r,body);
}
bool header(httpd_req_t *r,const char *name,char *value,size_t capacity) {
    return httpd_req_get_hdr_value_len(r,name) < capacity &&
           httpd_req_get_hdr_value_str(r,name,value,capacity)==ESP_OK;
}
bool authorized(httpd_req_t *r) {
    char value[65]{};
    bool ok=header(r,"X-OTA-Token",value,sizeof(value));
    unsigned diff=0;
    static_assert(sizeof(OTA_TOKEN)==65,"OTA token must contain 64 hex characters");
    for(unsigned i=0;i<64;++i) diff |= static_cast<unsigned char>(value[i]^OTA_TOKEN[i]);
    if(!ok || diff) { json(r,"{\"error\":\"unauthorized\"}","401 Unauthorized"); return false; }
    return true;
}
bool healthy() {
    test::telemetry t{}; test::get_telemetry(t);
    motor::foc_info m{}; motor::get_info(m);
    const uint64_t now=esp_timer_get_time();
    return __atomic_load_n(&app_ready,__ATOMIC_ACQUIRE) && wifi_debug::is_ready() &&
           m.initialized && !m.running && !m.ready && !m.fault.active &&
           t.cycle>100 && t.sample_us && now-t.sample_us<100000 &&
           t.imu_age_us<100000 && t.encoder_age_us<20000 && t.mode==0;
}
void hex(const uint8_t *bytes,char *out,size_t count) {
    const char digits[]="0123456789abcdef";
    for(size_t i=0;i<count;++i) { out[2*i]=digits[bytes[i]>>4];out[2*i+1]=digits[bytes[i]&15]; }
    out[2*count]=0;
}
esp_err_t status(httpd_req_t *r) {
    if(!authorized(r)) return ESP_OK;
    const esp_partition_t *running=esp_ota_get_running_partition(), *boot=esp_ota_get_boot_partition();
    esp_ota_img_states_t state=ESP_OTA_IMG_UNDEFINED;
    esp_ota_get_state_partition(running,&state);
    char elf[65];hex(esp_app_get_description()->app_elf_sha256,elf,32);
    char body[768];
    snprintf(body,sizeof(body),
        "{\"schema\":\"%s\",\"running\":\"%s\",\"boot\":\"%s\",\"state\":%u,"
        "\"pending_verify\":%s,\"healthy\":%s,\"locked\":%s,\"motor_ack\":%s,"
        "\"staged\":%s,\"target\":\"%s\",\"staged_sha256\":\"%s\",\"elf_sha256\":\"%s\","
        "\"confirm_remaining_ms\":%lld,\"uptime_ms\":%lld}",
        SCHEMA,running?running->label:"",boot?boot->label:"",static_cast<unsigned>(state),
        __atomic_load_n(&pending,__ATOMIC_ACQUIRE)?"true":"false",healthy()?"true":"false",
        motor::update_locked()?"true":"false",motor::update_stopped()?"true":"false",
        staged?"true":"false",target?target->label:"",staged_sha,elf,
        boot_pending() ? static_cast<long long>((CONFIRM_US-(esp_timer_get_time()-boot_us))/1000) : 0LL,
        static_cast<long long>(esp_timer_get_time()/1000));
    return json(r,body);
}
void release_lock() {
    // The control task has discarded its queued run before upload completes/aborts.
    staged=0;target=nullptr;staged_sha[0]=0;
    motor::set_update_lock(false);
}
esp_err_t upload(httpd_req_t *r) {
    if(!authorized(r)) return ESP_OK;
    if(boot_pending() || staged || restarting) return json(r,"{\"error\":\"confirm or cancel current update first\"}","409 Conflict");
    char wanted[65]{},schema[32]{};
    if(!header(r,"X-Image-SHA256",wanted,sizeof(wanted)) || strlen(wanted)!=64 ||
       strspn(wanted,"0123456789abcdef")!=64 || !header(r,"X-Image-Schema",schema,sizeof(schema)) || strcmp(schema,SCHEMA))
        return json(r,"{\"error\":\"SHA256 or schema header\"}","400 Bad Request");
    target=esp_ota_get_next_update_partition(nullptr);
    if(!target || r->content_len<1024 || static_cast<size_t>(r->content_len)>target->size) {
        target=nullptr;return json(r,"{\"error\":\"image size or partition\"}","413 Content Too Large");
    }
    motor::set_update_lock(true);
    // Wait for both cores to observe STOP before calling any flash API.
    const int64_t deadline=esp_timer_get_time()+2000000;
    while(esp_timer_get_time()<deadline) {
        test::telemetry t{};test::get_telemetry(t);
        if(motor::update_stopped() && t.mode==0 && !(t.flags&6U)) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    motor::foc_info m{};motor::get_info(m);
    if(!motor::update_stopped() || m.running || m.ready) {
        release_lock();return json(r,"{\"error\":\"STOP acknowledgement timeout\"}","409 Conflict");
    }
    esp_ota_handle_t handle=0;
    esp_err_t err=esp_ota_begin(target,OTA_WITH_SEQUENTIAL_WRITES,&handle);
    if(err!=ESP_OK) { release_lock();return json(r,"{\"error\":\"ota begin\"}","500 Internal Server Error"); }
    psa_hash_operation_t sha=PSA_HASH_OPERATION_INIT;
    if(psa_hash_setup(&sha,PSA_ALG_SHA_256)!=PSA_SUCCESS) {
        esp_ota_abort(handle);release_lock();return json(r,"{\"error\":\"hash init\"}","500 Internal Server Error");
    }
    uint8_t buffer[4096],prefix[sizeof(esp_image_header_t)+sizeof(esp_image_segment_header_t)+sizeof(esp_app_desc_t)]{};
    size_t received=0,prefix_used=0;
    // In addition to the socket timeout, cap an authenticated slow upload at 120 s.
    const int64_t upload_deadline=esp_timer_get_time()+120000000;
    bool good=true;
    while(received<static_cast<size_t>(r->content_len)) {
        int n=httpd_req_recv(r,reinterpret_cast<char *>(buffer),
            (static_cast<size_t>(r->content_len)-received)>sizeof(buffer)?sizeof(buffer):r->content_len-received);
        if(n<=0 || esp_timer_get_time()>upload_deadline) { good=false;break; }
        if(prefix_used<sizeof(prefix)) {
            size_t take=sizeof(prefix)-prefix_used;if(take>static_cast<size_t>(n)) take=n;
            memcpy(prefix+prefix_used,buffer,take);prefix_used+=take;
        }
        if(psa_hash_update(&sha,buffer,n)!=PSA_SUCCESS) { good=false;break; }
        if(esp_ota_write(handle,buffer,n)!=ESP_OK) { good=false;break; }
        received+=n;
    }
    uint8_t digest[32]{};char actual[65];size_t digest_size=0;
    if(psa_hash_finish(&sha,digest,sizeof(digest),&digest_size)!=PSA_SUCCESS || digest_size!=32) good=false;
    psa_hash_abort(&sha);hex(digest,actual,32);
    esp_app_desc_t description{};
    if(prefix_used==sizeof(prefix)) memcpy(&description,prefix+sizeof(esp_image_header_t)+sizeof(esp_image_segment_header_t),sizeof(description));
    good=good && received==static_cast<size_t>(r->content_len) && !strcmp(actual,wanted) &&
        description.magic_word==ESP_APP_DESC_MAGIC_WORD &&
        !strncmp(description.project_name,"balance_car",sizeof(description.project_name)) &&
        !strncmp(description.version,"current_pid_v1-",15);
    if(!good) {
        esp_ota_abort(handle);release_lock();
        json(r,"{\"error\":\"incomplete, hash mismatch or incompatible image\"}","422 Unprocessable Entity");
        return ESP_FAIL; // Close the connection; unread request bytes must not be reused.
    }
    err=esp_ota_end(handle); // Official image/checksum/chip validation; handle consumed even on failure.
    if(err!=ESP_OK) {
        release_lock();return json(r,"{\"error\":\"image validation failed\"}","422 Unprocessable Entity");
    }
    // No boot selection is changed by upload; explicit reboot commits it later.
    staged=1;snprintf(staged_sha,sizeof(staged_sha),"%s",actual);
    return json(r,"{\"ok\":true,\"staged\":true,\"locked\":true}");
}
void reboot_task(void *) { ulTaskNotifyTake(pdTRUE,portMAX_DELAY);vTaskDelay(pdMS_TO_TICKS(500));esp_restart(); }
esp_err_t reboot(httpd_req_t *r) {
    if(!authorized(r)) return ESP_OK;
    if(!staged || !target || boot_pending() || restarting) return json(r,"{\"error\":\"no staged image\"}","409 Conflict");
    // Create the deferred restart before committing boot metadata. If allocation fails, don't commit.
    TaskHandle_t task=nullptr;
    if(xTaskCreatePinnedToCore(reboot_task,"ota_reboot",3072,nullptr,1,&task,0)!=pdPASS)
        return json(r,"{\"error\":\"restart task\"}","500 Internal Server Error");
    esp_err_t err=esp_ota_set_boot_partition(target);
    if(err!=ESP_OK) { vTaskDelete(task);return json(r,"{\"error\":\"boot selection failed\"}","500 Internal Server Error"); }
    restarting=1;
    xTaskNotifyGive(task);
    return json(r,"{\"ok\":true,\"rebooting\":true}");
}
esp_err_t confirm(httpd_req_t *r) {
    if(!authorized(r)) return ESP_OK;
    if(!boot_pending() || !healthy() || !motor::update_stopped())
        return json(r,"{\"error\":\"not pending or health check failed\"}","409 Conflict");
    uint32_t expected=1;
    if(!__atomic_compare_exchange_n(&pending,&expected,3U,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))
        return json(r,"{\"error\":\"confirmation deadline expired\"}","409 Conflict");
    esp_err_t err=esp_ota_mark_app_valid_cancel_rollback();
    if(err!=ESP_OK) {
        __atomic_store_n(&pending,1U,__ATOMIC_RELEASE);
        return json(r,"{\"error\":\"mark valid failed\"}","500 Internal Server Error");
    }
    __atomic_store_n(&pending,0U,__ATOMIC_RELEASE);release_lock();
    return json(r,"{\"ok\":true,\"confirmed\":true}");
}
esp_err_t cancel(httpd_req_t *r) {
    if(!authorized(r)) return ESP_OK;
    if(boot_pending() || restarting) return json(r,"{\"error\":\"pending boot cannot be unlocked\"}","409 Conflict");
    release_lock();return json(r,"{\"ok\":true,\"cancelled\":true}");
}
void guard_task(void *) {
    while(true) {
        uint32_t expected=1;
        if(esp_timer_get_time()-boot_us>=CONFIRM_US &&
           __atomic_compare_exchange_n(&pending,&expected,2U,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE)) {
            printf("OTA confirmation timeout; rolling back with motors inhibited\n");
            esp_ota_mark_app_invalid_rollback_and_reboot();
            // If no fallback exists, remain locked. Never promote an unhealthy image.
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
}
void prepare_boot() {
    boot_us=esp_timer_get_time();
    esp_ota_img_states_t state;
    if(esp_ota_get_state_partition(esp_ota_get_running_partition(),&state)==ESP_OK && state==ESP_OTA_IMG_PENDING_VERIFY) {
        __atomic_store_n(&pending,1U,__ATOMIC_RELEASE);motor::set_update_lock(true);
        if(xTaskCreatePinnedToCore(guard_task,"ota_guard",4096,nullptr,2,nullptr,0)!=pdPASS)
            esp_ota_mark_app_invalid_rollback_and_reboot();
    }
}
bool init(bool application_ready) {
    __atomic_store_n(&app_ready,application_ready?1U:0U,__ATOMIC_RELEASE);
    if(!application_ready) return false;
    httpd_config_t config=HTTPD_DEFAULT_CONFIG();config.server_port=3335;config.ctrl_port=3336;
    config.core_id=0;config.stack_size=12288;config.max_open_sockets=3;
    config.recv_wait_timeout=3;config.send_wait_timeout=3;config.lru_purge_enable=true;
    httpd_handle_t server=nullptr;
    if(httpd_start(&server,&config)!=ESP_OK) return false;
    const struct { const char *path;httpd_method_t method;esp_err_t(*fn)(httpd_req_t *); } routes[]={
        {"/ota/status",HTTP_GET,status},{"/ota/upload",HTTP_POST,upload},{"/ota/reboot",HTTP_POST,reboot},
        {"/ota/confirm",HTTP_POST,confirm},{"/ota/cancel",HTTP_POST,cancel}};
    for(const auto &route:routes) {
        httpd_uri_t uri{};uri.uri=route.path;uri.method=route.method;uri.handler=route.fn;
        if(httpd_register_uri_handler(server,&uri)!=ESP_OK) { httpd_stop(server);return false; }
    }
    printf("OTA ready port=3335 pending=%u confirm_timeout=60s\n",boot_pending());
    return true;
}
}
