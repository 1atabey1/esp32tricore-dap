#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ctrl_server.h"

static const char *TAG = "ctrl_server";

/* Defined in web_server.c: HTTP Basic against the stored web credentials.
 * It sends the 401 itself on failure. */
esp_err_t check_auth(httpd_req_t *req);

static void reboot_timer_cb(void *arg)
{
    (void)arg;
    esp_restart();
}

static esp_err_t reboot_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) {
        return ESP_OK;
    }

    /*
     * Reboot from a timer, not from here: the handler has to return so the
     * response is flushed before the stack goes away, and it must never block
     * this task, or the server stops being the thing that is always available.
     */
    static esp_timer_handle_t timer;
    if (timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = reboot_timer_cb,
            .name = "ctrl_reboot",
        };
        if (esp_timer_create(&args, &timer) != ESP_OK) {
            timer = NULL;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no timer");
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "Reboot requested over the control port");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "Rebooting...\n");

    /* Already armed by an earlier request: the reboot is coming regardless. */
    esp_timer_start_once(timer, 500 * 1000);
    return ESP_OK;
}

static const httpd_uri_t uri_reboot = {
    .uri = "/api/reboot",
    .method = HTTP_POST,
    .handler = reboot_handler,
    .user_ctx = NULL,
};

esp_err_t ctrl_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.server_port = CTRL_SERVER_PORT;
    /* Each instance also takes a listen socket and an internal control socket
     * from the lwIP pool (CONFIG_LWIP_MAX_SOCKETS); keep this one small. */
    config.max_open_sockets = 2;
    config.max_uri_handlers = 2;
    config.lru_purge_enable = true;      /* a new client evicts an idle one */
    config.recv_wait_timeout = 3;        /* a slow client can't hold the task */
    config.send_wait_timeout = 3;
    config.stack_size = 4096;
    /* Above the main server's default so this one is never starved by it. */
    config.task_priority = tskIDLE_PRIORITY + 6;
    /* ctrl_port is the UDP socket httpd uses internally; it must not collide
     * with the main instance's default. */
    config.ctrl_port = ESP_HTTPD_DEF_CTRL_PORT + 1;

    httpd_handle_t handle = NULL;
    esp_err_t ret = httpd_start(&handle, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Control server failed to start on port %d: %s",
                 CTRL_SERVER_PORT, esp_err_to_name(ret));
        return ret;
    }
    httpd_register_uri_handler(handle, &uri_reboot);
    ESP_LOGI(TAG, "Control server on port %d (POST /api/reboot)", CTRL_SERVER_PORT);
    return ESP_OK;
}
